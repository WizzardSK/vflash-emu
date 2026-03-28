/* JIT compiler: ARM926EJ-S → x86_64 basic block translation.
 * Translates ARM basic blocks to native x86_64 code on demand.
 * Falls back to interpreter for complex/unsupported instructions. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include "jit.h"
#include "jit_emit.h"
#include "arm9.h"
#include "vflash.h"

/* Block cache size */
#define JIT_CACHE_SIZE  (16 * 1024 * 1024)  /* 16MB code cache */
#define JIT_MAX_BLOCKS  65536
#define JIT_BLOCK_HASH(pc) (((pc) >> 2) & (JIT_MAX_BLOCKS - 1))
#define JIT_MAX_BLOCK_INSNS 64

/* ARM register offsets in ARM9 struct */
#define ARM_R(n) ((int)__builtin_offsetof(ARM9, r[n]))
#define ARM_CPSR ((int)__builtin_offsetof(ARM9, cpsr))
#define ARM_PC   ARM_R(15)

/* x86_64 register assignments:
 * RBX = ARM9 *cpu pointer (callee-saved, persistent)
 * R12-R15 are callee-saved but we use them as scratch
 * RAX, RCX, RDX, RSI, RDI = scratch / call args */
#define REG_CPU  RBX  /* ARM9 *cpu — persistent */

/* Memory access through VFlash accessors */
static uint32_t jit_mem_read32(VFlash *vf, uint32_t addr) {
    ARM9 *cpu = (ARM9 *)vflash_get_cpu(vf);
    return cpu->mem_read32(cpu->mem_ctx, addr);
}
static void jit_mem_write32(VFlash *vf, uint32_t addr, uint32_t val) {
    ARM9 *cpu = (ARM9 *)vflash_get_cpu(vf);
    cpu->mem_write32(cpu->mem_ctx, addr, val);
}

/* ---- Block Cache ---- */

JitContext *jit_create(VFlash *vf) {
    JitContext *jit = calloc(1, sizeof(JitContext));
    if (!jit) return NULL;

    jit->code_cache = mmap(NULL, JIT_CACHE_SIZE,
                           PROT_READ | PROT_WRITE | PROT_EXEC,
                           MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (jit->code_cache == MAP_FAILED) {
        free(jit);
        return NULL;
    }
    jit->cache_size = JIT_CACHE_SIZE;
    jit->cache_used = 0;

    jit->blocks = calloc(JIT_MAX_BLOCKS, sizeof(JitBlock));
    jit->block_capacity = JIT_MAX_BLOCKS;
    jit->vf = vf;

    printf("[JIT] Created: %dMB code cache, %d block slots\n",
           JIT_CACHE_SIZE / (1024*1024), JIT_MAX_BLOCKS);
    /* Stats logged periodically from jit_run */
    return jit;
}

void jit_destroy(JitContext *jit) {
    if (!jit) return;
    if (jit->code_cache) munmap(jit->code_cache, jit->cache_size);
    free(jit->blocks);
    printf("[JIT] Destroyed: %lu blocks compiled, %lu executed, %lu insns\n",
           jit->blocks_compiled, jit->blocks_executed, jit->insns_executed);
    free(jit);
}

void jit_flush(JitContext *jit) {
    memset(jit->blocks, 0, jit->block_capacity * sizeof(JitBlock));
    jit->cache_used = 0;
    jit->block_count = 0;
    memset(jit->code_pages, 0, sizeof(jit->code_pages));
}

void jit_invalidate_page(JitContext *jit, uint32_t page_idx) {
    if (page_idx >= 4096) return;
    if (!jit->code_pages[page_idx]) return;
    /* Simple: flush ALL blocks. Could be smarter. */
    jit_flush(jit);
}

JitBlock *jit_lookup(JitContext *jit, uint32_t arm_pc) {
    uint32_t idx = JIT_BLOCK_HASH(arm_pc);
    for (uint32_t i = 0; i < 4; i++) { /* linear probe, 4 slots */
        uint32_t slot = (idx + i) & (JIT_MAX_BLOCKS - 1);
        if (jit->blocks[slot].arm_pc == arm_pc && jit->blocks[slot].native_code)
            return &jit->blocks[slot];
        if (!jit->blocks[slot].native_code)
            return NULL; /* empty slot = not found */
    }
    return NULL;
}

static JitBlock *jit_insert(JitContext *jit, uint32_t arm_pc, uint8_t *code,
                            uint32_t code_size, uint32_t insn_count) {
    uint32_t idx = JIT_BLOCK_HASH(arm_pc);
    for (uint32_t i = 0; i < 4; i++) {
        uint32_t slot = (idx + i) & (JIT_MAX_BLOCKS - 1);
        if (!jit->blocks[slot].native_code) {
            jit->blocks[slot].arm_pc = arm_pc;
            jit->blocks[slot].native_code = code;
            jit->blocks[slot].native_size = code_size;
            jit->blocks[slot].arm_insn_count = insn_count;
            jit->block_count++;
            return &jit->blocks[slot];
        }
    }
    return NULL; /* cache full at this hash */
}

/* ---- ARM → x86_64 Compiler ---- */

/* Emit: load ARM register Rn into x86 reg dst */
static void emit_load_arm_reg(EmitBuf *e, int dst, int arm_reg) {
    emit_ldr_disp32(e, dst, REG_CPU, ARM_R(arm_reg));
}

/* Emit: store x86 reg src into ARM register Rn */
static void emit_store_arm_reg(EmitBuf *e, int arm_reg, int src) {
    emit_str_disp32(e, REG_CPU, src, ARM_R(arm_reg));
}

/* Emit: compute barrel shifter operand2 (immediate) into x86 reg dst.
 * Returns the immediate value. */
static uint32_t arm_expand_imm(uint32_t insn) {
    uint32_t imm8 = insn & 0xFF;
    uint32_t rot = ((insn >> 8) & 0xF) * 2;
    if (rot == 0) return imm8;
    return (imm8 >> rot) | (imm8 << (32 - rot));
}

/* Check if instruction modifies PC (branch or data-processing with Rd=15) */
static int arm_modifies_pc(uint32_t insn) {
    uint32_t cond = (insn >> 28) & 0xF;
    if (cond == 0xF) return 1; /* NV = special */

    uint32_t op = (insn >> 25) & 7;
    if (op == 5) return 1; /* B/BL */
    if (op == 0 || op == 1) {
        /* Data processing — check Rd */
        int Rd = (insn >> 12) & 0xF;
        if (Rd == 15) return 1;
        /* BX/BLX */
        if ((insn & 0x0FFFFFF0) == 0x012FFF10) return 1; /* BX */
        if ((insn & 0x0FFFFFF0) == 0x012FFF30) return 1; /* BLX */
    }
    if (op == 2 || op == 3) {
        /* LDR/STR — check if LDR Rd=PC */
        int L = (insn >> 20) & 1;
        int Rd = (insn >> 12) & 0xF;
        if (L && Rd == 15) return 1;
    }
    if (op == 4) {
        /* LDM/STM — check if PC in register list */
        int L = (insn >> 20) & 1;
        if (L && (insn & (1 << 15))) return 1;
    }
    return 0;
}

/* Compile a single ARM data processing instruction to x86_64.
 * Returns 1 if compiled, 0 if needs interpreter fallback. */
static int compile_data_processing(EmitBuf *e, uint32_t insn, uint32_t pc) {
    int I = (insn >> 25) & 1;    /* immediate operand2 */
    int opcode = (insn >> 21) & 0xF;
    int S = (insn >> 20) & 1;
    int Rn = (insn >> 16) & 0xF;
    int Rd = (insn >> 12) & 0xF;

    if (Rd == 15) return 0; /* PC as destination — fallback */

    /* Load Rn into EAX (use PC+8 if Rn==15) */
    if (opcode != 13 && opcode != 15) { /* MOV/MVN don't use Rn */
        if (Rn == 15)
            emit_mov_r32_imm32(e, RAX, pc + 8);
        else
            emit_load_arm_reg(e, RAX, Rn);
    }

    /* Compute operand2 into ECX */
    if (I) {
        uint32_t imm = arm_expand_imm(insn);
        emit_mov_r32_imm32(e, RCX, imm);
    } else {
        int Rm = insn & 0xF;
        int shift_type = (insn >> 5) & 3;
        int shift_imm = (insn >> 7) & 0x1F;
        int shift_by_reg = (insn >> 4) & 1;

        if (Rm == 15)
            emit_mov_r32_imm32(e, RCX, pc + 8);
        else
            emit_load_arm_reg(e, RCX, Rm);

        if (shift_by_reg) {
            /* Register-controlled shift: shift amount in Rs */
            int Rs = (insn >> 8) & 0xF;
            if (Rs == 15) return 0;
            /* x86 shifts use CL for variable shift amount */
            emit_push(e, RCX); /* save operand */
            emit_load_arm_reg(e, RCX, Rs); /* CL = shift amount */
            emit_and_r32_imm32(e, RCX, 0xFF); /* mask to byte */
            emit_mov_r32_r32(e, RDX, RCX); /* save shift in EDX */
            emit_pop(e, RCX); /* restore operand */
            /* Now: ECX=value to shift, DL=shift amount. Need CL=amount. */
            emit_push(e, RCX);
            emit_mov_r32_r32(e, RCX, RDX); /* CL = shift amount */
            emit_pop(e, RDX); /* EDX = value */
            switch (shift_type) {
                case 0: emit8(e, 0xD3); emit8(e, modrm(3, 4, RDX)); break; /* SHL EDX, CL */
                case 1: emit8(e, 0xD3); emit8(e, modrm(3, 5, RDX)); break; /* SHR EDX, CL */
                case 2: emit8(e, 0xD3); emit8(e, modrm(3, 7, RDX)); break; /* SAR EDX, CL */
                case 3: emit8(e, 0xD3); emit8(e, modrm(3, 1, RDX)); break; /* ROR EDX, CL */
            }
            emit_mov_r32_r32(e, RCX, RDX); /* result back in ECX */
        } else if (shift_imm > 0) {
            switch (shift_type) {
                case 0: emit_shl_r32_imm(e, RCX, shift_imm); break;
                case 1: emit_shr_r32_imm(e, RCX, shift_imm ? shift_imm : 32); break;
                case 2: emit_sar_r32_imm(e, RCX, shift_imm ? shift_imm : 32); break;
                case 3: /* ROR #imm */
                    /* x86 ROR: C1 /1 ib */
                    emit8(e, 0xC1); emit8(e, modrm(3, 1, RCX)); emit8(e, shift_imm);
                    break;
            }
        } else if (shift_type == 1) {
            /* LSR #32 → result = 0 */
            emit_mov_r32_imm32(e, RCX, 0);
        } else if (shift_type == 2) {
            /* ASR #32 → result = sign extension */
            emit_sar_r32_imm(e, RCX, 31);
        }
    }

    /* Execute operation: EAX = Rn, ECX = operand2, result → EDX */
    switch (opcode) {
        case 0: /* AND */ emit_mov_r32_r32(e, RDX, RAX); emit_and_r32_r32(e, RDX, RCX); break;
        case 1: /* EOR */ emit_mov_r32_r32(e, RDX, RAX); emit_xor_r32_r32(e, RDX, RCX); break;
        case 2: /* SUB */ emit_mov_r32_r32(e, RDX, RAX); emit_sub_r32_r32(e, RDX, RCX); break;
        case 3: /* RSB */ emit_mov_r32_r32(e, RDX, RCX); emit_sub_r32_r32(e, RDX, RAX); break;
        case 4: /* ADD */ emit_mov_r32_r32(e, RDX, RAX); emit_add_r32_r32(e, RDX, RCX); break;
        case 12: /* ORR */ emit_mov_r32_r32(e, RDX, RAX); emit_or_r32_r32(e, RDX, RCX); break;
        case 13: /* MOV */ emit_mov_r32_r32(e, RDX, RCX); break;
        case 14: /* BIC */ emit_not_r32(e, RCX); emit_mov_r32_r32(e, RDX, RAX); emit_and_r32_r32(e, RDX, RCX); break;
        case 15: /* MVN */ emit_mov_r32_r32(e, RDX, RCX); emit_not_r32(e, RDX); break;
        case 8: /* TST */ emit_test_r32_r32(e, RAX, RCX); goto flags_only;
        case 9: /* TEQ */ emit_mov_r32_r32(e, RDX, RAX); emit_xor_r32_r32(e, RDX, RCX); goto flags_only;
        case 10: /* CMP */ emit_cmp_r32_r32(e, RAX, RCX); goto flags_only;
        case 11: /* CMN */ emit_mov_r32_r32(e, RDX, RAX); emit_add_r32_r32(e, RDX, RCX); goto flags_only;
        default: return 0; /* ADC, SBC, RSC — need carry, fallback */
    }

    /* Store result */
    emit_store_arm_reg(e, Rd, RDX);

    if (S) {
flags_only:
        /* Update CPSR N, Z flags from result/comparison.
         * Simple approach: compute N and Z from EDX (or flags). */
        if (opcode != 8 && opcode != 10 && opcode != 11) {
            /* N and Z from result in EDX */
            emit_load_arm_reg(e, RAX, -1); /* load CPSR */
            emit_ldr_disp32(e, RAX, REG_CPU, ARM_CPSR);
            emit_and_r32_imm32(e, RAX, 0x0FFFFFFF); /* clear N,Z,C,V */
            /* Z flag: if EDX == 0 */
            emit_test_r32_r32(e, RDX, RDX);
            /* Use CMOVZ to set Z bit */
            emit_mov_r32_imm32(e, RCX, 0x40000000); /* Z bit */
            /* jnz +2 → or eax, ecx */
            emit_jcc_rel32(e, CC_NE, 2);
            emit_or_r32_r32(e, RAX, RCX);
            /* N flag: bit 31 of EDX */
            emit_mov_r32_r32(e, RCX, RDX);
            emit_shr_r32_imm(e, RCX, 3); /* bit 31 → bit 28 (N position) */
            emit_and_r32_imm32(e, RCX, 0x80000000);
            emit_or_r32_r32(e, RAX, RCX);
            emit_str_disp32(e, REG_CPU, RAX, ARM_CPSR);
        }
    }
    return 1;
}

/* Compile a single ARM LDR/STR instruction */
static int compile_ldr_str(EmitBuf *e, uint32_t insn, uint32_t pc, VFlash *vf) {
    int I = (insn >> 25) & 1;
    int P = (insn >> 24) & 1;
    int U = (insn >> 23) & 1;
    int B = (insn >> 22) & 1;
    int W = (insn >> 21) & 1;
    int L = (insn >> 20) & 1;
    int Rn = (insn >> 16) & 0xF;
    int Rd = (insn >> 12) & 0xF;

    if (Rd == 15) return 0; /* LDR PC — fallback (branch) */
    if (!P || W) return 0; /* post-indexed or writeback — complex */
    if (I) return 0; /* register offset — complex */

    int32_t offset = insn & 0xFFF;
    if (!U) offset = -offset;

    /* Compute address: addr = Rn + offset (PC+8 if Rn==15) */
    if (Rn == 15)
        emit_mov_r32_imm32(e, RDI, (pc + 8) + offset);
    else {
        emit_load_arm_reg(e, RDI, Rn);
        if (offset != 0)
            emit_add_r32_imm32(e, RDI, offset);
    }

    /* Inline RAM fast path:
     * if (addr >= 0x10000000 && addr < 0x11000000)
     *   direct load/store from vf->ram
     * else
     *   call mem_read32/mem_write32 */
    emit_mov_r32_r32(e, RAX, RDI);
    emit_sub_r32_imm32(e, RAX, 0x10000000);
    emit_cmp_r32_imm32(e, RAX, 0x01000000);
    /* jae → slow path (call C function) */
    uint32_t jae_pos = e->pos;
    emit_jcc_rel32(e, CC_AE, 0); /* placeholder — patch later */

    /* Fast path: direct RAM access */
    /* Load ram pointer: vf->ram is at a known offset from cpu */
    /* We need the VFlash pointer. Store it at block entry. */
    emit_mov_r64_imm64(e, RSI, (uint64_t)(uintptr_t)vflash_get_ram(vf));
    /* RAX = offset in RAM, RSI = ram base */
    /* Use RAX as index (zero-extended from 32-bit) */
    emit8(e, 0x48); /* REX.W */ emit8(e, 0x63); emit8(e, modrm(3, RAX, RAX)); /* movsxd rax, eax — zero ext is fine since < 16MB */

    if (L) {
        if (B) {
            /* LDRB: result = *(uint8_t*)(ram + offset) */
            emit8(e, 0x0F); emit8(e, 0xB6); emit8(e, 0x14); emit8(e, 0x06); /* movzx edx, byte [rsi+rax] */
        } else {
            /* LDR: result = *(uint32_t*)(ram + offset) */
            emit8(e, 0x8B); emit8(e, 0x14); emit8(e, 0x06); /* mov edx, [rsi+rax] */
        }
        emit_store_arm_reg(e, Rd, RDX);
    } else {
        emit_load_arm_reg(e, RDX, Rd);
        if (B) {
            /* STRB */
            emit8(e, 0x88); emit8(e, 0x14); emit8(e, 0x06); /* mov [rsi+rax], dl */
        } else {
            /* STR */
            emit8(e, 0x89); emit8(e, 0x14); emit8(e, 0x06); /* mov [rsi+rax], edx */
        }
    }
    /* Jump past slow path */
    uint32_t jmp_pos = e->pos;
    emit_jmp_rel32(e, 0); /* placeholder */

    /* Patch JAE to here (slow path) */
    *(int32_t*)(e->buf + jae_pos + 2) = (int32_t)(e->pos - jae_pos - 6);

    /* Slow path: call C mem_read32/mem_write32 */
    /* RDI already has the address */
    emit_mov_r64_imm64(e, RSI, (uint64_t)(uintptr_t)vf); /* RSI = VFlash ptr */
    emit_mov_r32_r32(e, RDI, RDI); /* zero-extend addr */
    /* Swap: mem_read32(void *ctx, uint32_t addr) → RDI=ctx, RSI=addr */
    emit_mov_r64_r64(e, RCX, RSI); /* save vf */
    emit_mov_r64_r64(e, RSI, RDI); /* RSI = addr */
    emit_mov_r64_r64(e, RDI, RCX); /* RDI = vf */
    /* Actually: mem_read32(ctx, addr) → swap RDI and RSI */
    /* RDI = vf, RSI = addr → that's wrong, sig is (void*, uint32_t) → (rdi, esi) */
    /* Reset: */
    emit_mov_r64_imm64(e, RDI, (uint64_t)(uintptr_t)vf);
    emit_load_arm_reg(e, RSI, Rn);
    if (offset != 0) emit_add_r32_imm32(e, RSI, offset);

    if (L) {
        emit_call_abs(e, jit_mem_read32);
        emit_store_arm_reg(e, Rd, RAX);
    } else {
        emit_load_arm_reg(e, RDX, Rd); /* 3rd arg = value */
        emit_call_abs(e, jit_mem_write32);
    }

    /* Patch JMP to here */
    *(int32_t*)(e->buf + jmp_pos + 1) = (int32_t)(e->pos - jmp_pos - 5);

    return 1;
}

/* ---- Block Compiler ---- */

JitBlock *jit_compile_block(JitContext *jit, uint32_t arm_pc) {
    VFlash *vf = jit->vf;
    uint8_t *ram = vflash_get_ram(vf);

    /* Check if PC is in RAM range */
    if (arm_pc < 0x10000000 || arm_pc >= 0x11000000)
        return NULL; /* only compile RAM code */

    uint32_t ram_off = arm_pc - 0x10000000;
    if (ram_off + JIT_MAX_BLOCK_INSNS * 4 > 16 * 1024 * 1024)
        return NULL;

    /* Check cache space */
    if (jit->cache_used + 4096 > jit->cache_size) {
        jit_flush(jit); /* cache full — flush and restart */
    }

    EmitBuf e;
    e.buf = jit->code_cache + jit->cache_used;
    e.pos = 0;
    e.cap = 4096; /* max 4KB per block */

    /* Block prologue: push callee-saved, set up REG_CPU */
    emit_push(&e, RBX);
    emit_push(&e, RBP);
    emit_push(&e, R12);
    emit_mov_r64_r64(&e, RBX, RDI); /* RDI = first arg = ARM9 *cpu */

    /* Compile ARM instructions */
    int insn_count = 0;
    uint32_t cur_pc = arm_pc;
    int compiled_all = 1;

    for (int i = 0; i < JIT_MAX_BLOCK_INSNS; i++) {
        uint32_t arm_insn = *(uint32_t*)(ram + (cur_pc - 0x10000000));
        int cond = (arm_insn >> 28) & 0xF;

        /* Check if this modifies PC (end of block) */
        if (arm_modifies_pc(arm_insn)) {
            /* For B/BL we can still compile the branch */
            if ((arm_insn & 0x0E000000) == 0x0A000000 && cond == 0xE) {
                /* Unconditional B/BL — set next PC and end block */
                int L = (arm_insn >> 24) & 1;
                int32_t imm24 = arm_insn & 0x00FFFFFF;
                if (imm24 & 0x800000) imm24 -= 0x1000000;
                uint32_t target = cur_pc + 8 + imm24 * 4;
                if (L) {
                    /* BL: store return address in LR */
                    emit_mov_r32_imm32(&e, RAX, cur_pc + 4);
                    emit_store_arm_reg(&e, 14, RAX);
                }
                insn_count++;
                cur_pc = target; /* block exits to branch target */
            }
            break;
        }

        /* Emit conditional execution wrapper.
         * Load CPSR, test condition, skip instruction if not met. */
        uint32_t skip_pos = 0;
        int has_cond = (cond != 0xE && cond != 0xF);
        if (has_cond) {
            /* Load CPSR flags into EAX */
            emit_ldr_disp32(&e, RAX, REG_CPU, ARM_CPSR);
            /* Map ARM condition to a test:
             * N=bit31, Z=bit30, C=bit29, V=bit28 */
            uint8_t x86_cc = CC_E; /* default: skip if not met */
            switch (cond) {
                case 0x0: /* EQ: Z==1 */
                    emit_test_r32_r32(&e, RAX, RAX);
                    emit_and_r32_imm32(&e, RAX, 0x40000000);
                    x86_cc = CC_E; /* skip if Z bit clear */
                    break;
                case 0x1: /* NE: Z==0 */
                    emit_and_r32_imm32(&e, RAX, 0x40000000);
                    x86_cc = CC_NE; /* skip if Z bit set */
                    break;
                case 0x2: /* CS/HS: C==1 */
                    emit_and_r32_imm32(&e, RAX, 0x20000000);
                    x86_cc = CC_E;
                    break;
                case 0x3: /* CC/LO: C==0 */
                    emit_and_r32_imm32(&e, RAX, 0x20000000);
                    x86_cc = CC_NE;
                    break;
                case 0x4: /* MI: N==1 */
                    emit_and_r32_imm32(&e, RAX, 0x80000000);
                    x86_cc = CC_E;
                    break;
                case 0x5: /* PL: N==0 */
                    emit_and_r32_imm32(&e, RAX, 0x80000000);
                    x86_cc = CC_NE;
                    break;
                case 0xA: /* GE: N==V */
                    emit_mov_r32_r32(&e, RCX, RAX);
                    emit_shr_r32_imm(&e, RAX, 3); /* N(bit31)→bit28 */
                    emit_xor_r32_r32(&e, RAX, RCX); /* N^V */
                    emit_and_r32_imm32(&e, RAX, 0x10000000);
                    x86_cc = CC_NE; /* skip if N!=V */
                    break;
                case 0xB: /* LT: N!=V */
                    emit_mov_r32_r32(&e, RCX, RAX);
                    emit_shr_r32_imm(&e, RAX, 3);
                    emit_xor_r32_r32(&e, RAX, RCX);
                    emit_and_r32_imm32(&e, RAX, 0x10000000);
                    x86_cc = CC_E; /* skip if N==V */
                    break;
                case 0xC: /* GT: Z==0 && N==V */
                case 0xD: /* LE: Z==1 || N!=V */
                default:
                    /* Complex condition — end block, fall to interpreter */
                    break;
            }
            if (cond >= 0xC && cond != 0xE) {
                compiled_all = 0;
                break; /* too complex */
            }
            /* Emit conditional jump to skip this instruction */
            skip_pos = e.pos;
            emit_jcc_rel32(&e, x86_cc, 0); /* placeholder — patch after */
        }

        /* Try to compile this instruction */
        int ok = 0;
        uint32_t op = (arm_insn >> 25) & 7;

        if (op == 0 || op == 1) {
            /* Check for MUL/MLA (op=0, bits[7:4]=1001) */
            if (op == 0 && (arm_insn & 0x0FC000F0) == 0x00000090) {
                /* MUL: Rd = Rm * Rs */
                int Rd = (arm_insn >> 16) & 0xF;
                int Rm = arm_insn & 0xF;
                int Rs = (arm_insn >> 8) & 0xF;
                if (Rd < 15 && Rm < 15 && Rs < 15) {
                    emit_load_arm_reg(&e, RAX, Rm);
                    emit_load_arm_reg(&e, RCX, Rs);
                    emit_imul_r32_r32(&e, RAX, RCX);
                    if (arm_insn & (1<<21)) { /* MLA: += Rn */
                        int Rn = (arm_insn >> 12) & 0xF;
                        emit_load_arm_reg(&e, RCX, Rn);
                        emit_add_r32_r32(&e, RAX, RCX);
                    }
                    emit_store_arm_reg(&e, Rd, RAX);
                    ok = 1;
                }
            } else {
                ok = compile_data_processing(&e, arm_insn, cur_pc);
            }
        } else if (op == 2 || op == 3) {
            if (!((arm_insn >> 25) & 1))
                ok = compile_ldr_str(&e, arm_insn, cur_pc, vf);
        } else if (op == 4) {
            /* LDM/STM — compile PUSH/POP for common cases */
            int P = (arm_insn >> 24) & 1;
            int U = (arm_insn >> 23) & 1;
            int S = (arm_insn >> 22) & 1;
            int W = (arm_insn >> 21) & 1;
            int L = (arm_insn >> 20) & 1;
            int Rn = (arm_insn >> 16) & 0xF;
            uint16_t rlist = arm_insn & 0xFFFF;
            if (S || Rn == 15 || (rlist & (1<<15))) {
                ok = 0; /* complex: S bit, PC in list — fallback */
            } else {
                int reg_count = __builtin_popcount(rlist);
                /* Load base address into EDI */
                emit_load_arm_reg(&e, RDI, Rn);
                /* For STMDB (PUSH): pre-decrement */
                if (!U && P) emit_sub_r32_imm32(&e, RDI, reg_count * 4);
                /* For LDMIA (POP): address starts at Rn */
                int offset = 0;
                if (U && !P) offset = 0; /* IA: start at Rn */
                else if (U && P) offset = 4; /* IB: start at Rn+4 */
                else if (!U && !P) offset = -(reg_count-1)*4; /* DA */
                /* Process each register */
                emit_mov_r64_imm64(&e, RSI, (uint64_t)(uintptr_t)vflash_get_ram(vf));
                int cur_off = offset;
                for (int ri = 0; ri < 15; ri++) {
                    if (!(rlist & (1 << ri))) continue;
                    /* addr = RDI + cur_off, inline RAM fast path */
                    emit_mov_r32_r32(&e, RAX, RDI);
                    if (cur_off) emit_add_r32_imm32(&e, RAX, cur_off);
                    emit_sub_r32_imm32(&e, RAX, 0x10000000);
                    /* Bounds check — skip if outside RAM */
                    emit_cmp_r32_imm32(&e, RAX, 0x01000000);
                    uint32_t jae_pos = e.pos;
                    emit_jcc_rel32(&e, CC_AE, 0);
                    /* Zero-extend RAX for 64-bit addressing */
                    emit8(&e, 0x48); emit8(&e, 0x63); emit8(&e, modrm(3, RAX, RAX));
                    if (L) {
                        /* LDM: load from memory to register */
                        emit8(&e, 0x8B); emit8(&e, 0x14); emit8(&e, 0x06);
                        emit_store_arm_reg(&e, ri, RDX);
                    } else {
                        /* STM: store register to memory */
                        emit_load_arm_reg(&e, RDX, ri);
                        emit8(&e, 0x89); emit8(&e, 0x14); emit8(&e, 0x06);
                    }
                    /* Patch JAE */
                    *(int32_t*)(e.buf + jae_pos + 2) = (int32_t)(e.pos - jae_pos - 6);
                    cur_off += 4;
                }
                /* Writeback */
                if (W) {
                    if (U) emit_add_r32_imm32(&e, RDI, reg_count * 4);
                    else   emit_sub_r32_imm32(&e, RDI, reg_count * 4);
                    emit_store_arm_reg(&e, Rn, RDI);
                }
                ok = 1;
            }
        }

        if (!ok) {
            if (has_cond) {
                /* Undo the conditional jump we emitted */
                e.pos = skip_pos;
            }
            compiled_all = 0;
            break;
        }

        /* Patch the conditional skip jump to here */
        if (has_cond && skip_pos) {
            *(int32_t*)(e.buf + skip_pos + 2) = (int32_t)(e.pos - skip_pos - 6);
        }

        insn_count++;
        cur_pc += 4;
    }

    if (insn_count == 0)
        return NULL; /* couldn't compile anything */

    /* Block epilogue: store next PC, restore, return insn count */
    emit_mov_r32_imm32(&e, RAX, cur_pc);
    emit_store_arm_reg(&e, 15, RAX); /* cpu->r[15] = next_pc */
    emit_mov_r32_imm32(&e, RAX, insn_count); /* return insn count */
    emit_pop(&e, R12);
    emit_pop(&e, RBP);
    emit_pop(&e, RBX);
    emit_ret(&e);

    /* Mark code page */
    uint32_t page = ram_off >> 12;
    if (page < 4096) jit->code_pages[page] = 1;

    /* Insert into cache */
    uint8_t *code_ptr = jit->code_cache + jit->cache_used;
    jit->cache_used += e.pos;
    jit->blocks_compiled++;

    JitBlock *block = jit_insert(jit, arm_pc, code_ptr, e.pos, insn_count);
    if (block) {
        block->next_pc[0] = cur_pc;
        block->next_pc[1] = cur_pc;
    }
    return block;
}

/* ---- JIT Dispatcher ---- */

/* Native block signature: int block(ARM9 *cpu) → returns insn count */
typedef int (*JitBlockFn)(ARM9 *cpu);

int jit_run(JitContext *jit, int cycles) {
    VFlash *vf = jit->vf;
    ARM9 *cpu = (ARM9 *)vflash_get_cpu(vf);
    int executed = 0;

    while (executed < cycles) {
        uint32_t pc = cpu->r[15];

        /* Check HLE intercepts first */
        if (pc >= 0x109A0000 && pc < 0x10D00000) {
            /* Potential HLE target — fall back to interpreter */
            arm9_step(cpu);
            executed++;
            continue;
        }

        /* Lookup compiled block */
        JitBlock *block = jit_lookup(jit, pc);

        if (!block) {
            /* Try to compile */
            block = jit_compile_block(jit, pc);
        }

        if (block) {
            /* Execute native code */
            JitBlockFn fn = (JitBlockFn)block->native_code;
            int count = fn(cpu);
            executed += count;
            jit->blocks_executed++;
            jit->insns_executed += count;
        } else {
            /* Fallback: interpreter for one instruction */
            arm9_step(cpu);
            executed++;
        }
    }
    /* Periodic stats */
    static uint64_t last_log = 0;
    if (jit->blocks_executed - last_log > 10000) {
        printf("[JIT] blocks=%lu exec=%lu insns=%lu cache=%uKB/%uKB\n",
               jit->blocks_compiled, jit->blocks_executed,
               jit->insns_executed, jit->cache_used/1024, jit->cache_size/1024);
        last_log = jit->blocks_executed;
    }
    return executed;
}
