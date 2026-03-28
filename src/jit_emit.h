/* x86_64 machine code emitter — minimal set for ARM JIT.
 * Emits raw bytes into a code buffer. */
#ifndef JIT_EMIT_H
#define JIT_EMIT_H

#include <stdint.h>
#include <string.h>

/* Code buffer for emission */
typedef struct {
    uint8_t *buf;
    uint32_t pos;
    uint32_t cap;
} EmitBuf;

static inline void emit8(EmitBuf *e, uint8_t b) {
    if (e->pos < e->cap) e->buf[e->pos++] = b;
}
static inline void emit16(EmitBuf *e, uint16_t w) {
    emit8(e, w & 0xFF); emit8(e, w >> 8);
}
static inline void emit32(EmitBuf *e, uint32_t d) {
    emit8(e, d & 0xFF); emit8(e, (d>>8)&0xFF);
    emit8(e, (d>>16)&0xFF); emit8(e, (d>>24)&0xFF);
}
static inline void emit64(EmitBuf *e, uint64_t q) {
    emit32(e, (uint32_t)q); emit32(e, (uint32_t)(q >> 32));
}

/* x86_64 register encodings */
enum {
    RAX=0, RCX=1, RDX=2, RBX=3, RSP=4, RBP=5, RSI=6, RDI=7,
    R8=8, R9=9, R10=10, R11=11, R12=12, R13=13, R14=14, R15=15
};

/* REX prefix */
static inline void emit_rex(EmitBuf *e, int w, int r, int x, int b) {
    uint8_t rex = 0x40 | (w?8:0) | (r?4:0) | (x?2:0) | (b?1:0);
    if (rex != 0x40) emit8(e, rex);
}
static inline void emit_rexw(EmitBuf *e, int r_reg, int b_reg) {
    emit8(e, 0x48 | ((r_reg>>3)&1)<<2 | ((b_reg>>3)&1));
}

/* ModRM byte */
static inline uint8_t modrm(int mod, int reg, int rm) {
    return (uint8_t)((mod<<6) | ((reg&7)<<3) | (rm&7));
}

/* --- MOV --- */
/* mov reg64, imm64 */
static inline void emit_mov_r64_imm64(EmitBuf *e, int reg, uint64_t imm) {
    emit8(e, 0x48 | ((reg>>3)&1)); /* REX.W + REX.B */
    emit8(e, 0xB8 + (reg & 7));
    emit64(e, imm);
}

/* mov reg32, imm32 */
static inline void emit_mov_r32_imm32(EmitBuf *e, int reg, uint32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xB8 + (reg & 7));
    emit32(e, imm);
}

/* mov reg64, reg64 */
static inline void emit_mov_r64_r64(EmitBuf *e, int dst, int src) {
    emit_rexw(e, src, dst);
    emit8(e, 0x89);
    emit8(e, modrm(3, src, dst));
}

/* mov reg32, reg32 */
static inline void emit_mov_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x89);
    emit8(e, modrm(3, src, dst));
}

/* mov [reg64 + disp32], reg32 — store to memory */
static inline void emit_str_disp32(EmitBuf *e, int base, int src, int32_t disp) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((base>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x89);
    emit8(e, modrm(2, src, base & 7));
    if ((base & 7) == 4) emit8(e, 0x24); /* SIB for RSP */
    emit32(e, (uint32_t)disp);
}

/* mov reg32, [reg64 + disp32] — load from memory */
static inline void emit_ldr_disp32(EmitBuf *e, int dst, int base, int32_t disp) {
    int rex = 0x40 | ((dst>>3)&1)<<2 | ((base>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x8B);
    emit8(e, modrm(2, dst, base & 7));
    if ((base & 7) == 4) emit8(e, 0x24); /* SIB for RSP */
    emit32(e, (uint32_t)disp);
}

/* --- Arithmetic --- */
/* add reg32, imm32 */
static inline void emit_add_r32_imm32(EmitBuf *e, int reg, int32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x81);
    emit8(e, modrm(3, 0, reg));
    emit32(e, (uint32_t)imm);
}

/* sub reg32, imm32 */
static inline void emit_sub_r32_imm32(EmitBuf *e, int reg, int32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x81);
    emit8(e, modrm(3, 5, reg));
    emit32(e, (uint32_t)imm);
}

/* add reg32, reg32 */
static inline void emit_add_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x01);
    emit8(e, modrm(3, src, dst));
}

/* sub reg32, reg32 */
static inline void emit_sub_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x29);
    emit8(e, modrm(3, src, dst));
}

/* cmp reg32, imm32 */
static inline void emit_cmp_r32_imm32(EmitBuf *e, int reg, int32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x81);
    emit8(e, modrm(3, 7, reg));
    emit32(e, (uint32_t)imm);
}

/* cmp reg32, reg32 */
static inline void emit_cmp_r32_r32(EmitBuf *e, int r1, int r2) {
    int rex = 0x40 | ((r2>>3)&1)<<2 | ((r1>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x39);
    emit8(e, modrm(3, r2, r1));
}

/* and/or/xor reg32, reg32 */
static inline void emit_and_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x21);
    emit8(e, modrm(3, src, dst));
}
static inline void emit_or_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x09);
    emit8(e, modrm(3, src, dst));
}
static inline void emit_xor_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((src>>3)&1)<<2 | ((dst>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x31);
    emit8(e, modrm(3, src, dst));
}

/* and reg32, imm32 */
static inline void emit_and_r32_imm32(EmitBuf *e, int reg, uint32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x81);
    emit8(e, modrm(3, 4, reg));
    emit32(e, imm);
}

/* or reg32, imm32 */
static inline void emit_or_r32_imm32(EmitBuf *e, int reg, uint32_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x81);
    emit8(e, modrm(3, 1, reg));
    emit32(e, imm);
}

/* shl/shr/sar reg32, imm8 */
static inline void emit_shl_r32_imm(EmitBuf *e, int reg, uint8_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 4, reg));
    emit8(e, imm);
}
static inline void emit_shr_r32_imm(EmitBuf *e, int reg, uint8_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 5, reg));
    emit8(e, imm);
}
static inline void emit_sar_r32_imm(EmitBuf *e, int reg, uint8_t imm) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xC1);
    emit8(e, modrm(3, 7, reg));
    emit8(e, imm);
}

/* not reg32 */
static inline void emit_not_r32(EmitBuf *e, int reg) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xF7);
    emit8(e, modrm(3, 2, reg));
}

/* imul reg32, reg32 */
static inline void emit_imul_r32_r32(EmitBuf *e, int dst, int src) {
    int rex = 0x40 | ((dst>>3)&1)<<2 | ((src>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x0F); emit8(e, 0xAF);
    emit8(e, modrm(3, dst, src));
}

/* --- Branches --- */
/* jcc rel32 (conditional near jump) */
static inline void emit_jcc_rel32(EmitBuf *e, uint8_t cc, int32_t rel) {
    emit8(e, 0x0F);
    emit8(e, 0x80 + cc);
    emit32(e, (uint32_t)rel);
}
/* jmp rel32 */
static inline void emit_jmp_rel32(EmitBuf *e, int32_t rel) {
    emit8(e, 0xE9);
    emit32(e, (uint32_t)rel);
}

/* x86 condition codes */
#define CC_O  0x0
#define CC_NO 0x1
#define CC_B  0x2  /* below / carry */
#define CC_AE 0x3  /* above-equal / no carry */
#define CC_E  0x4  /* equal / zero */
#define CC_NE 0x5  /* not equal / not zero */
#define CC_BE 0x6
#define CC_A  0x7
#define CC_S  0x8  /* sign / negative */
#define CC_NS 0x9
#define CC_L  0xC  /* less (signed) */
#define CC_GE 0xD  /* greater-equal (signed) */
#define CC_LE 0xE
#define CC_G  0xF  /* greater (signed) */

/* --- Stack / Call --- */
/* push reg64 */
static inline void emit_push(EmitBuf *e, int reg) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x50 + (reg & 7));
}
/* pop reg64 */
static inline void emit_pop(EmitBuf *e, int reg) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0x58 + (reg & 7));
}
/* ret */
static inline void emit_ret(EmitBuf *e) {
    emit8(e, 0xC3);
}
/* call reg64 */
static inline void emit_call_r64(EmitBuf *e, int reg) {
    if (reg >= 8) emit8(e, 0x41);
    emit8(e, 0xFF);
    emit8(e, modrm(3, 2, reg));
}
/* call [rip + disp32] — for calling C functions */
static inline void emit_call_abs(EmitBuf *e, void *func_ptr) {
    /* mov rax, imm64; call rax */
    emit_mov_r64_imm64(e, RAX, (uint64_t)(uintptr_t)func_ptr);
    emit_call_r64(e, RAX);
}

/* test reg32, reg32 */
static inline void emit_test_r32_r32(EmitBuf *e, int r1, int r2) {
    int rex = 0x40 | ((r2>>3)&1)<<2 | ((r1>>3)&1);
    if (rex != 0x40) emit8(e, rex);
    emit8(e, 0x85);
    emit8(e, modrm(3, r2, r1));
}

#endif /* JIT_EMIT_H */
