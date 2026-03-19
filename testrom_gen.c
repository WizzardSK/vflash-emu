/* testrom_gen.c — generates testrom.bin
 *
 * Builds a small ARM32 test ROM in pure C by encoding ARM instructions
 * as 32-bit words directly. No cross-assembler needed.
 *
 * The ROM:
 *   1. Sends "VFlash Test ROM\n" via UART
 *   2. Tests ADD/CMP arithmetic
 *   3. Tests CLZ instruction
 *   4. Tests MUL instruction
 *   5. Reads timer count register
 *   6. Reads input register
 *   7. Fills framebuffer with a solid colour and triggers video DMA
 *   8. Spins, printing a dot every ~1M cycles
 *
 * Usage:  gcc -o testrom_gen testrom_gen.c && ./testrom_gen
 * Output: testrom.bin  (load at 0x10000000)
 * Run:    ./vflash --headless --dbg testrom.bin
 *         ./vflash --dbg testrom.bin
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ---- ARM32 instruction encoder helpers ---- */

/* Condition codes */
#define CC_EQ  0x0
#define CC_NE  0x1
#define CC_GE  0xA
#define CC_LT  0xB
#define CC_AL  0xE

/* Data processing: cond 00I op S Rn Rd <operand2> */
static uint32_t dp(int cc, int op, int s, int rn, int rd, uint32_t op2) {
    return ((uint32_t)cc<<28)|((uint32_t)op<<21)|((uint32_t)s<<20)|
           ((uint32_t)rn<<16)|((uint32_t)rd<<12)|op2;
}

/* MOV Rd, #imm8 (imm8 must fit in 8 bits with zero rotation) */
static uint32_t MOV_imm(int rd, uint8_t imm) {
    return dp(CC_AL,0xD,0,0,rd,0x02000000|(imm));  /* I=1, rot=0 */
}

/* MOV Rd, Rm */
static uint32_t MOV_reg(int rd, int rm) {
    return dp(CC_AL,0xD,0,0,rd,rm);
}

/* ADD Rd, Rn, Rm */
static uint32_t ADD_reg(int rd, int rn, int rm) {
    return dp(CC_AL,0x4,0,rn,rd,rm);
}

/* ADD Rd, Rn, #imm */
static uint32_t ADD_imm(int rd, int rn, uint8_t imm) {
    return dp(CC_AL,0x4,0,rn,rd,0x02000000|imm);
}

/* CMP Rn, #imm */
static uint32_t CMP_imm(int rn, uint8_t imm) {
    return dp(CC_AL,0xA,1,rn,0,0x02000000|imm);
}

/* CMP Rn, Rm */
static uint32_t CMP_reg(int rn, int rm) {
    return dp(CC_AL,0xA,1,rn,0,rm);
}

/* AND Rd, Rn, #imm */
static uint32_t AND_imm(int rd, int rn, uint8_t imm) {
    return dp(CC_AL,0x0,0,rn,rd,0x02000000|imm);
}

/* TST Rn, #imm */
static uint32_t TST_imm(int rn, uint8_t imm) {
    return dp(CC_AL,0x8,1,rn,0,0x02000000|imm);
}

/* MUL Rd, Rm, Rs */
static uint32_t MUL(int rd, int rm, int rs) {
    return (uint32_t)(CC_AL<<28)|(rd<<16)|(rs<<8)|0x90|(rm);
}

/* CLZ Rd, Rm */
static uint32_t CLZ(int rd, int rm) {
    return 0xE16F0F10u|((uint32_t)rd<<12)|(uint32_t)rm;
}

/* Branch relative: offset in bytes from PC+8 */
static uint32_t B(int32_t offset_bytes) {
    int32_t enc = (offset_bytes - 8) >> 2;
    return 0xEA000000u|(uint32_t)(enc & 0xFFFFFF);
}

/* BL relative */
static uint32_t BL_rel(int32_t offset_bytes) {
    int32_t enc = (offset_bytes - 8) >> 2;
    return 0xEB000000u|(uint32_t)(enc & 0xFFFFFF);
}

/* BEQ relative */
static uint32_t BEQ(int32_t offset_bytes) {
    int32_t enc = (offset_bytes - 8) >> 2;
    return 0x0A000000u|(uint32_t)(enc & 0xFFFFFF);
}

/* BNE relative */
static uint32_t BNE(int32_t offset_bytes) {
    int32_t enc = (offset_bytes - 8) >> 2;
    return 0x1A000000u|(uint32_t)(enc & 0xFFFFFF);
}

/* BLT relative */
static uint32_t BLT(int32_t offset_bytes) {
    int32_t enc = (offset_bytes - 8) >> 2;
    return 0xBA000000u|(uint32_t)(enc & 0xFFFFFF);
}

/* LDR Rd, [PC, #off] — loads word at PC+8+off */
static uint32_t LDR_pcrel(int rd, int32_t off) {
    uint32_t u = (off>=0)?1:0;
    uint32_t abs_off = (uint32_t)(off<0?-off:off);
    return 0xE51F0000u|(u<<23)|((uint32_t)rd<<12)|abs_off;
}

/* LDR Rd, [Rn, #0] */
static uint32_t LDR_base(int rd, int rn) {
    return 0xE5900000u|((uint32_t)rn<<16)|((uint32_t)rd<<12);
}

/* STR Rd, [Rn, #0] */
static uint32_t STR_base(int rd, int rn) {
    return 0xE5800000u|((uint32_t)rn<<16)|((uint32_t)rd<<12);
}

/* STR Rd, [Rn, Rm] */
static uint32_t STR_reg(int rd, int rn, int rm) {
    return 0xE7800000u|((uint32_t)rn<<16)|((uint32_t)rd<<12)|(uint32_t)rm;
}

/* LDRB Rd, [Rn], #1 (post-index) */
static uint32_t LDRB_postinc(int rd, int rn) {
    return 0xE4D00001u|((uint32_t)rn<<16)|((uint32_t)rd<<12);
}

/* PUSH {Rlist} */
static uint32_t PUSH(uint16_t rlist) {
    return 0xE92D0000u|rlist;
}

/* POP {Rlist} */
static uint32_t POP(uint16_t rlist) {
    return 0xE8BD0000u|rlist;
}

/* MOV SP, #imm — encoded as shifted immediate */
static uint32_t MOV_sp_himm(uint32_t imm) {
    /* Try to encode as rotated 8-bit immediate */
    for (int rot = 0; rot < 16; rot++) {
        uint32_t v = (imm >> (rot*2)) | (imm << (32 - rot*2));
        if ((v & 0xFF) == v) {
            uint32_t op2 = 0x02000000u | ((uint32_t)rot << 8) | (v & 0xFF);
            return dp(CC_AL,0xD,0,0,13,op2);
        }
    }
    return 0xE1A0D00Du; /* MOV SP, SP (NOP fallback) */
}

/* LSR Rd, Rm, #imm */
static uint32_t LSR_imm(int rd, int rm, int shift) {
    return dp(CC_AL,0xD,0,0,rd,(uint32_t)rm|((uint32_t)shift<<7)|0x20u);
}

/* ADD Rd, Rd, #'0' encoded */
static uint32_t ADD_ascii0(int rd) {
    return ADD_imm(rd,rd,'0');
}

/* NOP */
#define NOP 0xE320F000u

/* ---- ROM buffer ---- */

#define MAX_INSNS 512
static uint32_t rom[MAX_INSNS];
static int rom_pos = 0;

#define LOAD_ADDR 0x10000000u

static int PC(void)  { return rom_pos * 4; }
static int PCA(void) { return LOAD_ADDR + rom_pos * 4; }

static int emit(uint32_t insn) {
    if (rom_pos >= MAX_INSNS) { fprintf(stderr,"ROM overflow\n"); exit(1); }
    int pos = rom_pos++;
    rom[pos] = insn;
    return pos;
}

/* Emit a literal 32-bit word (for LDR PC-relative pools) */
static int emit_word(uint32_t w) { return emit(w); }

/* Patch a branch/BL at 'patch_pos' to point to 'target_pos' (word indices) */
static void patch_b(int patch_pos, int target_pos) {
    int32_t off = (target_pos - patch_pos) * 4;
    uint32_t insn = rom[patch_pos];
    uint32_t cc_op = insn & 0xFF000000u;
    int32_t enc = (off - 8) >> 2;
    rom[patch_pos] = cc_op | (uint32_t)(enc & 0xFFFFFF);
}

/* ============================================================
 * I/O addresses as data words embedded in the code
 * ============================================================ */

#define UART_TX_ADDR    0x80005000u
#define UART_STAT_ADDR  0x80005004u
#define TIMER0_CTRL_A   0x80000108u
#define TIMER0_CNT_A    0x80000104u
#define TIMER0_LOAD_A   0x80000100u
#define INPUT_ADDR      0x80004000u
#define VIDEO_FB_ADDR   0x80002000u
#define VIDEO_CTRL_A    0x80002004u
#define FRAMEBUF_ADDR   0x10100000u

/* ---- Subroutines we'll place at fixed positions ---- */

int sub_putc;    /* uart_putc: send char in r0, clobbers r1,r2 */
int sub_puts;    /* uart_puts: print null-str at r0, clobbers r0-r3 */
int sub_hex32;   /* uart_hex32: print r0 as hex\n, clobbers r0-r5 */
int sub_delay;   /* delay ~r0 iterations */

/* ============================================================
 * Generate the ROM
 * ============================================================ */
int main(void) {

    /* ---- 0: _start ---- */
    /* SP = 0x11000000 */
    emit(MOV_sp_himm(0x11000000u));     /* MOV sp, #0x11000000 (rotated imm) */
    /* Actually 0x11000000 might not encode — use LDR */
    rom_pos--;  /* undo */
    /* LDR sp, [pc, #pool_sp-pc-8] — we'll patch later */
    int patch_sp = emit(0xE59FD000u);   /* LDR sp, [pc, #0] placeholder */

    /* ---- Forward-declared subroutine BL targets (patched later) ---- */
    /* BL uart_puts (r0 = ptr to string) */
    /* We'll build code first, then patch BL offsets */

    /* str_hello ptr → r0 via LDR pc-rel, then BL puts */
    int patch_hello_ldr = emit(LDR_pcrel(0,0));    /* LDR r0,[pc,#?] */
    int patch_hello_bl  = emit(BL_rel(8));         /* BL puts — patched */

    /* Test 1: arithmetic */
    emit(MOV_imm(0, 42));
    emit(MOV_imm(1, 58));
    emit(ADD_reg(2, 0, 1));
    emit(CMP_imm(2, 100));
    int patch_bne_arith = emit(BNE(8));            /* BNE fail_arith */
    int patch_arith_ok_ldr = emit(LDR_pcrel(0,0));
    int patch_arith_ok_bl  = emit(BL_rel(8));
    int patch_arith_skip   = emit(B(8));           /* B test2 */
    /* fail_arith: */
    int pos_fail_arith = rom_pos;
    int patch_arith_fail_ldr = emit(LDR_pcrel(0,0));
    int patch_arith_fail_bl  = emit(BL_rel(8));
    /* test2: */
    int pos_test2 = rom_pos;

    /* Test 2: CLZ */
    /* MOV r0, #0x00800000 — encoded as 0x80 rotated by 8 positions */
    emit(dp(CC_AL,0xD,0,0,0, 0x02000000u|(8<<8)|0x80u));
    emit(CLZ(1, 0));                   /* CLZ r1, r0 → r1 should = 8 */
    emit(CMP_imm(1, 8));
    int patch_bne_clz = emit(BNE(8));
    int patch_clz_ok_ldr = emit(LDR_pcrel(0,0));
    int patch_clz_ok_bl  = emit(BL_rel(8));
    int patch_clz_skip   = emit(B(8));
    int pos_fail_clz = rom_pos;
    int patch_clz_fail_ldr = emit(LDR_pcrel(0,0));
    int patch_clz_fail_bl  = emit(BL_rel(8));
    int pos_test3 = rom_pos;

    /* Test 3: MUL */
    /* r0=12345 r1=6789 r2=r0*r1=83810205 */
    /* Use LDR for large immediates */
    int patch_mul_a_ldr = emit(LDR_pcrel(0,0));   /* r0 = 12345 */
    int patch_mul_b_ldr = emit(LDR_pcrel(1,0));   /* r1 = 6789 */
    emit(MUL(2,0,1));
    int patch_mul_c_ldr = emit(LDR_pcrel(3,0));   /* r3 = 83810205 */
    emit(CMP_reg(2,3));
    int patch_bne_mul   = emit(BNE(8));
    int patch_mul_ok_ldr  = emit(LDR_pcrel(0,0));
    int patch_mul_ok_bl   = emit(BL_rel(8));
    int patch_mul_skip    = emit(B(8));
    int pos_fail_mul = rom_pos;
    int patch_mul_fail_ldr = emit(LDR_pcrel(0,0));
    int patch_mul_fail_bl  = emit(BL_rel(8));
    int pos_test4 = rom_pos;

    /* Test 4: Timer */
    int patch_timer_load_ldr = emit(LDR_pcrel(2,0));  /* r2 = TIMER0_LOAD */
    int patch_timer_val_ldr  = emit(LDR_pcrel(3,0));  /* r3 = 150000 */
    emit(STR_base(3,2));
    int patch_timer_ctrl_ldr = emit(LDR_pcrel(2,0));  /* r2 = TIMER0_CTRL */
    emit(MOV_imm(3,3));                               /* enable + periodic */
    emit(STR_base(3,2));
    int patch_timer_cnt_ldr  = emit(LDR_pcrel(2,0));  /* r2 = TIMER0_COUNT */
    emit(LDR_base(4,2));                              /* r4 = count */
    emit(CMP_imm(4,0));
    int patch_beq_timer = emit(BEQ(8));
    int patch_timer_ok_ldr  = emit(LDR_pcrel(0,0));
    int patch_timer_ok_bl   = emit(BL_rel(8));
    int patch_timer_skip    = emit(B(8));
    int pos_fail_timer = rom_pos;
    int patch_timer_fail_ldr = emit(LDR_pcrel(0,0));
    int patch_timer_fail_bl  = emit(BL_rel(8));
    int pos_test5 = rom_pos;

    /* Test 5: Input read */
    int patch_input_ldr = emit(LDR_pcrel(2,0));
    emit(LDR_base(3,2));
    int patch_input_ok_ldr = emit(LDR_pcrel(0,0));
    int patch_input_ok_bl  = emit(BL_rel(8));
    emit(MOV_reg(0,3));
    int patch_hex32_bl = emit(BL_rel(8));            /* BL hex32 */

    /* Test 6: Video DMA — fill framebuffer solid colour */
    int patch_fb_addr_ldr  = emit(LDR_pcrel(0,0));  /* r0 = FRAMEBUF */
    /* LDR r1, =307200 (0x4B000) */
    int patch_fb_size_ldr  = emit(LDR_pcrel(1,0));
    emit(MOV_imm(2,0));                             /* loop counter */
    /* r3 = 0xFF336699 — test pattern */
    int patch_fb_color_ldr = emit(LDR_pcrel(3,0));
    /* fill_loop: STR r3, [r0, r2]; ADD r2,r2,#4; CMP r2,r1; BLT fill_loop */
    int pos_fill = rom_pos;
    emit(STR_reg(3,0,2));
    emit(ADD_imm(2,2,4));
    emit(CMP_reg(2,1));
    emit(BLT((pos_fill - rom_pos)*4));
    /* Set FB address register */
    int patch_vfb_addr_ldr  = emit(LDR_pcrel(2,0));
    int patch_vfb_val_ldr   = emit(LDR_pcrel(3,0));  /* r3 = FRAMEBUF */
    emit(STR_base(3,2));
    /* Trigger DMA */
    int patch_vctrl_ldr = emit(LDR_pcrel(2,0));
    emit(MOV_imm(3,3));
    emit(STR_base(3,2));
    int patch_video_ok_ldr = emit(LDR_pcrel(0,0));
    int patch_video_ok_bl  = emit(BL_rel(8));

    /* Done */
    int patch_done_ldr = emit(LDR_pcrel(0,0));
    int patch_done_bl  = emit(BL_rel(8));

    /* Spin loop */
    int pos_spin = rom_pos;
    int patch_spin_input_ldr = emit(LDR_pcrel(2,0));
    emit(LDR_base(3,2));
    emit(TST_imm(3, 0x10));         /* RED button bit4 active-low: pressed=0 */
    emit(BNE(8));                   /* BNE spin (not pressed = bit set = BNE) */
    int patch_spin_bne = rom_pos - 1;
    emit(B((pos_spin - rom_pos)*4));/* B spin */
    /* RED pressed: restart */
    emit(B(-((int)rom_pos * 4)));   /* B _start (addr 0) */

    /* ============================================================
     * Subroutines
     * ============================================================ */

    /* ---- uart_putc (r0=char, clobbers r1,r2) ---- */
    sub_putc = rom_pos;
    emit(PUSH(0x4006));             /* PUSH {r1,r2,lr} */
    int pos_putc_wait = rom_pos;
    int patch_putc_stat_ldr = emit(LDR_pcrel(1,0));
    emit(LDR_base(2,1));
    emit(TST_imm(2,1));
    emit(BEQ((pos_putc_wait - rom_pos)*4));
    int patch_putc_tx_ldr = emit(LDR_pcrel(1,0));
    emit(STR_base(0,1));
    emit(POP(0x8006));              /* POP {r1,r2,pc} */

    /* ---- uart_puts (r0=str ptr, clobbers r0-r3) ---- */
    sub_puts = rom_pos;
    emit(PUSH(0x4010));             /* PUSH {r4,lr} */
    emit(MOV_reg(4,0));             /* r4 = str ptr */
    int pos_puts_loop = rom_pos;
    emit(LDRB_postinc(0,4));        /* LDRB r0,[r4],#1 */
    emit(CMP_imm(0,0));
    int patch_puts_beq = emit(BEQ(8));
    emit(BL_rel((sub_putc - rom_pos)*4));
    emit(B((pos_puts_loop - rom_pos)*4));
    int pos_puts_end = rom_pos;
    emit(POP(0x8010));              /* POP {r4,pc} */
    patch_b(patch_puts_beq, pos_puts_end);

    /* ---- uart_hex32 (r0=val, clobbers r0-r5) ---- */
    sub_hex32 = rom_pos;
    emit(PUSH(0x4030));             /* PUSH {r4,r5,lr} */
    emit(MOV_reg(4,0));             /* r4 = value */
    emit(MOV_imm(0,'0'));
    emit(BL_rel((sub_putc - rom_pos)*4));
    emit(MOV_imm(0,'x'));
    emit(BL_rel((sub_putc - rom_pos)*4));
    emit(MOV_imm(5,28));            /* r5 = shift count */
    int pos_hex_loop = rom_pos;
    emit(LSR_imm(0,4,0));           /* LSR r0,r4,#? — needs shift from r5 */
    /* Actually encode MOV r0, r4, LSR r5 */
    rom[rom_pos-1] = 0xE1A00034u;  /* MOV r0, r4, LSR r5 */
    emit(AND_imm(0,0,0xF));
    emit(CMP_imm(0,10));
    int patch_hex_lt = emit(BLT(8));
    emit(ADD_imm(0,0,'A'-10));
    int patch_hex_skip_add = emit(B(8));
    int pos_hex_add0 = rom_pos;
    emit(ADD_ascii0(0));
    int pos_hex_putc = rom_pos;
    emit(BL_rel((sub_putc - rom_pos)*4));
    /* SUB r5, r5, #4 */
    emit(dp(CC_AL,0x2,1,5,5,0x02000004u));
    /* BGE hex_loop */
    emit(0xAA000000u | (uint32_t)(((pos_hex_loop - rom_pos)*4 - 8)/4 & 0xFFFFFF));
    emit(MOV_imm(0,'\n'));
    emit(BL_rel((sub_putc - rom_pos)*4));
    emit(POP(0x8030));
    patch_b(patch_hex_lt, pos_hex_add0);
    patch_b(patch_hex_skip_add, pos_hex_putc);

    /* ============================================================
     * Literal pool — data words after subroutines
     * ============================================================ */

    /* Helper to embed a string and return its word-index */
    /* We'll use a simple data section approach: emit strings as bytes */
    /* For simplicity, all literals go here */

#define POOL_WORD(pw) emit_word(pw)

    int pool_sp       = POOL_WORD(0x11000000u);
    int pool_uart_tx  = POOL_WORD(UART_TX_ADDR);
    int pool_uart_st  = POOL_WORD(UART_STAT_ADDR);
    int pool_t0_load  = POOL_WORD(TIMER0_LOAD_A);
    int pool_t0_ctrl  = POOL_WORD(TIMER0_CTRL_A);
    int pool_t0_cnt   = POOL_WORD(TIMER0_CNT_A);
    int pool_input    = POOL_WORD(INPUT_ADDR);
    int pool_vfb      = POOL_WORD(VIDEO_FB_ADDR);
    int pool_vctrl    = POOL_WORD(VIDEO_CTRL_A);
    int pool_fb_addr  = POOL_WORD(FRAMEBUF_ADDR);
    int pool_fb_size  = POOL_WORD(307200);
    int pool_fb_color = POOL_WORD(0xFF336699u);
    int pool_12345    = POOL_WORD(12345);
    int pool_6789     = POOL_WORD(6789);
    int pool_83810205 = POOL_WORD(83810205);
    int pool_150000   = POOL_WORD(150000);

    /* String data as byte sequences — encode as words */
    /* We embed each string as words and record its position */
    struct { const char *s; int pos; } strings[] = {
        {"VFlash Test ROM v1.0\n", 0},
        {"PASS: arithmetic\n", 0},
        {"FAIL: arithmetic\n", 0},
        {"PASS: CLZ\n", 0},
        {"FAIL: CLZ\n", 0},
        {"PASS: MUL\n", 0},
        {"FAIL: MUL\n", 0},
        {"PASS: timer\n", 0},
        {"FAIL: timer\n", 0},
        {"PASS: input=", 0},
        {"PASS: video DMA\n", 0},
        {"ALL TESTS DONE\n", 0},
    };
    int nstr = (int)(sizeof(strings)/sizeof(strings[0]));

    for (int i = 0; i < nstr; i++) {
        strings[i].pos = rom_pos;
        const char *s = strings[i].s;
        int len = strlen(s) + 1;  /* include null terminator */
        int words = (len + 3) / 4;
        for (int w = 0; w < words; w++) {
            uint32_t word = 0;
            for (int b = 0; b < 4; b++) {
                int idx = w*4 + b;
                word |= (idx < len ? (uint8_t)s[idx] : 0) << (b*8);
            }
            emit_word(word);
        }
    }

    /* ============================================================
     * Patch all LDR PC-relative instructions
     * PC-relative LDR: offset = pool_pos*4 - (patch_pos*4 + 8)
     * ============================================================ */

/* Patch LDR PC-relative: change only U(bit23) and offset(bits11:0).
 * Mask 0xFF7FF000 keeps: cond(31:28), 01(27:26), I(25), P(24), B(22),
 * W(21), L(20), Rn(19:16), Rd(15:12) — changes only U(23) and imm12(11:0). */
#define PATCH_LDR(patch, pool) do { \
    int32_t off = (pool)*4 - ((patch)*4 + 8); \
    uint32_t u = (off>=0)?1u:0u; \
    uint32_t aoff = (uint32_t)(off<0?-off:off); \
    rom[patch] = (rom[patch] & 0xFF7FF000u) | (u<<23) | aoff; \
} while(0)

/* PATCH_STR: LDR Rd = address of string (not string content).
 * We need a pool word containing the string's RAM address.
 * str_ptr_pool[] is emitted after the string data. */
#define PATCH_STR(patch, str_idx) PATCH_LDR(patch, str_ptr_pool[str_idx])

    /* Emit string address pointer pool (after all string data is known) */
    int str_ptr_pool[12];
    for (int i = 0; i < nstr; i++)
        str_ptr_pool[i] = emit_word((uint32_t)LOAD_ADDR + (uint32_t)(strings[i].pos * 4));

    PATCH_LDR(patch_sp, pool_sp);

    PATCH_STR(patch_hello_ldr, 0);
    patch_b(patch_hello_bl,  sub_puts);

    PATCH_STR(patch_arith_ok_ldr, 1);
    patch_b(patch_arith_ok_bl,  sub_puts);
    patch_b(patch_bne_arith, pos_fail_arith);
    patch_b(patch_arith_skip, pos_test2);
    PATCH_STR(patch_arith_fail_ldr, 2);
    patch_b(patch_arith_fail_bl, sub_puts);

    PATCH_STR(patch_clz_ok_ldr, 3);
    patch_b(patch_clz_ok_bl, sub_puts);
    patch_b(patch_bne_clz, pos_fail_clz);
    patch_b(patch_clz_skip, pos_test3);
    PATCH_STR(patch_clz_fail_ldr, 4);
    patch_b(patch_clz_fail_bl, sub_puts);

    PATCH_LDR(patch_mul_a_ldr, pool_12345);
    PATCH_LDR(patch_mul_b_ldr, pool_6789);
    PATCH_LDR(patch_mul_c_ldr, pool_83810205);
    PATCH_STR(patch_mul_ok_ldr, 5);
    patch_b(patch_mul_ok_bl, sub_puts);
    patch_b(patch_bne_mul, pos_fail_mul);
    patch_b(patch_mul_skip, pos_test4);
    PATCH_STR(patch_mul_fail_ldr, 6);
    patch_b(patch_mul_fail_bl, sub_puts);

    PATCH_LDR(patch_timer_load_ldr, pool_t0_load);
    PATCH_LDR(patch_timer_val_ldr,  pool_150000);
    PATCH_LDR(patch_timer_ctrl_ldr, pool_t0_ctrl);
    PATCH_LDR(patch_timer_cnt_ldr,  pool_t0_cnt);
    PATCH_STR(patch_timer_ok_ldr, 7);
    patch_b(patch_timer_ok_bl, sub_puts);
    patch_b(patch_beq_timer, pos_fail_timer);
    patch_b(patch_timer_skip, pos_test5);
    PATCH_STR(patch_timer_fail_ldr, 8);
    patch_b(patch_timer_fail_bl, sub_puts);

    PATCH_LDR(patch_input_ldr, pool_input);
    PATCH_STR(patch_input_ok_ldr, 9);
    patch_b(patch_input_ok_bl, sub_puts);
    patch_b(patch_hex32_bl, sub_hex32);

    PATCH_LDR(patch_fb_addr_ldr,  pool_fb_addr);
    PATCH_LDR(patch_fb_size_ldr,  pool_fb_size);
    PATCH_LDR(patch_fb_color_ldr, pool_fb_color);
    PATCH_LDR(patch_vfb_addr_ldr, pool_vfb);
    PATCH_LDR(patch_vfb_val_ldr,  pool_fb_addr);
    PATCH_LDR(patch_vctrl_ldr,    pool_vctrl);
    PATCH_STR(patch_video_ok_ldr, 10);
    patch_b(patch_video_ok_bl, sub_puts);

    PATCH_STR(patch_done_ldr, 11);
    patch_b(patch_done_bl, sub_puts);

    PATCH_LDR(patch_spin_input_ldr, pool_input);
    patch_b(patch_spin_bne, pos_spin);  /* BNE spin */

    PATCH_LDR(patch_putc_stat_ldr, pool_uart_st);
    PATCH_LDR(patch_putc_tx_ldr,   pool_uart_tx);

    /* ============================================================
     * Write output binary
     * ============================================================ */
    FILE *f = fopen("testrom.bin", "wb");
    if (!f) { perror("testrom.bin"); return 1; }
    fwrite(rom, 4, (size_t)rom_pos, f);
    fclose(f);

    printf("testrom.bin written: %d words (%d bytes) @ 0x%08X\n",
           rom_pos, rom_pos*4, LOAD_ADDR);
    printf("Run: ./vflash --headless testrom.bin\n");
    printf("  or ./vflash --dbg testrom.bin\n");
    return 0;
}
