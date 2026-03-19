/* testrom.S — V.Flash emulator test ROM
 *
 * Bare ARM32 binary loaded at 0x10000000 by HLE boot.
 * Tests every major subsystem and reports via UART ([UART] on stderr).
 *
 * Build:
 *   arm-linux-gnueabi-as -march=armv5te testrom.S -o testrom.o
 *   arm-linux-gnueabi-ld -Ttext=0x10000000 --oformat=binary testrom.o -o testrom.bin
 *
 * Run:
 *   ./vflash --headless testrom.bin   (if we extend HLE to accept raw .bin)
 *   ./vflash --dbg testrom.bin
 *
 * I/O addresses (estimated ZEVIO 1020):
 *   0x80000000  IRQ status
 *   0x80000004  IRQ enable
 *   0x80000008  IRQ clear
 *   0x80000100  Timer0 load
 *   0x80000108  Timer0 ctrl  (bit0=enable, bit1=periodic, bit2=irq_en)
 *   0x8000010C  Timer0 clear
 *   0x80001000  CD-ROM DMA (not tested here)
 *   0x80002000  Video FB addr
 *   0x80002004  Video ctrl   (bit1=DMA trigger)
 *   0x80004000  Input data   (active-low: 0=pressed)
 *   0x80005000  UART TX
 *   0x80005004  UART status  (bit0=TX_READY)
 */

.syntax unified
.arch   armv5te
.arm

/* I/O base */
.equ IO_BASE,       0x80000000
.equ UART_TX,       0x80005000
.equ UART_STATUS,   0x80005004
.equ TIMER0_LOAD,   0x80000100
.equ TIMER0_COUNT,  0x80000104
.equ TIMER0_CTRL,   0x80000108
.equ TIMER0_CLEAR,  0x8000010C
.equ IRQ_ENABLE,    0x80000004
.equ IRQ_CLEAR,     0x80000008
.equ VIDEO_FB,      0x80002000
.equ VIDEO_CTRL,    0x80002004
.equ INPUT_DATA,    0x80004000

.equ RAM_BASE,      0x10000000
.equ FRAMEBUF,      0x10100000   /* 1MB into RAM — 320*240*4 = 307200 bytes */

/* ============================================================ */
.section .text
.global _start
_start:
    /* SYS mode, IRQ/FIQ enabled */
    MOV  sp, #0x11000000        @ stack at top of RAM

    /* ---- Test 1: UART output ---- */
    LDR  r0, =str_hello
    BL   uart_puts

    /* ---- Test 2: Arithmetic ---- */
    MOV  r0, #42
    MOV  r1, #58
    ADD  r2, r0, r1             @ r2 = 100
    CMP  r2, #100
    BNE  fail_arith
    LDR  r0, =str_arith_ok
    BL   uart_puts
    B    test3
fail_arith:
    LDR  r0, =str_arith_fail
    BL   uart_puts

    /* ---- Test 3: CLZ ---- */
test3:
    MOV  r0, #0x00800000
    CLZ  r1, r0                 @ should be 8
    CMP  r1, #8
    BNE  fail_clz
    LDR  r0, =str_clz_ok
    BL   uart_puts
    B    test4
fail_clz:
    LDR  r0, =str_clz_fail
    BL   uart_puts

    /* ---- Test 4: Multiply ---- */
test4:
    MOV  r0, #12345
    MOV  r1, #6789
    MUL  r2, r0, r1             @ r2 = 83810205
    LDR  r3, =83810205
    CMP  r2, r3
    BNE  fail_mul
    LDR  r0, =str_mul_ok
    BL   uart_puts
    B    test5
fail_mul:
    LDR  r0, =str_mul_fail
    BL   uart_puts

    /* ---- Test 5: LDM/STM roundtrip ---- */
test5:
    LDR  r8, =test_data_addr
    STMIA r8, {r0-r7}           @ save r0-r7 to RAM
    MOV  r0, #0xAA
    MOV  r1, #0xBB
    MOV  r2, #0xCC
    LDMIA r8, {r0-r7}           @ restore (values from before STMIA)
    LDR  r0, =str_ldm_ok
    BL   uart_puts

    /* ---- Test 6: Timer read ---- */
test6:
    LDR  r2, =TIMER0_LOAD
    LDR  r3, =150000            @ 1ms at 150MHz
    STR  r3, [r2]
    LDR  r2, =TIMER0_CTRL
    MOV  r3, #0x3               @ enable + periodic
    STR  r3, [r2]
    LDR  r2, =TIMER0_COUNT
    LDR  r4, [r2]               @ read count
    CMP  r4, #0
    BEQ  fail_timer             @ should not be zero right after load
    LDR  r0, =str_timer_ok
    BL   uart_puts
    B    test7
fail_timer:
    LDR  r0, =str_timer_fail
    BL   uart_puts

    /* ---- Test 7: Input read (check it returns something) ---- */
test7:
    LDR  r2, =INPUT_DATA
    LDR  r3, [r2]               @ read buttons (active-low: ~0x1FF = no buttons)
    LDR  r0, =str_input_ok
    BL   uart_puts
    /* Print input value as hex */
    MOV  r0, r3
    BL   uart_hex32

    /* ---- Test 8: Video DMA — fill framebuffer with test pattern ---- */
test8:
    LDR  r0, =FRAMEBUF
    LDR  r1, =#307200           @ 320*240*4 bytes
    MOV  r2, #0                 @ loop counter
    LDR  r3, =0xFF4488CC        @ blue-ish ARGB test colour
fill_loop:
    STR  r3, [r0, r2]
    ADD  r2, r2, #4
    CMP  r2, r1
    BLT  fill_loop

    /* Tell video DMA where our framebuffer is */
    LDR  r2, =VIDEO_FB
    LDR  r3, =FRAMEBUF
    STR  r3, [r2]
    /* Trigger DMA blit */
    LDR  r2, =VIDEO_CTRL
    MOV  r3, #0x3               @ enable + DMA trigger
    STR  r3, [r2]

    LDR  r0, =str_video_ok
    BL   uart_puts

    /* ---- Done ---- */
    LDR  r0, =str_done
    BL   uart_puts

    /* Spin loop — keep running so debugger can inspect state */
spin:
    LDR  r2, =INPUT_DATA
    LDR  r3, [r2]
    /* Check if RED button pressed (bit 4 = 0 in active-low) */
    TST  r3, #0x10
    BEQ  spin               @ not pressed, keep spinning
    B    _start             @ RED pressed = restart

/* ============================================================
 * uart_puts: print null-terminated string at r0
 * Clobbers: r0, r1, r2, r3
 * ============================================================ */
uart_puts:
    PUSH {r4, lr}
    MOV  r4, r0
1:
    LDRB r0, [r4], #1
    CMP  r0, #0
    BEQ  2f
    BL   uart_putc
    B    1b
2:
    POP  {r4, pc}

/* uart_putc: send character in r0 via UART */
uart_putc:
    PUSH {r1, r2, lr}
    LDR  r1, =UART_STATUS
1:  LDR  r2, [r1]
    TST  r2, #1             @ TX_READY
    BEQ  1b
    LDR  r1, =UART_TX
    STR  r0, [r1]
    POP  {r1, r2, pc}

/* uart_hex32: print r0 as "0xNNNNNNNN\n" */
uart_hex32:
    PUSH {r4, r5, lr}
    MOV  r4, r0
    MOV  r0, #'0'
    BL   uart_putc
    MOV  r0, #'x'
    BL   uart_putc
    MOV  r5, #28
hex_loop:
    MOV  r0, r4, LSR r5
    AND  r0, r0, #0xF
    CMP  r0, #10
    ADDGE r0, r0, #'A'-10
    ADDLT r0, r0, #'0'
    BL   uart_putc
    SUBS r5, r5, #4
    BGE  hex_loop
    MOV  r0, #'\n'
    BL   uart_putc
    POP  {r4, r5, pc}

/* ============================================================
 * Strings
 * ============================================================ */
.section .rodata
str_hello:      .asciz "VFlash Test ROM v1.0\n"
str_arith_ok:   .asciz "PASS: arithmetic (42+58=100)\n"
str_arith_fail: .asciz "FAIL: arithmetic\n"
str_clz_ok:     .asciz "PASS: CLZ\n"
str_clz_fail:   .asciz "FAIL: CLZ\n"
str_mul_ok:     .asciz "PASS: MUL\n"
str_mul_fail:   .asciz "FAIL: MUL\n"
str_ldm_ok:     .asciz "PASS: LDM/STM\n"
str_timer_ok:   .asciz "PASS: timer read\n"
str_timer_fail: .asciz "FAIL: timer read\n"
str_input_ok:   .asciz "PASS: input read = "
str_video_ok:   .asciz "PASS: video DMA triggered\n"
str_done:       .asciz "ALL TESTS DONE\n"

.section .bss
test_data_addr: .space 32      @ 8 words for LDM/STM test
