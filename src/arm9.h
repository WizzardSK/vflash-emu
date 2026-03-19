#pragma once
#include <stdint.h>
#include "cp15.h"

/* ARM926EJ-S CPU skeleton
 * Full ARMv5TEJ instruction set
 * Thumb support
 * DSP extensions
 * Java accelerator (Jazelle)
 */

#define ARM9_MODE_USR  0x10
#define ARM9_MODE_FIQ  0x11
#define ARM9_MODE_IRQ  0x12
#define ARM9_MODE_SVC  0x13
#define ARM9_MODE_ABT  0x17
#define ARM9_MODE_UND  0x1B
#define ARM9_MODE_SYS  0x1F

#define ARM9_FLAG_N    (1 << 31)  /* Negative */
#define ARM9_FLAG_Z    (1 << 30)  /* Zero */
#define ARM9_FLAG_C    (1 << 29)  /* Carry */
#define ARM9_FLAG_V    (1 << 28)  /* Overflow */
#define ARM9_FLAG_I    (1 << 7)   /* IRQ disable */
#define ARM9_FLAG_F    (1 << 6)   /* FIQ disable */
#define ARM9_FLAG_T    (1 << 5)   /* Thumb mode */

struct ARM9 {
    uint32_t r[16];     /* r0-r15 (r15=PC, r14=LR, r13=SP) */
    uint32_t cpsr;      /* Current Program Status Register */
    uint32_t spsr;      /* Saved Program Status Register */

    /* Banked registers per mode */
    uint32_t r8_fiq,  r9_fiq,  r10_fiq, r11_fiq, r12_fiq;
    uint32_t r13_fiq, r14_fiq, spsr_fiq;
    uint32_t r13_irq, r14_irq, spsr_irq;
    uint32_t r13_svc, r14_svc, spsr_svc;
    uint32_t r13_abt, r14_abt, spsr_abt;
    uint32_t r13_und, r14_und, spsr_und;

    uint64_t cycles;
    int thumb;

    CP15     cp15;   /* System control coprocessor */

    /* Memory callbacks */
    void     *mem_ctx;
    uint32_t (*mem_read32)(void *ctx, uint32_t addr);
    uint16_t (*mem_read16)(void *ctx, uint32_t addr);
    uint8_t  (*mem_read8) (void *ctx, uint32_t addr);
    void     (*mem_write32)(void *ctx, uint32_t addr, uint32_t val);
    void     (*mem_write16)(void *ctx, uint32_t addr, uint16_t val);
    void     (*mem_write8) (void *ctx, uint32_t addr, uint8_t  val);
};
typedef struct ARM9 ARM9;

void     arm9_reset(ARM9 *cpu);
int      arm9_step(ARM9 *cpu);   /* returns cycles consumed */
void     arm9_run(ARM9 *cpu, int cycles);
void     arm9_irq(ARM9 *cpu);
void     arm9_fiq(ARM9 *cpu);
void     arm9_swi(ARM9 *cpu);
void     arm9_undef(ARM9 *cpu);
uint32_t arm9_get_pc(ARM9 *cpu);
