#pragma once
#include <stdint.h>

/* ZEVIO 1020 Interrupt Controller (estimated)
 * Based on typical ARM SoC designs of the era
 */

#define IRQ_TIMER0   0
#define IRQ_TIMER1   1
#define IRQ_CDROM    2
#define IRQ_AUDIO    3
#define IRQ_INPUT    4
#define IRQ_DMA      5
#define IRQ_COUNT    16

typedef struct {
    uint32_t enable;    /* IRQ enable mask */
    uint32_t status;    /* Pending IRQs */
    uint32_t fiq_sel;   /* Which IRQs route to FIQ */
} IRQCtrl;

/* Timer — two 32-bit countdown timers */
typedef struct {
    uint32_t load;      /* Reload value */
    uint32_t count;     /* Current count */
    uint32_t ctrl;      /* Control: bit0=enable, bit1=periodic, bit2=irq_en */
    uint32_t irq_pending;
} Timer;

typedef struct {
    IRQCtrl  irq;
    Timer    timer[2];
    uint64_t cycles;
} ZevioTimer;

void     ztimer_reset(ZevioTimer *zt);
void     ztimer_tick(ZevioTimer *zt, uint32_t cycles);
int      ztimer_irq_pending(ZevioTimer *zt);
int      ztimer_fiq_pending(ZevioTimer *zt);
void     ztimer_clear_irq(ZevioTimer *zt, int irq);
void     ztimer_raise_irq(ZevioTimer *zt, int irq);

uint32_t ztimer_read(ZevioTimer *zt, uint32_t reg);
void     ztimer_write(ZevioTimer *zt, uint32_t reg, uint32_t val);
