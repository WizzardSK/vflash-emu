#include "ztimer.h"
#include <string.h>
#include <stdio.h>

/* Register offsets */
#define REG_IRQ_STATUS  0x000
#define REG_IRQ_ENABLE  0x004
#define REG_IRQ_CLEAR   0x008
#define REG_FIQ_SELECT  0x00C
#define REG_T0_LOAD     0x100
#define REG_T0_COUNT    0x104
#define REG_T0_CTRL     0x108
#define REG_T0_CLEAR    0x10C
#define REG_T1_LOAD     0x120
#define REG_T1_COUNT    0x124
#define REG_T1_CTRL     0x128
#define REG_T1_CLEAR    0x12C

void ztimer_reset(ZevioTimer *zt) {
    memset(zt, 0, sizeof(ZevioTimer));
    printf("[ZTIMER] Reset\n");
}

void ztimer_tick(ZevioTimer *zt, uint32_t cycles) {
    zt->cycles += cycles;

    for (int i = 0; i < 2; i++) {
        Timer *t = &zt->timer[i];
        if (!(t->ctrl & 1)) continue;  /* Not enabled */

        if (t->count <= cycles) {
            t->count = (t->ctrl & 2) ? t->load : 0;  /* Reload or stop */
            if (t->ctrl & 4) {  /* IRQ enabled */
                t->irq_pending = 1;
                ztimer_raise_irq(zt, IRQ_TIMER0 + i);
            }
        } else {
            t->count -= cycles;
        }
    }
}

int ztimer_irq_pending(ZevioTimer *zt) {
    return (zt->irq.status & zt->irq.enable & ~zt->irq.fiq_sel) != 0;
}

int ztimer_fiq_pending(ZevioTimer *zt) {
    return (zt->irq.status & zt->irq.enable & zt->irq.fiq_sel) != 0;
}

void ztimer_raise_irq(ZevioTimer *zt, int irq) {
    zt->irq.status |= (1u << irq);
}

void ztimer_clear_irq(ZevioTimer *zt, int irq) {
    zt->irq.status &= ~(1u << irq);
}

uint32_t ztimer_read(ZevioTimer *zt, uint32_t reg) {
    switch (reg) {
        case REG_IRQ_STATUS: return zt->irq.status;
        case REG_IRQ_ENABLE: return zt->irq.enable;
        case REG_FIQ_SELECT: return zt->irq.fiq_sel;
        case REG_T0_LOAD:    return zt->timer[0].load;
        case REG_T0_COUNT:   return zt->timer[0].count;
        case REG_T0_CTRL:    return zt->timer[0].ctrl;
        case REG_T1_LOAD:    return zt->timer[1].load;
        case REG_T1_COUNT:   return zt->timer[1].count;
        case REG_T1_CTRL:    return zt->timer[1].ctrl;
        default:
            fprintf(stderr, "[ZTIMER] Unknown read reg 0x%03X\n", reg);
            return 0;
    }
}

void ztimer_write(ZevioTimer *zt, uint32_t reg, uint32_t val) {
    switch (reg) {
        case REG_IRQ_ENABLE: zt->irq.enable = val; break;
        case REG_IRQ_CLEAR:  zt->irq.status &= ~val; break;
        case REG_FIQ_SELECT: zt->irq.fiq_sel = val; break;
        case REG_T0_LOAD:    zt->timer[0].load = zt->timer[0].count = val; break;
        case REG_T0_CTRL:    zt->timer[0].ctrl = val; break;
        case REG_T0_CLEAR:   zt->timer[0].irq_pending = 0; ztimer_clear_irq(zt, IRQ_TIMER0); break;
        case REG_T1_LOAD:    zt->timer[1].load = zt->timer[1].count = val; break;
        case REG_T1_CTRL:    zt->timer[1].ctrl = val; break;
        case REG_T1_CLEAR:   zt->timer[1].irq_pending = 0; ztimer_clear_irq(zt, IRQ_TIMER1); break;
        default:
            fprintf(stderr, "[ZTIMER] Unknown write reg 0x%03X = 0x%08X\n", reg, val);
    }
}
