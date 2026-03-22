#pragma once
#include <stdint.h>

/* ARM926EJ-S CP15 System Control Coprocessor
 * Handles: MMU, caches, TCM, protection, clock
 */

typedef struct {
    /* c0 - ID registers (read-only) */
    uint32_t id;           /* Main ID */
    uint32_t cache_type;   /* Cache type */
    uint32_t tcm_status;   /* TCM status */

    /* c1 - Control register */
    uint32_t control;      /* MMU/cache enable bits */

    /* c2 - Translation table base */
    uint32_t ttb;          /* Translation Table Base */

    /* c3 - Domain access control */
    uint32_t domain;

    /* c5 - Fault status */
    uint32_t dfsr;         /* Data Fault Status */
    uint32_t ifsr;         /* Instruction Fault Status */

    /* c6 - Fault address */
    uint32_t dfar;         /* Data Fault Address */

    /* c7 - Cache/TLB operations (write-only) */

    /* c8 - TLB operations (write-only) */

    /* c9 - Cache lockdown */
    uint32_t dtcm_base;    /* Data TCM base/size */
    uint32_t itcm_base;    /* Instruction TCM base/size */

    /* c13 - Process ID */
    uint32_t pid;          /* FCSE PID */
    uint32_t context_id;

    /* Internal state */
    int mmu_enabled;
    int dcache_enabled;
    int icache_enabled;
    int write_buffer;
    int hivec;          /* 1 = vectors at 0xFFFF0000, 0 = vectors at 0x00000000 */
    int tlb_flush_needed; /* Set by MCR c8, checked by mmu_translate */
} CP15;

/* Control register bits */
#define CP15_CTRL_MMU     (1 << 0)
#define CP15_CTRL_ALIGN   (1 << 1)
#define CP15_CTRL_DCACHE  (1 << 2)
#define CP15_CTRL_WBUF    (1 << 3)
#define CP15_CTRL_BIGEND  (1 << 7)
#define CP15_CTRL_SYS     (1 << 8)
#define CP15_CTRL_ROM     (1 << 9)
#define CP15_CTRL_ICACHE  (1 << 12)
#define CP15_CTRL_HIVEC   (1 << 13)  /* High vectors at 0xFFFF0000 */
#define CP15_CTRL_RR      (1 << 14)  /* Round-robin cache replacement */

void     cp15_reset(CP15 *cp);
uint32_t cp15_read(CP15 *cp, uint32_t crn, uint32_t crm, uint32_t op2);
void     cp15_write(CP15 *cp, uint32_t crn, uint32_t crm, uint32_t op2, uint32_t val);
uint32_t cp15_translate(CP15 *cp, uint32_t vaddr, uint8_t *mem, uint32_t mem_base, uint32_t mem_size);
