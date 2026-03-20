#include "cp15.h"
#include <string.h>
#include <stdio.h>

/* ARM926EJ-S CP15 implementation
 * Reference: ARM926EJ-S Technical Reference Manual (ARM DDI 0198E)
 */

/* ARM926EJ-S specific ID values */
#define ARM926_MAIN_ID      0x41069265  /* ARM, v5TEJ, ARM926EJ-S */
#define ARM926_CACHE_TYPE   0x1D172172  /* 8KB I-cache, 8KB D-cache, write-back */

void cp15_reset(CP15 *cp) {
    memset(cp, 0, sizeof(CP15));

    /* c0 - ID registers */
    cp->id         = ARM926_MAIN_ID;
    cp->cache_type = ARM926_CACHE_TYPE;
    cp->tcm_status = 0x00010001;  /* 16KB DTCM, 16KB ITCM */

    /* c1 - Control: reset state has MMU/caches disabled */
    cp->control    = 0x00000078;  /* bits 3,4,5,6 are RAO (read as one) */

    cp->domain     = 0;
    cp->ttb        = 0;
    cp->pid        = 0;

    cp->mmu_enabled    = 0;
    cp->dcache_enabled = 0;
    cp->icache_enabled = 0;
    cp->write_buffer   = 0;
    cp->hivec          = 0;

    printf("[CP15] Reset: ID=0x%08X\n", cp->id);
}

uint32_t cp15_read(CP15 *cp, uint32_t crn, uint32_t crm, uint32_t op2) {
    switch (crn) {
        case 0:  /* ID registers */
            switch (op2) {
                case 0: return cp->id;
                case 1: return cp->cache_type;
                case 2: return cp->tcm_status;
                default: return 0;
            }
        case 1:  /* Control */
            return cp->control;
        case 2:  /* TTB */
            return cp->ttb;
        case 3:  /* Domain */
            return cp->domain;
        case 5:  /* Fault status */
            return (crm == 0) ? cp->dfsr : cp->ifsr;
        case 6:  /* Fault address */
            return cp->dfar;
        case 9:  /* TCM */
            return (op2 == 0) ? cp->dtcm_base : cp->itcm_base;
        case 13: /* PID / Context ID */
            return (op2 == 0) ? cp->pid : cp->context_id;
        default:
            fprintf(stderr, "[CP15] Read unknown CRn=%u CRm=%u op2=%u\n", crn, crm, op2);
            return 0;
    }
}

void cp15_write(CP15 *cp, uint32_t crn, uint32_t crm, uint32_t op2, uint32_t val) {
    switch (crn) {
        case 1:  /* Control register */
            cp->control        = (val & ~0x00000078u) | 0x00000078u; /* bits 3-6 RAO */
            cp->mmu_enabled    = (val & CP15_CTRL_MMU)    ? 1 : 0;
            cp->dcache_enabled = (val & CP15_CTRL_DCACHE) ? 1 : 0;
            cp->icache_enabled = (val & CP15_CTRL_ICACHE) ? 1 : 0;
            cp->write_buffer   = (val & CP15_CTRL_WBUF)   ? 1 : 0;
            cp->hivec          = (val & CP15_CTRL_HIVEC)  ? 1 : 0;
            /* Suppress repeated Control writes (µMORE context switch spam) */
            if (0) printf("[CP15] Control=0x%08X MMU=%d DC=%d IC=%d HIVEC=%d\n",
                   cp->control, cp->mmu_enabled, cp->dcache_enabled,
                   cp->icache_enabled, cp->hivec);
            break;
        case 2:  /* TTB */
            cp->ttb = val & ~0x3FFFu;
            printf("[CP15] TTB=0x%08X\n", cp->ttb);
            break;
        case 3:  /* Domain */
            cp->domain = val;
            break;
        case 5:  /* Fault status */
            if (crm == 0) cp->dfsr = val; else cp->ifsr = val;
            break;
        case 6:  /* Fault address */
            cp->dfar = val;
            break;
        case 7:  /* Cache operations */
            switch ((crm << 4) | op2) {
                case 0x00: break; /* WFI */
                case 0x40: break; /* Invalidate I-cache */
                case 0x60: break; /* Invalidate D-cache */
                case 0x6A: break; /* Clean D-cache by MVA */
                case 0x6B: break; /* Invalidate D-cache by index */
                case 0xEA: break; /* Clean+invalidate D-cache by MVA */
                case 0x5A: break; /* Drain write buffer */
                default: break;
            }
            break;
        case 8:  /* TLB operations */
            printf("[CP15] TLB op CRm=%u op2=%u val=0x%08X\n", crm, op2, val);
            break;
        case 9:  /* TCM */
            if (op2 == 0) { cp->dtcm_base = val; printf("[CP15] DTCM base=0x%08X\n", val); }
            else           { cp->itcm_base = val; printf("[CP15] ITCM base=0x%08X\n", val); }
            break;
        case 13: /* PID */
            if (op2 == 0) cp->pid = val;
            else cp->context_id = val;
            break;
        default:
            fprintf(stderr, "[CP15] Write unknown CRn=%u CRm=%u op2=%u val=0x%08X\n",
                    crn, crm, op2, val);
    }
}

/* Simple MMU translation — first-level descriptor lookup
 * V.Flash likely uses flat section mapping (1MB sections)
 * Full page table walk not needed for basic operation */
uint32_t cp15_translate(CP15 *cp, uint32_t vaddr,
                        uint8_t *mem, uint32_t mem_base, uint32_t mem_size) {
    if (!cp->mmu_enabled) return vaddr;

    /* Get first-level descriptor */
    uint32_t ttb      = cp->ttb;
    uint32_t l1_index = vaddr >> 20;
    uint32_t l1_pa    = ttb + (l1_index << 2);

    if (l1_pa < mem_base || l1_pa + 4 > mem_base + mem_size)
        return vaddr;  /* Out of range — pass through */

    uint32_t desc = *(uint32_t*)(mem + (l1_pa - mem_base));
    uint32_t type = desc & 3;

    switch (type) {
        case 0: /* Fault */
            fprintf(stderr, "[CP15] MMU fault: vaddr=0x%08X L1[%u]=0x%08X\n",
                    vaddr, l1_index, desc);
            return vaddr;
        case 1: /* Coarse page table — not implemented, pass through */
            return vaddr;
        case 2: /* Section (1MB) */
            return (desc & 0xFFF00000u) | (vaddr & 0x000FFFFFu);
        case 3: /* Fine page table — not implemented, pass through */
            return vaddr;
    }
    return vaddr;
}
