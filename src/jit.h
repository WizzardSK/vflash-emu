/* JIT compiler for ARM926EJ-S → x86_64 translation.
 * Basic block JIT with inline RAM fast path and interpreter fallback. */
#ifndef JIT_H
#define JIT_H

#include <stdint.h>

typedef struct ARM9 ARM9;
typedef struct VFlash VFlash;

/* Block cache entry */
typedef struct JitBlock {
    uint32_t arm_pc;        /* ARM entry address */
    uint8_t *native_code;   /* Pointer into code cache */
    uint32_t native_size;   /* Size of generated x86_64 code */
    uint32_t arm_insn_count;/* Number of ARM instructions in block */
    uint32_t next_pc[2];    /* Branch targets: [0]=taken, [1]=not-taken */
} JitBlock;

/* JIT context */
typedef struct JitContext {
    /* Code cache: mmap'd executable memory */
    uint8_t *code_cache;
    uint32_t cache_size;
    uint32_t cache_used;

    /* Block hash map: ARM PC → JitBlock */
    JitBlock *blocks;       /* Hash table */
    uint32_t block_count;
    uint32_t block_capacity;

    /* Page tracking for self-modifying code */
    uint8_t code_pages[4096]; /* 4KB pages over 16MB RAM */

    /* Pointer to VFlash for memory callbacks */
    VFlash *vf;

    /* Statistics */
    uint64_t blocks_compiled;
    uint64_t blocks_executed;
    uint64_t insns_executed;
} JitContext;

/* API */
JitContext *jit_create(VFlash *vf);
void        jit_destroy(JitContext *jit);

/* Run ARM code via JIT for 'cycles' instructions. Returns actual count. */
int         jit_run(JitContext *jit, int cycles);

/* Invalidate translated code for a RAM page (called on write to code page) */
void        jit_invalidate_page(JitContext *jit, uint32_t page_idx);

/* Invalidate all translated code */
void        jit_flush(JitContext *jit);

/* Compile a basic block starting at given ARM PC */
JitBlock   *jit_compile_block(JitContext *jit, uint32_t arm_pc);

/* Lookup a block in the cache */
JitBlock   *jit_lookup(JitContext *jit, uint32_t arm_pc);

#endif /* JIT_H */
