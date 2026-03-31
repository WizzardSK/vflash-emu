#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* V.Flash Hardware Specs
 * CPU: LSI Logic ZEVIO 1020 SoC, ARM926EJ-S @ 150MHz
 * RAM: 16MB SDRAM
 * Media: CD-ROM, ISO 9660, no copy protection
 * Video: Motion JPEG (.mjp), raw picture (.ptx)
 * Audio: PCM WAV (.snd)
 * OS: uMORE v4.0 RTOS
 */

#define VFLASH_RAM_SIZE     (16 * 1024 * 1024)  /* 16MB SDRAM */
#define VFLASH_ROM_SIZE     (512 * 1024)         /* 512KB boot ROM (estimated) */
#define VFLASH_SRAM_BASE    0x0FFE0000           /* Internal SRAM / TCM */
#define VFLASH_SRAM_SIZE    (128 * 1024)         /* 128KB (estimated) */
#define VFLASH_SCREEN_W     320
#define VFLASH_SCREEN_H     240

/* Memory map */
#define VFLASH_ROM_BASE     0x00000000
#define VFLASH_RAM_BASE     0x10000000
#define VFLASH_IO_BASE      0x80000000

/* ZEVIO 1020 I/O regions (verified from RTOS RE) */
#define ZEVIO_TIMER_BASE    0xB000000C  /* Timer block: 64 timers, stride 0x40 */
#define ZEVIO_INTC_BASE     0xB0001000  /* SoC interrupt controller */
#define ZEVIO_VIC_BASE      0xDC000000  /* ARM VIC (IRQ=+0x00, FIQ=+0x100) */

/* Legacy I/O (estimated) */
#define VFLASH_IO_CDROM     0x80001000
#define VFLASH_IO_VIDEO     0x80002000
#define VFLASH_IO_AUDIO     0x80003000
#define VFLASH_IO_INPUT     0x80004000

/* Input buttons */
#define VFLASH_BTN_UP       (1 << 0)
#define VFLASH_BTN_DOWN     (1 << 1)
#define VFLASH_BTN_LEFT     (1 << 2)
#define VFLASH_BTN_RIGHT    (1 << 3)
#define VFLASH_BTN_RED      (1 << 4)
#define VFLASH_BTN_YELLOW   (1 << 5)
#define VFLASH_BTN_GREEN    (1 << 6)
#define VFLASH_BTN_BLUE     (1 << 7)
#define VFLASH_BTN_ENTER    (1 << 8)

typedef struct VFlash    VFlash;

/* JIT accessors — allows jit.c to access VFlash internals */
void    *vflash_get_cpu(VFlash *vf);   /* returns ARM9* */
void    *vflash_get_timer(VFlash *vf); /* returns ZevioTimer* */
uint8_t *vflash_get_ram(VFlash *vf);
typedef struct ARM9      ARM9;
typedef struct CDROM     CDROM;
typedef struct MJPDecoder MJPDecoder;

VFlash* vflash_create(const char *disc_path);
void    vflash_destroy(VFlash *vf);
void    vflash_run_frame(VFlash *vf);
void    vflash_set_input(VFlash *vf, uint32_t buttons);
uint32_t* vflash_get_framebuffer(VFlash *vf);
void    vflash_init_audio(VFlash *vf);
void    vflash_set_debug(VFlash *vf, int on);

/* ---- Debugger API ---- */
/* Access CPU state */
uint32_t  vflash_get_pc(VFlash *vf);
uint32_t  vflash_get_reg(VFlash *vf, int r);
void      vflash_set_reg(VFlash *vf, int r, uint32_t val);
uint32_t  vflash_get_cpsr(VFlash *vf);
int       vflash_is_thumb(VFlash *vf);
/* Read/write RAM directly (addr is virtual, returns 0 if unmapped) */
uint32_t  vflash_read32(VFlash *vf, uint32_t addr);
uint8_t   vflash_read8(VFlash *vf, uint32_t addr);
void      vflash_write32(VFlash *vf, uint32_t addr, uint32_t val);
/* Execute exactly one instruction; returns cycles consumed */
int       vflash_step(VFlash *vf);
/* Breakpoints (max 16) */
void      vflash_bp_set(VFlash *vf, uint32_t addr);
void      vflash_bp_clear(VFlash *vf, uint32_t addr);
void      vflash_bp_clear_all(VFlash *vf);
int       vflash_bp_hit(VFlash *vf);   /* 1 if last step hit a breakpoint */
uint32_t  vflash_bp_list(VFlash *vf, uint32_t *out, int maxn);
/* Run until breakpoint or frame end; returns 1 if bp hit */
int       vflash_run_until_bp(VFlash *vf, int max_cycles);
