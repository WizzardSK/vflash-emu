#include "vflash.h"
#include "arm9.h"
#include "cdrom.h"
#include "mjp.h"
#include "audio.h"
#include "ztimer.h"
#include "disasm.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* ============================================================
 * V.Flash System — ZEVIO 1020 SoC
 * ARM926EJ-S @ 150MHz, 16MB SDRAM
 * ============================================================ */

/* Load address for HLE boot — game code loaded here */
#define VFLASH_LOAD_ADDR  0x10000000  /* RAM base */
#define VFLASH_STACK_TOP  0x11000000  /* 16MB into RAM */

/* I/O region layout (estimated) */
#define IO_TIMER_BASE    0x80000100
#define IO_IRQ_BASE      0x80000000
#define IO_INPUT_BASE    0x80004000
#define IO_VIDEO_BASE    0x80002000
#define IO_AUDIO_BASE    0x80003000

/* UART stub — ZEVIO 1020 has at least one UART for debug output.
 * Estimated base address. Key registers:
 *   +0x00  UART_TX    : write byte to transmit
 *   +0x04  UART_STATUS: bit0=TX_READY, bit1=RX_READY
 *   +0x08  UART_RX    : read received byte
 *   +0x0C  UART_CTRL  : bit0=enable, bit1=loopback
 */
#define IO_UART_BASE     0x80005000

/* ARM exception vector offsets (byte 0x00000000) */
#define VEC_RESET   0x00  /* Reset */
#define VEC_UNDEF   0x04  /* Undefined instruction */
#define VEC_SWI     0x08  /* Software interrupt */
#define VEC_PABT    0x0C  /* Prefetch abort */
#define VEC_DABT    0x10  /* Data abort */
#define VEC_RESERVED 0x14
#define VEC_IRQ     0x18  /* IRQ */
#define VEC_FIQ     0x1C  /* FIQ */

/* HLE stub addresses — placed at TOP of RAM below stack, well away from
 * game code. load_raw() loads at VFLASH_RAM_BASE (0x10000000), so stubs
 * at 0x10000020 would get OVERWRITTEN. Use 0x10FFE000 instead (16MB-8KB). */
#define HLE_STUB_BASE   0x10FFE000
#define HLE_STUB_SIZE   0x100       /* reserve 256 bytes for stubs */

/* ---- I/O register model (ZEVIO 1020 SoC, estimated) ----
 *
 * 0x80000000 + 0x000..0x1FF  IRQ controller + timers   (ztimer)
 * 0x80000000 + 0x1000..0x1FFF  CD-ROM DMA controller
 *   +0x00  CD_CMD        : command (0=idle, 1=read sectors, 2=seek)
 *   +0x04  CD_LBA        : target LBA
 *   +0x08  CD_COUNT      : number of sectors to read
 *   +0x0C  CD_DST        : RAM destination address
 *   +0x10  CD_STATUS     : bit0=busy, bit1=done, bit2=error
 *   +0x14  CD_CTRL       : bit0=start DMA, bit1=abort
 *   +0x18  CD_SECTOR_SZ  : bytes per sector (2048 or 2352)
 * 0x80000000 + 0x2000..0x2FFF  Video DMA controller
 *   +0x00  VDISP_FBADDR   : game writes RAM addr of ARGB32 framebuffer
 *   +0x04  VDISP_CTRL     : bit0=enable, bit1=dma_trigger
 *   +0x08  VDISP_STATUS   : bit0=vsync, bit1=ready (read-only)
 *   +0x0C  VJPEG_SRC      : RAM addr of JPEG data
 *   +0x10  VJPEG_SIZE     : JPEG data size in bytes
 *   +0x14  VJPEG_CTRL     : write 1 → decode JPEG into VDISP_FBADDR buffer
 *   +0x18  VJPEG_STATUS   : bit0=busy, bit1=done
 *
 * 0x80000000 + 0x3000..0x3FFF  Audio DMA
 *   +0x00  AUD_SRC        : RAM addr of PCM/WAV data
 *   +0x04  AUD_SIZE       : size in bytes
 *   +0x08  AUD_CTRL       : bit0=play, bit1=loop, bit2=stereo, bit3=16bit
 *   +0x0C  AUD_STATUS     : bit0=playing, bit1=done
 *
 * 0x80000000 + 0x4000..0x4FFF  GPIO/Input
 *   +0x00  INPUT_DATA     : current button state (active high)
 *   +0x04  INPUT_STATUS   : bit0=changed since last read
 */

/* CD-ROM DMA state */
typedef struct {
    uint32_t cmd;
    uint32_t lba;
    uint32_t count;
    uint32_t dst;
    uint32_t status;
    uint32_t sector_sz;
} CDRomRegs;

/* Video DMA state */
typedef struct {
    uint32_t fb_addr;
    uint32_t ctrl;
    uint32_t jpeg_src;
    uint32_t jpeg_size;
    uint32_t jpeg_ctrl;
    uint32_t jpeg_status;
    int      fb_dirty;
} VideoRegs;

/* Audio DMA state */
typedef struct {
    uint32_t src;
    uint32_t size;
    uint32_t ctrl;
    int      triggered;
} AudioRegs;

struct VFlash {
    ARM9       cpu;
    CDROM     *cd;
    MJPDecoder *video;
    Audio     *audio;
    ZevioTimer timer;

    uint8_t  *ram;
    uint8_t  *sram;    /* Internal SRAM / TCM (128KB at 0x0FFE0000) */
    uint8_t  *rom;     /* Boot ROM (2MB, mapped at 0x00000000 and 0xB8000000) */
    uint32_t  rom_size;
    uint32_t  input;
    uint32_t  input_prev;
    int       debug;
    int       has_rom;    /* 1 if boot ROM loaded (real boot, no HLE) */
    uint32_t  flash_remap; /* Flash controller: remap address at 0xB8000800 */
    uint32_t  dma_param_a, dma_param_b, dma_param_c;
    uint64_t  frame_count;

    VideoRegs vid;
    AudioRegs aud;
    CDRomRegs cdr;

    /* Debugger state */
    uint32_t bp[16];   /* breakpoint addresses (0 = unused) */
    int      bp_hit;   /* set by vflash_step() when BP hit */

    uint32_t  framebuf[VFLASH_SCREEN_W * VFLASH_SCREEN_H];
};

/* ---- Memory bus ---- */

/* Helper: safe RAM read */
static inline uint32_t ram_r32(VFlash *vf, uint32_t addr) {
    if (addr >= VFLASH_RAM_BASE && addr + 3 < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE));
    return 0;
}

/* Read a physical 32-bit word (for page table walks) — no MMU translation */
static inline uint32_t phys_read32(VFlash *vf, uint32_t pa) {
    if (pa < VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + pa);
    if (pa >= VFLASH_RAM_BASE && pa + 3 < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + (pa - VFLASH_RAM_BASE));
    if (pa >= VFLASH_SRAM_BASE && pa + 3 < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return *(uint32_t*)(vf->sram + (pa - VFLASH_SRAM_BASE));
    return 0;
}

/* MMU virtual→physical address translation (ARM926EJ-S two-level page table)
 * Supports: section (1MB), coarse page table (4KB/64KB pages), fault.
 * Called on every memory access when MMU is enabled. */
static uint32_t mmu_translate(VFlash *vf, uint32_t va) {
    CP15 *cp = &vf->cpu.cp15;
    if (!cp->mmu_enabled)
        return va;

    /* L1 descriptor lookup */
    uint32_t l1_index = va >> 20;
    uint32_t l1_addr  = cp->ttb + (l1_index << 2);
    uint32_t l1_desc  = phys_read32(vf, l1_addr);
    uint32_t type     = l1_desc & 3;

    switch (type) {
        case 0: /* Fault — return VA unchanged (will hit unmapped handler) */
            return va;
        case 2: { /* Section (1MB) */
            uint32_t pa = (l1_desc & 0xFFF00000u) | (va & 0x000FFFFFu);
            if (pa != va && vf->debug)
                fprintf(stderr, "[MMU] VA 0x%08X → PA 0x%08X (section L1[%u]=0x%08X)\n",
                        va, pa, l1_index, l1_desc);
            return pa;
        }
        case 1: { /* Coarse page table (256 entries, 4KB pages) */
            uint32_t l2_base  = l1_desc & 0xFFFFFC00u;
            uint32_t l2_index = (va >> 12) & 0xFF;
            uint32_t l2_addr  = l2_base + (l2_index << 2);
            uint32_t l2_desc  = phys_read32(vf, l2_addr);
            uint32_t l2_type  = l2_desc & 3;
            switch (l2_type) {
                case 0: return va; /* Fault */
                case 1: /* Large page (64KB) */
                    return (l2_desc & 0xFFFF0000u) | (va & 0x0000FFFFu);
                case 2: /* Small page (4KB) */
                    return (l2_desc & 0xFFFFF000u) | (va & 0x00000FFFu);
                case 3: /* Tiny page (1KB) — rare */
                    return (l2_desc & 0xFFFFFC00u) | (va & 0x000003FFu);
            }
            return va;
        }
        case 3: /* Fine page table — very rare, treat as section */
            return (l1_desc & 0xFFF00000u) | (va & 0x000FFFFFu);
    }
    return va;
}

static uint32_t mem_read32(void *ctx, uint32_t addr) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);

    /* RAM mirror at 0x00000000 (aliased from VFLASH_RAM_BASE).
     * ZEVIO 1020 maps SDRAM at both 0x00000000 and 0x10000000.
     * This is essential: ARM exception vectors (0x00–0x1C) and
     * HIVEC vectors (0xFFFF0000+) must be readable from these addresses. */
    if (addr < VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + addr);

    /* High vector mirror at 0xFFFF0000–0xFFFFFFFF (ARM HIVEC, CR1.V=1)
     * Mirror the first 64KB of RAM here. The upper bound check is omitted
     * because 0xFFFF0000 + 0x10000 == 0 in uint32 (overflow). */
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;  /* 0x0000..0xFFFF */
        if (off + 3 < VFLASH_RAM_SIZE)
            return *(uint32_t*)(vf->ram + off);
        return 0;
    }

    /* RAM */
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE));

    /* Internal SRAM / TCM */
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return *(uint32_t*)(vf->sram + (addr - VFLASH_SRAM_BASE));

    if (addr >= VFLASH_IO_BASE) {
        uint32_t off = addr - VFLASH_IO_BASE;

        /* Primary IRQ + timers: 0x80000000+0x000–0x1FF */
        if (off < 0x200)
            return ztimer_read(&vf->timer, off);

        /* Secondary IRQ controller at 0xDC000000 (VIC)
         * mapped to same ztimer — games use this via MMU */
        if (off >= 0x5C000000u && off < 0x5C000200u)
            return ztimer_read(&vf->timer, off - 0x5C000000u);

        /* Boot ROM at 0xB8000000 (full 2MB).
         * Flash remap register at 0xB8000800: writing an address
         * redirects reads from 0xB8000800+ to that RAM address. */
        if (off >= 0x38000000u && off < 0x38200000u && vf->has_rom) {
            uint32_t foff = off - 0x38000000u;
            if (foff >= 0x800 && vf->flash_remap) {
                /* Flash remap: 0xB8000800+N maps to remap_addr+N.
                 * Init code self-modifies a small area (~64 bytes) at
                 * RAM[0x118-0x160] (the BL sequence and main init code).
                 * Everything else should come from original ROM.
                 *
                 * Use RAM for the self-modified region (N=0x00-0x50),
                 * ROM for the rest (BL function bodies, handlers). */
                uint32_t N = foff - 0x800;
                uint32_t remap_off = vf->flash_remap + N;
                if (N < 0x50) {
                    /* Self-modified BL sequence area → RAM */
                    if (remap_off < VFLASH_RAM_SIZE)
                        return *(uint32_t*)(vf->ram + remap_off);
                } else {
                    /* BL function bodies → original ROM */
                    if (remap_off < vf->rom_size)
                        return *(uint32_t*)(vf->rom + remap_off);
                }
                return 0;
            }
            if (foff < vf->rom_size)
                return *(uint32_t*)(vf->rom + foff);
            return 0;
        }

        /* CD-ROM DMA: 0x1000–0x1FFF */
        if (off >= 0x1000 && off < 0x2000) {
            switch (off - 0x1000) {
                case 0x00: return vf->cdr.cmd;
                case 0x04: return vf->cdr.lba;
                case 0x08: return vf->cdr.count;
                case 0x0C: return vf->cdr.dst;
                case 0x10: return vf->cdr.status;
                case 0x14: return 0;   /* CTRL write-only */
                case 0x18: return vf->cdr.sector_sz ? vf->cdr.sector_sz : 2048;
                default:   return 0;
            }
        }

        /* Video DMA: 0x2000–0x2FFF */
        if (off >= 0x2000 && off < 0x3000) {
            switch (off - 0x2000) {
                case 0x00: return vf->vid.fb_addr;
                case 0x04: return vf->vid.ctrl;
                case 0x08: return 0x3; /* VDISP_STATUS: vsync+ready */
                case 0x0C: return vf->vid.jpeg_src;
                case 0x10: return vf->vid.jpeg_size;
                case 0x14: return vf->vid.jpeg_ctrl;
                case 0x18: return vf->vid.jpeg_status;
                default:   return 0;
            }
        }

        /* Audio DMA: 0x3000–0x3FFF */
        if (off >= 0x3000 && off < 0x4000) {
            switch (off - 0x3000) {
                case 0x00: return vf->aud.src;
                case 0x04: return vf->aud.size;
                case 0x08: return vf->aud.ctrl;
                case 0x0C: return 0; /* AUD_STATUS: idle */
                default:   return 0;
            }
        }

        /* GPIO/Input: 0x4000–0x4FFF */
        if (off >= 0x4000 && off < 0x5000) {
            switch (off - 0x4000) {
                case 0x00: {
                    /* Active-low GPIO: 0 = pressed */
                    return (~vf->input) & 0x1FF;
                }
                case 0x04: {
                    uint32_t changed = (vf->input != vf->input_prev) ? 1 : 0;
                    vf->input_prev = vf->input;
                    return changed;
                }
                default: return 0xFFFFFFFF;
            }
        }

        /* ATAPI CD-ROM controller at 0xAA000000 (off = 0x2A000000) */
        if (off >= 0x2A000000u && off < 0x2A001000u) {
            uint32_t areg = off - 0x2A000000u;
            switch (areg) {
                case 0x02: return 0;     /* Sector count */
                case 0x03: return 0;     /* LBA low */
                case 0x05: return 0;     /* LBA mid/high */
                case 0x07: return 0x40;  /* Status: DRDY (drive ready) */
                case 0x17: return 0x40;  /* Alt status: DRDY */
                default:   return 0;
            }
        }

        /* DMA / CD-ROM controller at 0x8FFF0000 (off = 0x0FFF0000) */
        if (off >= 0x0FFF0000u && off < 0x0FFF1000u) {
            uint32_t dreg = off - 0x0FFF0000u;
            switch (dreg) {
                case 0x08: return 0x01;  /* DMA status: complete */
                default:   return 0;
            }
        }

        /* Timer A at 0x90010000 (off = 0x10010000) */
        if (off >= 0x10010000u && off < 0x10010100u)
            return ztimer_read(&vf->timer, 0x100 + (off - 0x10010000u));

        /* Timer B at 0x900C0000 (off = 0x100C0000) */
        if (off >= 0x100C0000u && off < 0x100C0100u)
            return ztimer_read(&vf->timer, 0x120 + (off - 0x100C0000u));

        /* System control at 0x900A0000-0x900BFFFF (off = 0x100A0000-0x100BFFFF) */
        if (off >= 0x100A0000u && off < 0x100C0000u) {
            uint32_t sreg = off - 0x100A0000u;
            switch (sreg) {
                case 0x000C: return 0x00;    /* Boot status: bit1=0 (cold boot → copy ROM to RAM) */
                case 0x10014: return 0x01;   /* PLL lock: bit0=1 (locked) */
                default:     return 0xFFFFFFFF; /* All ready/done bits set */
            }
        }

        /* UART stub: 0x5000–0x5FFF
         * Always report TX_READY so OS never gets stuck polling */
        if (off >= 0x5000 && off < 0x6000) {
            switch (off - 0x5000) {
                case 0x00: return 0;            /* UART_TX (write-only, reads 0) */
                case 0x04: return 0x1;          /* UART_STATUS: TX_READY always set */
                case 0x08: return 0xFF;         /* UART_RX: no data */
                case 0x0C: return 0x1;          /* UART_CTRL: enabled */
                default:   return 0;
            }
        }

        return 0;
    }

    return 0;
}

static uint16_t mem_read16(void *ctx, uint32_t addr) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);
    if (addr < VFLASH_RAM_SIZE)
        return *(uint16_t*)(vf->ram + addr);
    if (addr >= 0xFFFF0000u && addr - 0xFFFF0000u < VFLASH_RAM_SIZE)
        return *(uint16_t*)(vf->ram + (addr - 0xFFFF0000u));
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return *(uint16_t*)(vf->ram + (addr - VFLASH_RAM_BASE));
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return *(uint16_t*)(vf->sram + (addr - VFLASH_SRAM_BASE));
    if (addr >= VFLASH_IO_BASE)
        return (uint16_t)(mem_read32(ctx, addr & ~3u) >> ((addr & 2) * 8));
    return 0;
}

static uint8_t mem_read8(void *ctx, uint32_t addr) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);
    if (addr < VFLASH_RAM_SIZE)
        return vf->ram[addr];
    if (addr >= 0xFFFF0000u && addr - 0xFFFF0000u < VFLASH_RAM_SIZE)
        return vf->ram[addr - 0xFFFF0000u];
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return vf->ram[addr - VFLASH_RAM_BASE];
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return vf->sram[addr - VFLASH_SRAM_BASE];
    if (addr >= VFLASH_IO_BASE)
        return (uint8_t)(mem_read32(ctx, addr & ~3u) >> ((addr & 3) * 8));
    return 0;
}

static void mem_write32(void *ctx, uint32_t addr, uint32_t val) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);

    /* RAM mirror at 0x00000000 */
    if (addr < VFLASH_RAM_SIZE) {
        *(uint32_t*)(vf->ram + addr) = val;
        return;
    }

    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE)) = val;
        return;
    }

    /* Internal SRAM / TCM */
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE) {
        *(uint32_t*)(vf->sram + (addr - VFLASH_SRAM_BASE)) = val;
        return;
    }

    if (addr >= VFLASH_IO_BASE) {
        uint32_t off = addr - VFLASH_IO_BASE;

        /* Primary IRQ + timers */
        if (off < 0x200) { ztimer_write(&vf->timer, off, val); return; }

        /* Secondary IRQ controller at 0xDC000000 */
        if (off >= 0x5C000000u && off < 0x5C000200u) {
            ztimer_write(&vf->timer, off - 0x5C000000u, val); return;
        }

        /* ATAPI CD-ROM write at 0xAA000000 */
        if (off >= 0x2A000000u && off < 0x2A001000u) {
            uint32_t areg = off - 0x2A000000u;
            if (areg == 0x07) {
                /* Command register — ATAPI command issued */
                printf("[ATAPI] Command: 0x%02X\n", val & 0xFF);
            }
            return;
        }

        /* Timer A write at 0x90010000 */
        if (off >= 0x10010000u && off < 0x10010100u) {
            ztimer_write(&vf->timer, 0x100 + (off - 0x10010000u), val);
            return;
        }
        /* Timer B write at 0x900C0000 */
        if (off >= 0x100C0000u && off < 0x100C0100u) {
            ztimer_write(&vf->timer, 0x120 + (off - 0x100C0000u), val);
            return;
        }

        /* DMA controller write at 0x8FFF0000 */
        if (off >= 0x0FFF0000u && off < 0x0FFF1000u) {
            uint32_t dreg = off - 0x0FFF0000u;
            switch (dreg) {
                case 0x00: vf->dma_param_a = val; break;
                case 0x04: vf->dma_param_b = val; break;
                case 0x08:
                    /* DMA enable — trigger CD-ROM sector read into RAM.
                     * Params: A = LBA start, B = sector count, C = dest/config
                     * First call loads BOOT.BIN, subsequent calls load game data. */
                    if (val == 1 && vf->cd && vf->cd->is_open) {
                        static int dma_call = 0;
                        dma_call++;
                        /* DMA at 0x8FFF is likely NOT CD-ROM but memory/flash DMA.
                         * HLE: load BOOT.BIN from disc on first call.
                         * Params A/B/C are flash controller config, not LBA. */
                        if (dma_call == 1) {
                            /* Copy full ROM to RAM[0x10000] (0x10010000) and
                             * also load BOOT.BIN at its load address. ROM expects
                             * its own code at 0x10010000+ for µMORE init. */
                            if (vf->rom && vf->rom_size > 0) {
                                uint32_t dest = 0x10000;
                                uint32_t sz = vf->rom_size;
                                if (dest + sz > VFLASH_RAM_SIZE) sz = VFLASH_RAM_SIZE - dest;
                                memcpy(vf->ram + dest, vf->rom, sz);
                                printf("[DMA#%d] ROM: %u bytes → RAM[0x%X]\n",
                                       dma_call, sz, dest);
                            }
                            /* Also load BOOT.BIN at its load address */
                            CDEntry entry;
                            if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &entry)) {
                                uint32_t dest = 0xC00000;
                                uint32_t max_sz = VFLASH_RAM_SIZE - dest;
                                if (entry.size < max_sz) max_sz = entry.size;
                                int rd = cdrom_read_file(vf->cd, &entry,
                                    vf->ram + dest, 0, max_sz);
                                printf("[DMA#%d] BOOT.BIN: %d bytes → RAM[0x%X]\n",
                                       dma_call, rd, dest);
                            }

                            /* Force-enable timer and IRQ for µMORE scheduler.
                             * On real HW, ROM BL #1 enables these, but with
                             * self-modifying code the enable sequence may
                             * not execute properly in emulation. */
                            vf->timer.timer[0].load = 150000; /* 1ms @ 150MHz */
                            vf->timer.timer[0].count = 150000;
                            vf->timer.timer[0].ctrl = 0x77; /* enable+periodic+IRQ */
                            vf->timer.irq.enable = 0xFFFFFFFF; /* all IRQs enabled */
                            /* Don't force CPSR here — too early, IRQ SP not set.
                             * IRQ enable happens in run_frame after init settles. */
                            printf("[DMA] Force-enabled timer0 + IRQ controller\n");
                        }
                    }
                    break;
                case 0x0C: vf->dma_param_c = val; break;
                case 0x10: break;  /* config */
            }
            return;
        }

        /* Flash controller write at 0xB8000800 (remap register) */
        if (off >= 0x38000000u && off < 0x38200000u && vf->has_rom) {
            uint32_t foff = off - 0x38000000u;
            if (foff == 0x800) {
                vf->flash_remap = val;
                printf("[ROM] Flash remap: 0x%08X\n", val);
            }
            return;
        }

        /* CD-ROM DMA: 0x1000–0x1FFF */
        if (off >= 0x1000 && off < 0x2000) {
            uint32_t creg = off - 0x1000;
            switch (creg) {
                case 0x00: vf->cdr.cmd = val; break;
                case 0x04: vf->cdr.lba = val; break;
                case 0x08: vf->cdr.count = val; break;
                case 0x0C: vf->cdr.dst = val; break;
                case 0x18: vf->cdr.sector_sz = val; break;
                case 0x14:
                    /* CTRL: bit0=start DMA */
                    if (val & 0x1) {
                        uint32_t lba   = vf->cdr.lba;
                        uint32_t count = vf->cdr.count;
                        uint32_t dst   = vf->cdr.dst;
                        uint32_t ssz   = vf->cdr.sector_sz ? vf->cdr.sector_sz : 2048;
                        uint32_t total = count * ssz;

                        if (vf->debug)
                            fprintf(stderr, "[CDROM] DMA: LBA=%u count=%u dst=0x%08X ssz=%u\n",
                                    lba, count, dst, ssz);

                        vf->cdr.status = 0x1; /* busy */

                        if (dst >= VFLASH_RAM_BASE &&
                            dst + total <= VFLASH_RAM_BASE + VFLASH_RAM_SIZE &&
                            count > 0 && count <= 256) {

                            uint8_t *dest = vf->ram + (dst - VFLASH_RAM_BASE);
                            int ok = 1;
                            for (uint32_t s = 0; s < count && ok; s++) {
                                uint8_t sector[2048];
                                ok = cdrom_read_sector(vf->cd, lba + s, sector);
                                if (ok) memcpy(dest + s * ssz, sector, ssz < 2048 ? ssz : 2048);
                            }

                            vf->cdr.status = ok ? 0x2 : 0x4; /* done or error */
                            ztimer_raise_irq(&vf->timer, IRQ_CDROM);

                            if (vf->debug)
                                fprintf(stderr, "[CDROM] DMA done: %s\n",
                                        ok ? "OK" : "ERROR");
                        } else {
                            fprintf(stderr, "[CDROM] DMA bad params: dst=0x%08X total=%u\n",
                                    dst, total);
                            vf->cdr.status = 0x4; /* error */
                        }
                    }
                    if (val & 0x2) { /* abort */
                        vf->cdr.status = 0;
                    }
                    break;
            }
            return;
        }

        /* Video DMA: 0x2000–0x2FFF */
        if (off >= 0x2000 && off < 0x3000) {
            uint32_t vreg = off - 0x2000;
            switch (vreg) {
                case 0x00:
                    vf->vid.fb_addr = val;
                    if (vf->debug)
                        fprintf(stderr, "[VID] FB addr = 0x%08X\n", val);
                    break;
                case 0x04:
                    vf->vid.ctrl = val;
                    /* bit1 = DMA trigger: copy game's ARGB framebuffer to our display */
                    if (val & 0x2) {
                        uint32_t src = vf->vid.fb_addr;
                        if (src >= VFLASH_RAM_BASE &&
                            src + VFLASH_SCREEN_W * VFLASH_SCREEN_H * 4
                                <= VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
                            memcpy(vf->framebuf,
                                   vf->ram + (src - VFLASH_RAM_BASE),
                                   VFLASH_SCREEN_W * VFLASH_SCREEN_H * 4);
                            vf->vid.fb_dirty = 1;
                            if (vf->debug)
                                fprintf(stderr, "[VID] DMA blit from 0x%08X\n", src);
                        }
                        vf->vid.ctrl &= ~0x2; /* auto-clear trigger */
                    }
                    break;
                case 0x0C:
                    vf->vid.jpeg_src = val;
                    break;
                case 0x10:
                    vf->vid.jpeg_size = val;
                    break;
                case 0x14:
                    vf->vid.jpeg_ctrl = val;
                    /* write 1 → decode JPEG from RAM into framebuffer */
                    if (val & 0x1) {
                        uint32_t src  = vf->vid.jpeg_src;
                        uint32_t size = vf->vid.jpeg_size;
                        if (size > 0 && size <= 512 * 1024 &&
                            src >= VFLASH_RAM_BASE &&
                            src + size <= VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
                            uint8_t *jpegdata = vf->ram + (src - VFLASH_RAM_BASE);
                            if (mjp_decode_frame(vf->video, jpegdata, size)) {
                                memcpy(vf->framebuf,
                                       mjp_get_framebuf(vf->video),
                                       VFLASH_SCREEN_W * VFLASH_SCREEN_H * 4);
                                vf->vid.fb_dirty = 1;
                                if (vf->debug)
                                    fprintf(stderr, "[VID] JPEG decode OK src=0x%08X size=%u\n",
                                            src, size);
                            } else {
                                fprintf(stderr, "[VID] JPEG decode FAILED src=0x%08X size=%u\n",
                                        src, size);
                            }
                        }
                        vf->vid.jpeg_ctrl  = 0;
                        vf->vid.jpeg_status = 0x2; /* done */
                        /* Raise IRQ if DMA IRQ enabled */
                        ztimer_raise_irq(&vf->timer, IRQ_DMA);
                    }
                    break;
            }
            return;
        }

        /* Audio DMA: 0x3000–0x3FFF */
        if (off >= 0x3000 && off < 0x4000) {
            uint32_t areg = off - 0x3000;
            switch (areg) {
                case 0x00: vf->aud.src  = val; break;
                case 0x04: vf->aud.size = val; break;
                case 0x08:
                    vf->aud.ctrl = val;
                    /* bit0 = play trigger */
                    if (val & 0x1) {
                        uint32_t src  = vf->aud.src;
                        uint32_t size = vf->aud.size;
                        if (size > 0 && size <= 2 * 1024 * 1024 &&
                            src >= VFLASH_RAM_BASE &&
                            src + size <= VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
                            uint8_t *pcm   = vf->ram + (src - VFLASH_RAM_BASE);
                            int      stereo = (val >> 2) & 1;
                            int      s16    = (val >> 3) & 1;
                            audio_queue_pcm(vf->audio, pcm, size, stereo, s16);
                            if (vf->debug)
                                fprintf(stderr, "[AUD] Queue PCM src=0x%08X size=%u s=%d 16=%d\n",
                                        src, size, stereo, s16);
                        }
                        ztimer_raise_irq(&vf->timer, IRQ_AUDIO);
                    }
                    break;
            }
            return;
        }

        /* UART stub: 0x5000–0x5FFF
         * Intercepts TX writes and prints to stderr as [UART] output.
         * This catches µMORE RTOS debug/error messages before the game starts.
         * Line-buffered: print when '\n' received or buffer full. */
        if (off >= 0x5000 && off < 0x6000) {
            if ((off - 0x5000) == 0x00) {
                /* UART_TX: accumulate into line buffer */
                static char uart_line[256];
                static int  uart_pos = 0;
                char c = (char)(val & 0xFF);
                if (c == '\n' || uart_pos >= 254) {
                    uart_line[uart_pos] = 0;
                    fprintf(stderr, "[UART] %s\n", uart_line);
                    uart_pos = 0;
                } else if (c >= 0x20) {
                    uart_line[uart_pos++] = c;
                }
            }
            /* All other UART registers: silently accept */
            return;
        }

        return;
    }
}


static void mem_write16(void *ctx, uint32_t addr, uint16_t val) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);
    if (addr < VFLASH_RAM_SIZE) {
        *(uint16_t*)(vf->ram + addr) = val;
        return;
    }
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        *(uint16_t*)(vf->ram + (addr - VFLASH_RAM_BASE)) = val;
        return;
    }
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE) {
        *(uint16_t*)(vf->sram + (addr - VFLASH_SRAM_BASE)) = val;
        return;
    }
    /* Route I/O: read-modify-write the 32-bit register */
    if (addr >= VFLASH_IO_BASE) {
        uint32_t aligned = addr & ~3u;
        uint32_t shift   = (addr & 2) * 8;
        uint32_t cur     = mem_read32(ctx, aligned);
        uint32_t updated = (cur & ~(0xFFFFu << shift)) | ((uint32_t)val << shift);
        mem_write32(ctx, aligned, updated);
    }
}

static void mem_write8(void *ctx, uint32_t addr, uint8_t val) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);
    if (addr < VFLASH_RAM_SIZE) {
        vf->ram[addr] = val;
        return;
    }
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        vf->ram[addr - VFLASH_RAM_BASE] = val;
        return;
    }
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE) {
        vf->sram[addr - VFLASH_SRAM_BASE] = val;
        return;
    }
    /* Route I/O: read-modify-write */
    if (addr >= VFLASH_IO_BASE) {
        uint32_t aligned = addr & ~3u;
        uint32_t shift   = (addr & 3) * 8;
        uint32_t cur     = mem_read32(ctx, aligned);
        uint32_t updated = (cur & ~(0xFFu << shift)) | ((uint32_t)val << shift);
        mem_write32(ctx, aligned, updated);
    }
}

/* ============================================================
 * HLE Boot — scan disc for ARM ELF/raw binary, load to RAM
 * ============================================================ */

/* Check if data looks like ARM code:
 * ARM reset vector at offset 0 should be a branch or MOV instruction */
static int looks_like_arm(const uint8_t *data, uint32_t size) {
    if (size < 32) return 0;
    uint32_t first = *(uint32_t*)data;

    /* ARM branch: 0xEA?????? or 0xEB?????? */
    if ((first >> 24) == 0xEA || (first >> 24) == 0xEB) return 1;

    /* ARM MOV PC,#imm or LDR PC,[PC,#off] */
    if ((first & 0xFFFF0000) == 0xE59F0000) return 1;
    if ((first & 0xFFFF0000) == 0xE3A00000) return 1;

    /* ARM NOP + branch at vector 4 (reset) */
    uint32_t reset = *(uint32_t*)(data + 4);
    if ((reset >> 24) == 0xEA) return 1;

    return 0;
}

/* Check for ELF header */
static int is_elf(const uint8_t *data, uint32_t size) {
    return size > 52 &&
           data[0] == 0x7F && data[1] == 'E' &&
           data[2] == 'L'  && data[3] == 'F';
}

/* Load ELF segments into RAM */
static uint32_t load_elf(VFlash *vf, const uint8_t *data, uint32_t size) {
    /* ELF32 header */
    uint32_t e_entry  = *(uint32_t*)(data + 24);
    uint16_t e_phoff  = *(uint16_t*)(data + 28);
    uint16_t e_phnum  = *(uint16_t*)(data + 44);
    uint16_t e_phsize = *(uint16_t*)(data + 42);

    printf("[HLE] ELF: entry=0x%08X phnum=%u\n", e_entry, e_phnum);

    for (int i = 0; i < e_phnum; i++) {
        const uint8_t *ph = data + e_phoff + i * e_phsize;
        uint32_t p_type   = *(uint32_t*)(ph + 0);
        uint32_t p_offset = *(uint32_t*)(ph + 4);
        uint32_t p_vaddr  = *(uint32_t*)(ph + 8);
        uint32_t p_filesz = *(uint32_t*)(ph + 16);
        uint32_t p_memsz  = *(uint32_t*)(ph + 20);

        if (p_type != 1) continue;  /* PT_LOAD */
        if (p_vaddr < VFLASH_RAM_BASE ||
            p_vaddr + p_memsz > VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
            fprintf(stderr, "[HLE] ELF segment 0x%08X out of RAM range\n", p_vaddr);
            continue;
        }

        uint8_t *dst = vf->ram + (p_vaddr - VFLASH_RAM_BASE);
        memset(dst, 0, p_memsz);
        if (p_filesz > 0 && p_offset + p_filesz <= size)
            memcpy(dst, data + p_offset, p_filesz);

        printf("[HLE] ELF load: 0x%08X size=0x%X\n", p_vaddr, p_filesz);
    }

    return e_entry;
}

/* Load raw binary at LOAD_ADDR */
/* V.Flash BOOT format: "BOOT" + size(4) + load_addr(4) + header_size(4) + code... */
static int is_vflash_boot(const uint8_t *data, uint32_t size) {
    return size >= 16 && memcmp(data, "BOOT", 4) == 0;
}

static uint32_t load_vflash_boot(VFlash *vf, const uint8_t *data, uint32_t size) {
    uint32_t file_size  = *(uint32_t*)(data + 4);
    uint32_t load_addr  = *(uint32_t*)(data + 8);
    uint32_t hdr_size   = *(uint32_t*)(data + 12);

    printf("[HLE] V.Flash BOOT format: load=0x%08X hdr=%u file_size=%u\n",
           load_addr, hdr_size, file_size);

    /* The entire BOOT.BIN (including the 16-byte header) is loaded at load_addr.
     * The code uses absolute addresses relative to load_addr+0, so the header
     * bytes (BOOT magic etc.) occupy the first 16 bytes at load_addr and the
     * LDR PC at offset 0x10 jumps past them to the real entry point. */
    uint32_t code_size = size;
    const uint8_t *code = data;

    /* Load code at the specified address */
    if (load_addr >= VFLASH_RAM_BASE &&
        load_addr + code_size <= VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        memcpy(vf->ram + (load_addr - VFLASH_RAM_BASE), code, code_size);
        printf("[HLE] Loaded %u bytes at 0x%08X\n", code_size, load_addr);
    } else if (load_addr >= VFLASH_SRAM_BASE &&
               load_addr + code_size <= VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE) {
        memcpy(vf->sram + (load_addr - VFLASH_SRAM_BASE), code, code_size);
        printf("[HLE] Loaded %u bytes at 0x%08X (SRAM)\n", code_size, load_addr);
    } else {
        fprintf(stderr, "[HLE] BOOT load address 0x%08X out of range\n", load_addr);
        /* Try loading at RAM base as fallback */
        if (code_size <= VFLASH_RAM_SIZE) {
            memcpy(vf->ram, code, code_size);
            load_addr = VFLASH_RAM_BASE;
            printf("[HLE] Fallback: loaded at 0x%08X\n", load_addr);
        }
    }

    /* Entry point is at load_addr + hdr_size (the LDR PC trampoline) */
    return load_addr + hdr_size;
}

static uint32_t load_raw(VFlash *vf, const uint8_t *data, uint32_t size) {
    uint32_t load = VFLASH_LOAD_ADDR;
    if (size > VFLASH_RAM_SIZE) size = VFLASH_RAM_SIZE;
    memcpy(vf->ram, data, size);
    printf("[HLE] Raw binary: %u bytes at 0x%08X\n", size, load);
    return load;
}

/* ============================================================
 * HLE Vector Table + UART stub
 *
 * The ARM exception vectors live at 0x00000000 (or high vecs 0xFFFF0000).
 * µMORE RTOS will overwrite these at runtime, but until then
 * a crash at any of these vectors = instant hang at address 0.
 *
 * Strategy: write LDR PC,[PC,#-4] trampolines pointing at small
 * ARM stubs in low RAM (HLE_STUB_BASE). Each stub logs the exception
 * and either returns (for SWI/IRQ/FIQ — OS will install real handlers)
 * or loops with debug print (UNDEF/ABORT — always fatal).
 *
 * Because the game is loaded at 0x10000000+ and the vector table
 * sits at 0x10000000 (same base as RAM), we write stub code directly
 * into the RAM array at offset 0.
 * ============================================================ */

/* ARM32 instruction helpers */
static inline uint32_t arm_ldr_pc_rel(uint32_t target_addr, uint32_t insn_addr) {
    /* LDR PC, [PC, #off]  — loads word at (insn_addr+8+off) into PC */
    int32_t off = (int32_t)(target_addr - (insn_addr + 8));
    return 0xE51FF000u | (uint32_t)(off < 0 ? (-off) | 0x800000 : off);
    /* Simpler: use E59FF000 (positive) or E51FF000 (negative) */
}

/* Write a 32-bit word at a RAM offset */
static inline void ram_w32(VFlash *vf, uint32_t addr, uint32_t val) {
    if (addr >= VFLASH_RAM_BASE && addr + 3 < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE)) = val;
}

static void install_vector_table(VFlash *vf) {
    /* Each vector entry = LDR PC, [PC, #-4]
     * which loads the word stored immediately before the LDR itself.
     * Layout at each vector slot:
     *   +0: LDR PC,[PC,#-4]  (0xE51FF004)
     *   +4: .word target_stub_addr
     *
     * But ARM vectors are only 4 bytes apart (0x00–0x1C, 8 entries).
     * Standard trick: place the target addresses in a table at 0x20–0x3C,
     * and use LDR PC,[PC,#0x18] to index into it.
     *
     *   0x00: LDR PC,[PC,#0x18]   → loads 0x20
     *   0x04: LDR PC,[PC,#0x18]   → loads 0x24
     *   ...
     *   0x1C: LDR PC,[PC,#0x18]   → loads 0x3C
     *   0x20: .word stub_reset
     *   0x24: .word stub_undef
     *   0x28: .word stub_swi
     *   0x2C: .word stub_pabt
     *   0x30: .word stub_dabt
     *   0x34: .word 0 (reserved)
     *   0x38: .word stub_irq
     *   0x3C: .word stub_fiq
     */

    /* LDR PC,[PC,#0x18] = 0xE59FF018 */
    uint32_t ldr = 0xE59FF018u;
    for (int i = 0; i < 8; i++)
        ram_w32(vf, VFLASH_RAM_BASE + i * 4, ldr);

    /* Stub addresses — stubs sit at HLE_STUB_BASE */
#define STUB(n) (HLE_STUB_BASE + (n) * 20)  /* 5 instructions each */
    ram_w32(vf, VFLASH_RAM_BASE + 0x20, STUB(0));  /* reset  */
    ram_w32(vf, VFLASH_RAM_BASE + 0x24, STUB(1));  /* undef  */
    ram_w32(vf, VFLASH_RAM_BASE + 0x28, STUB(2));  /* swi    */
    ram_w32(vf, VFLASH_RAM_BASE + 0x2C, STUB(3));  /* pabt   */
    ram_w32(vf, VFLASH_RAM_BASE + 0x30, STUB(4));  /* dabt   */
    ram_w32(vf, VFLASH_RAM_BASE + 0x34, 0);
    ram_w32(vf, VFLASH_RAM_BASE + 0x38, STUB(5));  /* irq    */
    ram_w32(vf, VFLASH_RAM_BASE + 0x3C, STUB(6));  /* fiq    */
#undef STUB

    /* Write the stubs. Each stub:
     *   PUSH {r0-r3, lr}         ; save caller state
     *   MOV  r0, #stub_id        ; argument: which exception
     *   BL   hle_exception_log   ; <-- we intercept this via SWI trap
     *   POP  {r0-r3, pc}^        ; restore and return (SPSR→CPSR)
     *
     * For fatal exceptions (undef/pabt/dabt): infinite loop instead of return.
     * For SWI/IRQ/FIQ/reset: return immediately — OS will install real handlers.
     *
     * Encoding:
     *   PUSH {r0-r3,lr}  = 0xE92D400F
     *   MOV  r0, #N      = 0xE3A00000 | N
     *   NOP              = 0xE320F000
     *   POP  {r0-r3,pc}^ = 0xE8FD400F
     *   B    .-8 (loop)  = 0xEAFFFFFD
     */
    static const struct {
        const char *name;
        int         fatal;
    } stubs[] = {
        { "RESET", 0 },
        { "UNDEF", 1 },
        { "SWI",   0 },
        { "PABT",  1 },
        { "DABT",  1 },
        { "IRQ",   0 },
        { "FIQ",   0 },
    };

    for (int i = 0; i < 7; i++) {
        uint32_t base = HLE_STUB_BASE + i * 20;
        ram_w32(vf, base +  0, 0xE92D400Fu);          /* PUSH {r0-r3,lr} */
        ram_w32(vf, base +  4, 0xE3A00000u | (uint32_t)i); /* MOV r0,#i */
        ram_w32(vf, base +  8, 0xE320F000u);          /* NOP */
        if (stubs[i].fatal) {
            ram_w32(vf, base + 12, 0xEAFFFFFDu);      /* B . (loop) */
            ram_w32(vf, base + 16, 0xEAFFFFFCu);      /* B . (loop) */
        } else {
            ram_w32(vf, base + 12, 0xE8FD400Fu);      /* POP {r0-r3,pc}^ */
            ram_w32(vf, base + 16, 0xE320F000u);      /* NOP (padding) */
        }
        (void)stubs[i].name;
    }

    printf("[HLE] Vector table installed at 0x%08X\n", VFLASH_RAM_BASE);
    printf("[HLE]   0x00 RESET → 0x%08X\n", HLE_STUB_BASE + 0 * 20);
    printf("[HLE]   0x04 UNDEF → 0x%08X (fatal loop)\n", HLE_STUB_BASE + 1 * 20);
    printf("[HLE]   0x08 SWI   → 0x%08X\n", HLE_STUB_BASE + 2 * 20);
    printf("[HLE]   0x18 IRQ   → 0x%08X\n", HLE_STUB_BASE + 5 * 20);
    printf("[HLE]   0x1C FIQ   → 0x%08X\n", HLE_STUB_BASE + 6 * 20);
}

static int vflash_hle_boot(VFlash *vf) {
    uint8_t sector[2048];

    /* Install HLE vector table BEFORE loading the binary.
     * If the binary is a raw image starting at RAM_BASE, load_raw() will
     * overwrite our stubs with the real OS vector table — that's correct.
     * If it's an ELF loaded above 0x10000040, our stubs survive and provide
     * safe fallback vectors until µMORE installs its own. */
    install_vector_table(vf);

    /* Read PVD to get root directory */
    if (!cdrom_read_sector(vf->cd, 16, sector)) {
        fprintf(stderr, "[HLE] Cannot read PVD\n");
        return 0;
    }

    printf("[HLE] Scanning disc for executable...\n");

    /* First pass: try well-known names in root and common subdirs */
    static const char *boot_paths[] = {
        /* Bare names (root) */
        "BOOT.BIN", "MAIN.BIN", "GAME.BIN", "APP.BIN",
        "BOOT.EXE", "MAIN.EXE", "START.BIN", "EXEC.BIN",
        "PROGRAM.BIN", "VMCORE.BIN", "KERNEL.BIN",
        /* With common subdir prefixes */
        "GAME/BOOT.BIN", "GAME/MAIN.BIN", "GAME/APP.BIN",
        "APP/BOOT.BIN",  "APP/MAIN.BIN",
        "DATA/BOOT.BIN", "SYS/BOOT.BIN",
        NULL
    };

    CDEntry entry;
    int found_entry = 0;

    for (int i = 0; boot_paths[i] && !found_entry; i++) {
        if (cdrom_find_file(vf->cd, boot_paths[i], &entry))
            found_entry = 1;
    }

    /* Second pass: recursive disc-wide search for any known name */
    if (!found_entry) {
        static const char *bare_names[] = {
            "BOOT.BIN", "MAIN.BIN", "GAME.BIN", "APP.BIN",
            "BOOT.EXE", "MAIN.EXE", NULL
        };
        printf("[HLE] Trying recursive disc search...\n");
        for (int i = 0; bare_names[i] && !found_entry; i++) {
            if (cdrom_find_file_any(vf->cd, bare_names[i], &entry))
                found_entry = 1;
        }
    }

    if (!found_entry) {
        fprintf(stderr, "[HLE] No bootable executable found on disc\n");
        return 0;
    }

    {
        printf("[HLE] Loading: %s (%u bytes)\n", entry.name, entry.size);

        if (entry.size == 0) {
            fprintf(stderr, "[HLE] Entry has zero size\n");
            return 0;
        }

        uint8_t *buf = malloc(entry.size);
        if (!buf) { fprintf(stderr, "[HLE] OOM\n"); return 0; }

        int bytes = cdrom_read_file(vf->cd, &entry, buf, 0, entry.size);
        if (bytes <= 0) { free(buf); fprintf(stderr, "[HLE] Read failed\n"); return 0; }

        uint32_t entry_point = 0;
        if (is_elf(buf, entry.size)) {
            printf("[HLE] ELF binary detected\n");
            entry_point = load_elf(vf, buf, entry.size);
        } else if (is_vflash_boot(buf, entry.size)) {
            printf("[HLE] V.Flash BOOT format detected\n");
            entry_point = load_vflash_boot(vf, buf, entry.size);

            /* HLE: Install ROM stub for REL processing function.
             * The BOOT init code jumps to a ROM function (typically at 0x1880)
             * via LDR PC,[R0] where R0 points to the REL descriptor.
             * The ROM function processes relocations and continues boot.
             * Since we don't have the ROM, we install a stub that:
             *  1. Reads the REL descriptor to find if there are relocations
             *  2. If table is empty (padded with 0x5A), skips processing
             *  3. Returns by jumping to the code after the REL check
             *
             * For now: REL table is empty in tested games, so the stub is a
             * simple NOP that patches the BOOT code to skip the REL jump.
             * We replace the LDR PC,[R0] at load_addr+0x180 with a NOP. */
            {
                uint32_t load_addr = *(uint32_t*)(buf + 8);
                if (load_addr >= VFLASH_RAM_BASE &&
                    load_addr + 0x184 < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
                    uint32_t off_180 = load_addr - VFLASH_RAM_BASE + 0x180;
                    uint32_t insn_at_180 = *(uint32_t*)(vf->ram + off_180);
                    /* Verify it's LDR PC,[R0] = 0xE590F000 */
                    if (insn_at_180 == 0xE590F000u) {
                        /* Install HLE ROM stub at 0x1880 in RAM mirror.
                         * The BOOT init code jumps here via LDR PC,[R0].
                         *
                         * ROM REL function would: process relocations (empty
                         * in tested games), enable IRQs, then return to caller
                         * (0x10C00184). The caller writes "boot complete" to a
                         * system register and enters WFI loop. Timer IRQ fires
                         * and the OS ISR takes over.
                         *
                         * Our stub:
                         *   MRS  R0, CPSR           ; read CPSR
                         *   BIC  R0, R0, #0xC0      ; clear I+F bits (enable IRQs)
                         *   MSR  CPSR_c, R0         ; write back
                         *   LDR  PC, [PC, #-4]      ; jump to return address
                         *   .word <return_addr>      ; = load_addr + 0x184
                         */
                        /* ROM REL function normally processes relocations
                         * and jumps to OS entry. For now, return to caller
                         * (0x184 = error path with write to 0x900A0008).
                         * We also map 0x900A0008 to a harmless I/O register
                         * so the write succeeds, and NOP the infinite loop. */
                        uint32_t base = load_addr - VFLASH_RAM_BASE;

                        /* Skip the entire REL block by patching init2's
                         * return address in the data pool. Init2 returns
                         * to [load_addr+0xF8] which is normally 0x150
                         * (the BL disable_caches + REL check). We redirect
                         * it to the µMORE RTOS init function instead.
                         *
                         * The RTOS init at load_addr+0x53AA0 sets up ARM
                         * mode stacks and starts the scheduler. */
                        uint32_t os_init = load_addr + 0x53AA0;
                        *(uint32_t*)(vf->ram + base + 0xF8) = os_init;
                        printf("[HLE] Patched init2 return → 0x%08X (µMORE RTOS init)\n",
                               os_init);

                        /* ROM initializes [load_addr+0xC004] to non-zero
                         * (task count). If zero, init3 skips task setup
                         * entirely. Set to 1 for minimal task setup. */
                        *(uint32_t*)(vf->ram + base + 0xC004) = 1;
                        printf("[HLE] Set task count [0x%08X] = 1\n",
                               load_addr + 0xC004);

                        /* µMORE init at 0x53AA0 does PUSH{R0}/POP{R0,R1}.
                         * POP reads R1 from [SP_init] = [load_addr+0x425C].
                         * R1 then gets stored to [0x10B602E0] (task PC).
                         * ROM pushes the game entry PC there before calling
                         * µMORE init. We write it to the stack slot directly.
                         *
                         * Similarly, R0 gets stored to [0x10BCA4E0] (task CPSR).
                         * R0 = saved R0 from PUSH{R0} which is whatever R0
                         * was when µMORE init was called. ROM sets R0 to the
                         * initial CPSR. Our patched return from init2 doesn't
                         * set R0, so we also write CPSR to the stack. */
                        uint32_t sp_init_off = 0xC0425C; /* load_addr+0x425C - RAM_BASE */
                        /* [SP_init+0] = R0 slot (PUSH {R0}) = initial CPSR */
                        /* [SP_init+4] would be R1 slot but POP reads below SP */
                        /* Actually: PUSH{R0} decrements SP by 4, so:
                         * After PUSH: SP = 0x10C04258, [0x10C04258] = R0
                         * POP{R0,R1}: R0 = [0x10C04258], R1 = [0x10C0425C]
                         * So R1 comes from ORIGINAL SP = load_addr+0x425C */
                        *(uint32_t*)(vf->ram + sp_init_off) = load_addr + 0xC4A4;
                        printf("[HLE] Set stack[SP_init] = 0x%08X (game main for R1→task PC)\n",
                               load_addr + 0xC4A4);

                        /* Pre-write initial CPSR at [0x10BCA4E0] and NOP ALL
                         * STR R0,[R2] instructions in µMORE init that would
                         * overwrite it with a bad value. There are 6 such STRs
                         * in the SVC/UND/ABT mode setup blocks. */
                        *(uint32_t*)(vf->ram + 0xBCA4E0) = 0x000000D3;
                        static const uint32_t cpsr_str_offsets[] = {
                            0x53AC0, 0x53B0C, 0x53BA8, 0x53BF4, 0x53C54, 0x53CA0
                        };
                        for (int i = 0; i < 6; i++)
                            *(uint32_t*)(vf->ram + base + cpsr_str_offsets[i]) = 0xE1A00000u;
                        printf("[HLE] Set task CPSR=0xD3, NOP'd %d CPSR writes\n", 6);

                        /* Fill task #1 in the task table.
                         * Table at 0x10B0DF00, 16 bytes per entry.
                         * Entry format: +0=?, +4=?, +8=callback, +C=active.
                         * Scanner ORs callback into R4 for all active tasks,
                         * then uses R4 as interrupt mask or dispatch value.
                         *
                         * Task #1 at 0x10B0DF10: */
                        /* Task table at 0x10B0DF00, 16 bytes per entry.
                         * Entry format:
                         *   +0: IRQ register address (for callback==-1 path)
                         *       OR IRQ bitmask (for callback!=-1 path)
                         *   +4: ???
                         *   +8: callback (-1 = use entry[+0] as IRQ addr)
                         *   +C: active flag (1)
                         *
                         * After scan, code writes:
                         *   [R0] = 0xDC000008  (R0 = task[+0] for cb==-1)
                         *   [0xDC000108] = R4  (R4 = OR'd task[+0] for cb!=-1)
                         *
                         * Set task #1 as IRQ controller task:
                         *   entry[+0] = 0xDC000004 (IRQ enable register addr)
                         *   entry[+8] = -1 (use addr path)
                         *   entry[+C] = 1 (active) */
                        uint32_t task1 = 0xB0DF10;
                        *(uint32_t*)(vf->ram + task1 + 0x00) = 0xDC000004;
                        *(uint32_t*)(vf->ram + task1 + 0x04) = 0;
                        *(uint32_t*)(vf->ram + task1 + 0x08) = 0xFFFFFFFF;
                        *(uint32_t*)(vf->ram + task1 + 0x0C) = 1;
                        printf("[HLE] Set task #1: IRQ addr=0xDC000004\n");
                    }
                }
            }
        } else if (looks_like_arm(buf, entry.size)) {
            printf("[HLE] Raw ARM binary detected\n");
            entry_point = load_raw(vf, buf, entry.size);
        } else {
            printf("[HLE] Unknown format, trying as raw ARM\n");
            entry_point = load_raw(vf, buf, entry.size);
        }

        free(buf);

        /* Setup CPU */
        arm9_reset(&vf->cpu);
        vf->cpu.r[15] = entry_point;
        vf->cpu.r[13] = VFLASH_STACK_TOP;
        vf->cpu.cpsr  = 0x0000001F;  /* SYS mode, ARM (T=0), IRQ+FIQ enabled */

        /* Set stack pointers for all ARM modes.
         * µMORE RTOS expects these to be pre-initialized by boot ROM.
         * Place each mode's stack 4KB apart below STACK_TOP. */
        vf->cpu.r13_svc = VFLASH_STACK_TOP - 0x1000;
        vf->cpu.r13_irq = VFLASH_STACK_TOP - 0x2000;
        vf->cpu.r13_fiq = VFLASH_STACK_TOP - 0x3000;
        vf->cpu.r13_abt = VFLASH_STACK_TOP - 0x4000;
        vf->cpu.r13_und = VFLASH_STACK_TOP - 0x5000;
        /* Initialize banked LR and SPSR for all modes.
         * µMORE RTOS init switches between modes via MSR CPSR_c and
         * returns via MOVS PC,LR which restores SPSR→CPSR. The ROM
         * normally sets these up. We initialize LR to entry_point
         * (safe default) and SPSR to SVC mode (the primary mode). */
        vf->cpu.r14_svc = entry_point;
        vf->cpu.r14_irq = entry_point;
        vf->cpu.r14_fiq = entry_point;
        vf->cpu.r14_abt = entry_point;
        vf->cpu.r14_und = entry_point;
        vf->cpu.spsr_svc = 0x000000D3;  /* SVC mode, IRQ+FIQ disabled */
        vf->cpu.spsr_irq = 0x000000D3;
        vf->cpu.spsr_fiq = 0x000000D3;
        vf->cpu.spsr_abt = 0x000000D3;
        vf->cpu.spsr_und = 0x000000D3;

        printf("[HLE] Boot: PC=0x%08X SP=0x%08X\n", entry_point, VFLASH_STACK_TOP);

        /* Disassemble first 8 instructions */
        if (entry_point >= VFLASH_RAM_BASE &&
            entry_point < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
            printf("[HLE] First instructions:\n");
            uint32_t pc = entry_point;
            for (int j = 0; j < 8; j++) {
                uint32_t insn = *(uint32_t*)(vf->ram + (pc - VFLASH_RAM_BASE));
                char dis[64];
                arm_disasm(pc, insn, dis, sizeof(dis));
                printf("  0x%08X: %08X  %s\n", pc, insn, dis);
                pc += 4;
            }
        }

        return 1;
    }

    fprintf(stderr, "[HLE] No bootable executable found on disc\n");
    return 0;
}

/* ============================================================
 * System create / destroy
 * ============================================================ */

/* Detect whether path is a raw ARM binary (not an ISO image).
 * Heuristic: file < 4MB AND starts with ARM/Thumb code OR ends in .bin/.elf */
static int is_raw_binary(const char *path) {
    /* Extension check */
    const char *dot = strrchr(path, '.');
    if (dot) {
        /* .cue and .iso are always disc images */
        if (strcasecmp(dot, ".cue") == 0) return 0;
        if (strcasecmp(dot, ".iso") == 0) return 0;
        if (strcasecmp(dot, ".elf") == 0) return 1;
        if (strcasecmp(dot, ".axf") == 0) return 1;
    }
    /* .bin: check file size - CD images are large (>4MB), raw ARM binaries are small */
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);

    /* Check for ISO PVD at sector 16 (2048-byte sectors) */
    uint8_t sig[5] = {0};
    if (fseek(f, 16 * 2048 + 1, SEEK_SET) == 0)
        (void)fread(sig, 1, 5, f);
    if (memcmp(sig, "CD001", 5) == 0) { fclose(f); return 0; }

    /* Check for ISO PVD in raw BIN at sector 16 (2352-byte sectors, +16 data offset) */
    memset(sig, 0, sizeof(sig));
    if (fseek(f, 16 * 2352 + 16 + 1, SEEK_SET) == 0)
        (void)fread(sig, 1, 5, f);
    if (memcmp(sig, "CD001", 5) == 0) { fclose(f); return 0; }

    fclose(f);
    /* Large files without ISO signature: assume disc image anyway if >4MB */
    if (sz > 4 * 1024 * 1024) return 0;
    return 1;
}

static int vflash_load_rawbin(VFlash *vf, const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return 0; }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > VFLASH_RAM_SIZE) {
        fprintf(stderr, "[HLE] Raw binary too large: %ld bytes\n", sz);
        fclose(f); return 0;
    }
    uint8_t *buf = malloc((size_t)sz);
    if (!buf) { fclose(f); return 0; }
    if (fread(buf, 1, (size_t)sz, f) != (size_t)sz) {
        fprintf(stderr, "[HLE] Read error: %s\n", path);
        free(buf); fclose(f); return 0;
    }
    fclose(f);

    install_vector_table(vf);

    uint32_t entry_point;
    if (is_elf(buf, (uint32_t)sz)) {
        printf("[HLE] ELF binary: %s\n", path);
        entry_point = load_elf(vf, buf, (uint32_t)sz);
    } else {
        printf("[HLE] Raw ARM binary: %s (%ld bytes)\n", path, sz);
        entry_point = load_raw(vf, buf, (uint32_t)sz);
    }
    free(buf);

    arm9_reset(&vf->cpu);
    vf->cpu.r[15] = entry_point;
    vf->cpu.r[13] = VFLASH_STACK_TOP;
    vf->cpu.cpsr  = 0x0000001F;  /* SYS mode ARM */

    printf("[HLE] Boot: PC=0x%08X SP=0x%08X\n", entry_point, VFLASH_STACK_TOP);
    if (entry_point >= VFLASH_RAM_BASE &&
        entry_point <  VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        printf("[HLE] First instructions:\n");
        uint32_t pc = entry_point;
        for (int j = 0; j < 8; j++) {
            uint32_t insn = *(uint32_t*)(vf->ram + (pc - VFLASH_RAM_BASE));
            char dis[64];
            arm_disasm(pc, insn, dis, sizeof(dis));
            printf("  0x%08X: %08X  %s\n", pc, insn, dis);
            pc += 4;
        }
    }
    return 1;
}

VFlash* vflash_create(const char *disc_path) {
    VFlash *vf = calloc(1, sizeof(VFlash));

    vf->ram   = calloc(1, VFLASH_RAM_SIZE);
    vf->sram  = calloc(1, VFLASH_SRAM_SIZE);
    vf->cd    = cdrom_create();
    vf->video = mjp_create(VFLASH_SCREEN_W, VFLASH_SCREEN_H);
    vf->audio = audio_create();

    cp15_reset(&vf->cpu.cp15);
    ztimer_reset(&vf->timer);

    vf->cpu.mem_ctx      = vf;
    vf->cpu.mem_read32   = mem_read32;
    vf->cpu.mem_read16   = mem_read16;
    vf->cpu.mem_read8    = mem_read8;
    vf->cpu.mem_write32  = mem_write32;
    vf->cpu.mem_write16  = mem_write16;
    vf->cpu.mem_write8   = mem_write8;

    /* Try to load boot ROM (70004.bin) — enables real boot instead of HLE */
    {
        const char *rom_paths[] = {
            "70004.bin",
            "../vflash-roms/70004.bin",
            "/home/wizzard/share/GitHub/vflash-roms/70004.bin",
            NULL
        };
        FILE *rom_fp = NULL;
        for (int i = 0; rom_paths[i]; i++) {
            rom_fp = fopen(rom_paths[i], "rb");
            if (rom_fp) break;
        }
        if (rom_fp) {
            fseek(rom_fp, 0, SEEK_END);
            long rom_sz = ftell(rom_fp);
            fseek(rom_fp, 0, SEEK_SET);
            if (rom_sz > 0 && rom_sz <= 0x200000) {
                vf->rom = malloc((size_t)rom_sz);
                vf->rom_size = (uint32_t)rom_sz;
                fread(vf->rom, 1, (size_t)rom_sz, rom_fp);
                /* Map full ROM at physical address 0 (boot mirror).
                 * ROM code copies itself to RAM and expects data at 0x10000+.
                 * Full 2MB also accessible at 0xB8000000 via I/O handler. */
                uint32_t boot_size = (uint32_t)rom_sz;
                memcpy(vf->ram, vf->rom, boot_size);
                printf("[ROM] Loaded boot ROM: %ld bytes (512KB at 0x0, full at 0xB8000000)\n", rom_sz);
                vf->has_rom = 1;
            }
            fclose(rom_fp);
        }
    }

    if (disc_path) {
        if (is_raw_binary(disc_path)) {
            /* Raw ARM binary — load directly, skip ISO layer */
            if (!vflash_load_rawbin(vf, disc_path)) {
                fprintf(stderr, "[VFlash] Failed to load binary: %s\n", disc_path);
                vflash_destroy(vf);
                return NULL;
            }
        } else {
            /* ISO 9660 disc image */
            if (!cdrom_open(vf->cd, disc_path)) {
                fprintf(stderr, "[VFlash] Failed to open disc: %s\n", disc_path);
                vflash_destroy(vf);
                return NULL;
            }
            if (vf->has_rom) {
                /* Real boot ROM loaded — let it boot naturally.
                 * ROM will read BOOT.BIN from CD, init µMORE, start game. */
                arm9_reset(&vf->cpu);
                vf->cpu.r[15] = 0x00000000;  /* ARM boots from address 0 */
                printf("[ROM] Booting from ROM at 0x00000000\n");
            } else if (!vflash_hle_boot(vf)) {
                fprintf(stderr, "[VFlash] HLE boot failed — starting at 0x%08X\n",
                        VFLASH_LOAD_ADDR);
                arm9_reset(&vf->cpu);
                vf->cpu.r[15] = VFLASH_LOAD_ADDR;
                vf->cpu.r[13] = VFLASH_STACK_TOP;
            }
        }
    } else {
        arm9_reset(&vf->cpu);
    }

    printf("[VFlash] System ready\n");
    return vf;
}

void vflash_destroy(VFlash *vf) {
    if (!vf) return;
    cdrom_destroy(vf->cd);
    mjp_destroy(vf->video);
    audio_destroy(vf->audio);
    free(vf->ram);
    free(vf->sram);
    free(vf->rom);
    free(vf);
}

void vflash_init_audio(VFlash *vf) {
    audio_init_sdl(vf->audio);
}

void vflash_set_debug(VFlash *vf, int on) {
    vf->debug = on;
}

/* ============================================================
 * Per-frame execution
 * ============================================================ */

void vflash_run_frame(VFlash *vf) {
    /* ARM926EJ-S @ 150MHz, ~60fps → 2,500,000 cycles/frame.
     * We measure actual cycles consumed by arm9_run() via cpu.cycles
     * so ztimer_tick always gets the real count, not just the request. */
    const int SLICE = 10000;
    const int TOTAL = 2500000;

    vf->vid.fb_dirty = 0;

    int done = 0;
    while (done < TOTAL) {
        int slice = (TOTAL - done < SLICE) ? (TOTAL - done) : SLICE;

        uint64_t cyc_before = vf->cpu.cycles;
        arm9_run(&vf->cpu, slice);
        uint32_t actual = (uint32_t)(vf->cpu.cycles - cyc_before);

        ztimer_tick(&vf->timer, actual);
        done += (int)actual;   /* advance by what CPU actually consumed */

        /* Force-clear I bit so IRQ can be delivered.
         * Only after init completes (~5M cycles = ~2 frames).
         * µMORE RTOS never enables IRQ itself (ROM normally does). */
        /* Force IRQ delivery after init settles (~500K cycles = 200ms).
         * Also ensure IRQ mode SP is valid before first IRQ fires.
         * ROM BL #1 normally sets IRQ SP but self-modified code skips it. */
        if (vf->has_rom && vf->cpu.cycles > 1250000 && (vf->cpu.cpsr & 0x80)) {
            /* Set IRQ SP if not yet set (check for suspicious values) */
            if (vf->cpu.r13_irq == 0 || vf->cpu.r13_irq > 0xF0000000u)
                vf->cpu.r13_irq = 0x10800000;  /* Safe IRQ stack in high RAM */
            vf->cpu.cpsr &= ~0x80u;
        }

        if (ztimer_fiq_pending(&vf->timer))
            arm9_fiq(&vf->cpu);
        else if (ztimer_irq_pending(&vf->timer))
            arm9_irq(&vf->cpu);
    }

    ztimer_raise_irq(&vf->timer, IRQ_TIMER0);   /* vsync */

    if (!vf->vid.fb_dirty)
        memset(vf->framebuf, 0,
               VFLASH_SCREEN_W * VFLASH_SCREEN_H * sizeof(uint32_t));

    if (vf->debug && (vf->frame_count % 300) == 0) {
        fprintf(stderr, "[Frame %lu] fb_dirty=%d PC=0x%08X cycles=%llu\n",
                (unsigned long)vf->frame_count, vf->vid.fb_dirty,
                vf->cpu.r[15], (unsigned long long)vf->cpu.cycles);
        arm9_dump_regs(vf->cpu.r, vf->cpu.cpsr);
    }

    vf->frame_count++;
}

void     vflash_set_input(VFlash *vf, uint32_t buttons) { vf->input = buttons; }
uint32_t* vflash_get_framebuffer(VFlash *vf) { return vf->framebuf; }

/* ============================================================
 * Debugger API
 * ============================================================ */

uint32_t vflash_get_pc(VFlash *vf)         { return vf->cpu.r[15]; }
uint32_t vflash_get_reg(VFlash *vf, int r) { return (r>=0&&r<16) ? vf->cpu.r[r] : 0; }
void     vflash_set_reg(VFlash *vf, int r, uint32_t val) {
    if (r >= 0 && r < 16) vf->cpu.r[r] = val;
}
uint32_t vflash_get_cpsr(VFlash *vf)       { return vf->cpu.cpsr; }
int      vflash_is_thumb(VFlash *vf)       { return (vf->cpu.cpsr >> 5) & 1; }

uint32_t vflash_read32(VFlash *vf, uint32_t addr) {
    return vf->cpu.mem_read32(vf, addr);
}
uint8_t vflash_read8(VFlash *vf, uint32_t addr) {
    return vf->cpu.mem_read8(vf, addr);
}
void vflash_write32(VFlash *vf, uint32_t addr, uint32_t val) {
    vf->cpu.mem_write32(vf, addr, val);
}

int vflash_step(VFlash *vf) {
    uint64_t before = vf->cpu.cycles;
    arm9_step(&vf->cpu);
    uint32_t cyc = (uint32_t)(vf->cpu.cycles - before);

    ztimer_tick(&vf->timer, cyc);
    if (ztimer_fiq_pending(&vf->timer)) arm9_fiq(&vf->cpu);
    else if (ztimer_irq_pending(&vf->timer)) arm9_irq(&vf->cpu);

    /* Check breakpoints against new PC */
    uint32_t pc = vf->cpu.r[15];
    vf->bp_hit = 0;
    for (int i = 0; i < 16; i++) {
        if (vf->bp[i] && vf->bp[i] == pc) {
            vf->bp_hit = 1;
            break;
        }
    }
    return (int)cyc;
}

void vflash_bp_set(VFlash *vf, uint32_t addr) {
    /* Find empty slot or replace existing */
    for (int i = 0; i < 16; i++) {
        if (vf->bp[i] == addr) return;  /* already set */
        if (vf->bp[i] == 0) { vf->bp[i] = addr; return; }
    }
    fprintf(stderr, "[DBG] No free breakpoint slots\n");
}

void vflash_bp_clear(VFlash *vf, uint32_t addr) {
    for (int i = 0; i < 16; i++)
        if (vf->bp[i] == addr) vf->bp[i] = 0;
}

void vflash_bp_clear_all(VFlash *vf) {
    memset(vf->bp, 0, sizeof(vf->bp));
}

int vflash_bp_hit(VFlash *vf) { return vf->bp_hit; }

uint32_t vflash_bp_list(VFlash *vf, uint32_t *out, int maxn) {
    int n = 0;
    for (int i = 0; i < 16 && n < maxn; i++)
        if (vf->bp[i]) out[n++] = vf->bp[i];
    return (uint32_t)n;
}

int vflash_run_until_bp(VFlash *vf, int max_cycles) {
    int done = 0;
    vf->bp_hit = 0;
    while (done < max_cycles) {
        done += vflash_step(vf);
        if (vf->bp_hit) return 1;
    }
    return 0;
}

