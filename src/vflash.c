#include "vflash.h"
#include "arm9.h"
#include "jit.h"
#include "cdrom.h"
#include "mjp.h"
#include "audio.h"
#include "ztimer.h"
#include "disasm.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/* Forward declarations */
static int hle_service_intercept(void *ctx, uint32_t addr);

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

/* ATAPI CD-ROM controller state (at 0xAA000000)
 * Standard ATA task file + PACKET command state machine.
 * The ZEVIO 1020 SoC wraps this with additional registers
 * for DMA and interrupt control at offsets 0x08+ */
#define ATAPI_STATE_IDLE     0
#define ATAPI_STATE_CMD      1  /* waiting for 12-byte PACKET CDB */
#define ATAPI_STATE_DATA_IN  2  /* transferring data to host */
#define ATAPI_STATE_DATA_OUT 3  /* transferring data from host */
#define ATAPI_STATE_COMPLETE 4  /* command complete */

/* ATA status bits */
#define ATA_SR_BSY   0x80
#define ATA_SR_DRDY  0x40
#define ATA_SR_DRQ   0x08
#define ATA_SR_ERR   0x01
/* ATA interrupt reason (sector count register in ATAPI) */
#define ATAPI_IR_COD  0x01  /* 1=command, 0=data */
#define ATAPI_IR_IO   0x02  /* 1=to host, 0=from host */

typedef struct {
    /* Standard ATA task file registers */
    uint8_t  error;       /* reg 0x01 read */
    uint8_t  features;    /* reg 0x01 write */
    uint8_t  sect_count;  /* reg 0x02 (ATAPI: interrupt reason) */
    uint8_t  lba_low;     /* reg 0x03 */
    uint8_t  byte_count_lo; /* reg 0x04 */
    uint8_t  byte_count_hi; /* reg 0x05 */
    uint8_t  drive_head;  /* reg 0x06 */
    uint8_t  status;      /* reg 0x07 read */
    uint8_t  dev_ctrl;    /* reg 0x0E write */

    /* PACKET command state */
    int      state;
    uint8_t  packet[12];  /* 12-byte CDB */
    int      packet_pos;  /* bytes received of CDB so far */

    /* Data transfer buffer */
    uint8_t *data_buf;
    uint32_t data_size;   /* total bytes to transfer */
    uint32_t data_pos;    /* current position in buffer */

    /* Sense data for REQUEST SENSE */
    uint8_t  sense_key;
    uint8_t  asc;         /* additional sense code */
    uint8_t  ascq;        /* additional sense code qualifier */

    /* ZEVIO IDE wrapper registers (0x08-0xFF) */
    uint32_t zevio_regs[64];

    /* Reboot counter for warm boot handling */
    int      reboot_count;

    /* Logging */
    int      log_count;
} ATAPIState;

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

/* PL111 LCD Controller state */
typedef struct {
    uint32_t timing[4];   /* 0x00-0x0C: timing registers */
    uint32_t upbase;      /* 0x10: upper panel framebuffer PA */
    uint32_t lpbase;      /* 0x14: lower panel framebuffer PA */
    uint32_t control;     /* 0x18: control register */
    uint32_t imsc;        /* 0x1C: interrupt mask */
    uint32_t ris;         /* 0x20: raw interrupt status */
    uint16_t palette[256]; /* 0x200-0x3FF: 256-entry XBGR1555 palette */
    int      pal_written;  /* number of palette entries written */
} LCDRegs;

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
    uint32_t  flash_remap; /* Flash controller: remap base written to reg 0x800 */
    uint8_t   flash_buf[0x2000]; /* Flash controller write buffer (captures writes to 0xB8000800+) */
    int       flash_buf_dirty;   /* 1 if flash_buf has been written to */
    int       flash_preloaded;   /* 1 after ROM preload done (don't overwrite flash_buf[0]) */
    int       rom_remapped;      /* 1 after flash remap: PA 0 reads from low_ram, not ROM */
    uint8_t   low_ram[0x2000];   /* Separate buffer for PA 0x00000000-0x00001FFF (vector table etc.)
                                  * On real HW, PA 0 = ROM; after remap = flash window → SDRAM.
                                  * Must NOT alias with SDRAM at 0x10000000 (vf->ram). */
    uint32_t  dma_param_a, dma_param_b, dma_param_c;
    uint32_t  misc_regs[64];   /* Misc system control at 0x900A0000 (read/write) */
    uint32_t  rtc_regs[64];    /* RTC/scratch at 0x90090000 (µMORE uses 0x8C for FIQ flag) */
    uint32_t  pmu_regs[16];    /* PMU at 0x900B0000 */
    uint64_t  frame_count;

    VideoRegs vid;
    AudioRegs aud;
    CDRomRegs cdr;
    ATAPIState atapi;
    LCDRegs lcd;

    /* MJP video player state */
    struct {
        uint8_t  *data;        /* full MJP file data */
        uint32_t  data_size;
        uint8_t  *hdr;         /* I-frame JPEG header (SOI+tables, before SOS) */
        uint32_t  hdr_len;
        uint32_t  chunk_off;   /* current chunk offset in data */
        uint32_t  vid_w, vid_h; /* video dimensions from MIAV header */
        int       playing;
        int       frame_rate;  /* frames to skip between decodes (throttle) */
        int       frame_skip;
    } mjp_player;

    /* PTX gallery state */
    CDEntry  *ptx_list;    /* array of PTX entries on disc */
    int       ptx_count;
    int       ptx_index;   /* current displayed index */
    int       ptx_loaded;  /* 1 after gallery scanned */

    /* WAV sound list */
    CDEntry  *wav_list;
    int       wav_count;
    int       wav_auto_idx;  /* auto-play index */

    /* MJP video list */
    CDEntry  *mjp_list;
    int       mjp_count;
    int       mjp_index;

    /* Debugger state */
    uint32_t bp[16];   /* breakpoint addresses (0 = unused) */
    int      bp_hit;   /* set by vflash_step() when BP hit */

    int      boot_phase; /* Boot flow phase tracking */
    int      render_budget; /* Instructions remaining in render pipeline (0=unlimited) */
    int      restore_vtable_pending; /* Restore vtable after render_init */
    JitContext *jit;                 /* JIT compiler context */
    uint8_t *rtos_backup;           /* Backup of RTOS code area for restore */
    uint32_t native_alloc_lr; /* Track native alloc return address for logging */
    uint32_t flash_last_write; /* Last value written to NOR flash (for status polling) */
    /* Saved game task info from BOOT TCB scan (before service entry may overwrite) */
    uint32_t saved_game_entry;
    uint32_t saved_game_stack;
    uint32_t saved_game_stack_sz;
    uint32_t fiq_saved_r8_12[5]; /* r[8]-r[12] saved before FIQ mode switch */
    uint32_t dc_vblank_cb;       /* Display controller VBlank callback (from 0xB80007C0) */
    uint32_t dc_irq_handler;     /* Display controller IRQ handler (from 0xB8000784) */
    uint32_t dc_regs[64];        /* Display controller regs 0xB8000100-0xB80001FF */

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
    if (pa < 0x00200000u) {
        if (vf->has_rom && pa + 3 < vf->rom_size)
            return *(uint32_t*)(vf->rom + pa);
        return 0;
    }
    if (pa >= VFLASH_RAM_BASE && pa + 3 < VFLASH_RAM_BASE + VFLASH_RAM_SIZE)
        return *(uint32_t*)(vf->ram + (pa - VFLASH_RAM_BASE));
    if (pa >= VFLASH_SRAM_BASE && pa + 3 < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return *(uint32_t*)(vf->sram + (pa - VFLASH_SRAM_BASE));
    return 0;
}

/* ---- ATAPI CD-ROM controller ---- */

static void atapi_init(ATAPIState *a) {
    memset(a, 0, sizeof(*a));
    a->status = ATA_SR_DRDY;
    a->sect_count = ATAPI_IR_COD | ATAPI_IR_IO; /* ready for command */
    a->sense_key = 0x00; /* NO SENSE — disc ready, no errors */
    a->asc = 0x00;       /* no additional sense */
}

static void atapi_set_sense(ATAPIState *a, uint8_t key, uint8_t asc, uint8_t ascq) {
    a->sense_key = key;
    a->asc = asc;
    a->ascq = ascq;
}

static void atapi_complete_ok(ATAPIState *a) {
    a->state = ATAPI_STATE_IDLE;
    a->status = ATA_SR_DRDY;
    a->sect_count = ATAPI_IR_COD | ATAPI_IR_IO; /* command complete, to host */
    a->error = 0;
    atapi_set_sense(a, 0, 0, 0); /* no error */
}

static void atapi_complete_err(ATAPIState *a, uint8_t key, uint8_t asc, uint8_t ascq) {
    a->state = ATAPI_STATE_IDLE;
    a->status = ATA_SR_DRDY | ATA_SR_ERR;
    a->sect_count = ATAPI_IR_COD | ATAPI_IR_IO;
    a->error = (key << 4); /* sense key in error register bits 7:4 */
    atapi_set_sense(a, key, asc, ascq);
}

static void atapi_start_data_in(ATAPIState *a, uint8_t *buf, uint32_t size) {
    a->data_buf = buf;
    a->data_size = size;
    a->data_pos = 0;
    a->state = ATAPI_STATE_DATA_IN;
    a->status = ATA_SR_DRDY | ATA_SR_DRQ;
    a->sect_count = ATAPI_IR_IO; /* data to host, not command */
    a->byte_count_lo = size & 0xFF;
    a->byte_count_hi = (size >> 8) & 0xFF;
}

/* Process a 12-byte ATAPI PACKET command */
static void atapi_exec_packet(VFlash *vf) {
    ATAPIState *a = &vf->atapi;
    uint8_t *cdb = a->packet;
    uint8_t opcode = cdb[0];

    if (a->log_count < 200)
        printf("[ATAPI] PACKET cmd: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
               cdb[0],cdb[1],cdb[2],cdb[3],cdb[4],cdb[5],
               cdb[6],cdb[7],cdb[8],cdb[9],cdb[10],cdb[11]);

    /* Free previous data buffer */
    if (a->data_buf) { free(a->data_buf); a->data_buf = NULL; }
    a->data_size = 0;
    a->data_pos = 0;

    switch (opcode) {
        case 0x00: /* TEST UNIT READY */
            if (vf->cd && vf->cd->is_open)
                atapi_complete_ok(a);
            else
                atapi_complete_err(a, 0x02, 0x3A, 0x00); /* NOT READY, medium not present */
            break;

        case 0x03: { /* REQUEST SENSE */
            uint8_t alloc = cdb[4] ? cdb[4] : 18;
            uint8_t *buf = calloc(1, alloc);
            buf[0] = 0x70;           /* current errors, fixed format */
            buf[2] = a->sense_key;
            buf[7] = 10;             /* additional sense length */
            buf[12] = a->asc;
            buf[13] = a->ascq;
            atapi_start_data_in(a, buf, alloc);
            break;
        }

        case 0x12: { /* INQUIRY */
            uint16_t alloc = (cdb[3] << 8) | cdb[4];
            if (alloc == 0) alloc = 36;
            if (alloc > 96) alloc = 96;
            uint8_t *buf = calloc(1, alloc);
            buf[0] = 0x05;  /* peripheral type: CD-ROM */
            buf[1] = 0x80;  /* removable */
            buf[2] = 0x00;  /* ATAPI version */
            buf[3] = 0x21;  /* response format = 2, HiSup */
            buf[4] = 31;    /* additional length */
            /* Vendor (8 bytes) */
            memcpy(buf + 8,  "VTECH   ", 8);
            /* Product (16 bytes) */
            memcpy(buf + 16, "V.Flash CD-ROM  ", 16);
            /* Revision (4 bytes) */
            memcpy(buf + 32, "1.00", 4);
            atapi_start_data_in(a, buf, alloc);
            break;
        }

        case 0x1A: { /* MODE SENSE(6) */
            uint8_t alloc = cdb[4] ? cdb[4] : 4;
            uint8_t *buf = calloc(1, alloc);
            buf[0] = alloc - 1; /* mode data length */
            buf[1] = 0x05;     /* medium type: CD-ROM */
            atapi_start_data_in(a, buf, alloc);
            break;
        }

        case 0x1E: /* PREVENT/ALLOW MEDIUM REMOVAL */
            atapi_complete_ok(a);
            break;

        case 0x25: { /* READ CAPACITY */
            if (!vf->cd || !vf->cd->is_open) {
                atapi_complete_err(a, 0x02, 0x3A, 0x00);
                break;
            }
            uint8_t *buf = calloc(1, 8);
            uint32_t last_lba = vf->cd->sector_count - 1;
            buf[0] = (last_lba >> 24) & 0xFF;
            buf[1] = (last_lba >> 16) & 0xFF;
            buf[2] = (last_lba >> 8) & 0xFF;
            buf[3] = last_lba & 0xFF;
            buf[4] = 0; buf[5] = 0; buf[6] = 0x08; buf[7] = 0x00; /* 2048 bytes */
            atapi_start_data_in(a, buf, 8);
            break;
        }

        case 0x28: { /* READ(10) */
            if (!vf->cd || !vf->cd->is_open) {
                atapi_complete_err(a, 0x02, 0x3A, 0x00);
                break;
            }
            uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                           ((uint32_t)cdb[4] << 8)  | cdb[5];
            uint16_t count = ((uint16_t)cdb[7] << 8) | cdb[8];
            if (count == 0) { atapi_complete_ok(a); break; }
            uint32_t total = (uint32_t)count * 2048;
            uint8_t *buf = malloc(total);
            if (!buf) { atapi_complete_err(a, 0x04, 0x00, 0x00); break; }
            for (uint16_t i = 0; i < count; i++) {
                if (!cdrom_read_sector(vf->cd, lba + i, buf + i * 2048)) {
                    free(buf);
                    atapi_complete_err(a, 0x03, 0x11, 0x00); /* medium error */
                    return;
                }
            }
            if (a->log_count < 200)
                printf("[ATAPI] READ(10): LBA=%u count=%u (%u bytes)\n", lba, count, total);
            atapi_start_data_in(a, buf, total);
            break;
        }

        case 0x43: { /* READ TOC */
            uint16_t alloc = ((uint16_t)cdb[7] << 8) | cdb[8];
            uint8_t format = cdb[2] & 0x0F;
            if (alloc == 0) alloc = 12;
            if (alloc > 804) alloc = 804;
            uint8_t *buf = calloc(1, alloc);
            if (format == 0) {
                /* Format 0: TOC — single data track */
                uint32_t last_lba = vf->cd ? vf->cd->sector_count : 0;
                int len = 10; /* 2 header + 8 per track descriptor (1 track + lead-out) */
                if (alloc >= 4) {
                    buf[0] = 0; buf[1] = len; /* TOC data length */
                    buf[2] = 1; buf[3] = 1;   /* first/last track */
                }
                if (alloc >= 12) {
                    /* Track 1 descriptor */
                    buf[4] = 0; buf[5] = 0x14; /* ADR=1, control=data */
                    buf[6] = 1;  /* track number */
                    buf[7] = 0;
                    buf[8] = 0; buf[9] = 0; buf[10] = 0; buf[11] = 0; /* LBA 0 */
                }
                if (alloc >= 20) {
                    len = 18;
                    buf[0] = 0; buf[1] = len;
                    /* Lead-out (track 0xAA) */
                    buf[12] = 0; buf[13] = 0x14;
                    buf[14] = 0xAA; buf[15] = 0;
                    buf[16] = (last_lba >> 24) & 0xFF;
                    buf[17] = (last_lba >> 16) & 0xFF;
                    buf[18] = (last_lba >> 8) & 0xFF;
                    buf[19] = last_lba & 0xFF;
                }
            }
            atapi_start_data_in(a, buf, alloc);
            break;
        }

        case 0x46: /* GET CONFIGURATION */
        case 0x4A: /* GET EVENT STATUS NOTIFICATION */
        case 0x51: /* READ DISC INFORMATION */
        case 0x5A: { /* MODE SENSE(10) */
            uint16_t alloc = ((uint16_t)cdb[7] << 8) | cdb[8];
            if (alloc == 0) alloc = 8;
            uint8_t *buf = calloc(1, alloc);
            buf[0] = 0; buf[1] = 6; /* mode data length */
            atapi_start_data_in(a, buf, alloc < 8 ? alloc : 8);
            break;
        }

        case 0xBE: { /* READ CD */
            if (!vf->cd || !vf->cd->is_open) {
                atapi_complete_err(a, 0x02, 0x3A, 0x00);
                break;
            }
            uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                           ((uint32_t)cdb[4] << 8)  | cdb[5];
            uint32_t count = ((uint32_t)cdb[6] << 16) | ((uint32_t)cdb[7] << 8) | cdb[8];
            if (count == 0) { atapi_complete_ok(a); break; }
            uint32_t total = count * 2048;
            uint8_t *buf = malloc(total);
            if (!buf) { atapi_complete_err(a, 0x04, 0x00, 0x00); break; }
            for (uint32_t i = 0; i < count; i++) {
                if (!cdrom_read_sector(vf->cd, lba + i, buf + i * 2048)) {
                    free(buf);
                    atapi_complete_err(a, 0x03, 0x11, 0x00);
                    return;
                }
            }
            atapi_start_data_in(a, buf, total);
            break;
        }

        default:
            printf("[ATAPI] Unknown PACKET opcode: 0x%02X\n", opcode);
            atapi_complete_err(a, 0x05, 0x20, 0x00); /* ILLEGAL REQUEST, invalid opcode */
            break;
    }
}

/* ---- MJP decode helper ----
 * Pair-swap + byte-stuffing restore + JPEG decode.
 * Input: raw MIAV chunk data (16-bit swapped, no byte-stuffing).
 * If 'header' is non-NULL, prepend it (for P-frames that lack SOI/tables).
 * Returns 1 on success (framebuf updated), 0 on failure. */
static int mjp_decode_raw(MJPDecoder *dec, const uint8_t *raw, uint32_t raw_sz,
                          const uint8_t *header, uint32_t hdr_len) {
    if (raw_sz < 4) return 0;
    /* Pair-swap */
    uint8_t *swapped = malloc(hdr_len + raw_sz + 2);
    if (!swapped) return 0;
    uint32_t off = 0;

    /* Copy header if P-frame */
    int need_header = (raw_sz < 2 || raw[1] != 0xD8 || raw[0] != 0xFF);
    /* After swap, check: swapped[0]=raw[1], swapped[1]=raw[0] */
    int has_soi = (raw_sz >= 2 && raw[1] == 0xFF && raw[0] == 0xD8);
    if (!has_soi && header && hdr_len > 0) {
        memcpy(swapped, header, hdr_len);
        off = hdr_len;
    }

    /* Pair-swap raw data */
    for (uint32_t i = 0; i + 1 < raw_sz; i += 2) {
        swapped[off + i]     = raw[i + 1];
        swapped[off + i + 1] = raw[i];
    }
    if (raw_sz & 1) swapped[off + raw_sz - 1] = raw[raw_sz - 1];
    uint32_t total = off + raw_sz;

    /* Find SOS to locate entropy start */
    uint32_t sos_data = total;
    uint32_t hp = 2;
    while (hp + 3 < total && swapped[hp] == 0xFF) {
        uint8_t m = swapped[hp + 1];
        uint16_t mlen = ((uint16_t)swapped[hp+2] << 8) | swapped[hp+3];
        if (m == 0xDA) { sos_data = hp + 2 + mlen; break; }
        hp += 2 + mlen;
    }

    /* Re-insert byte-stuffing in entropy data.
     * V.Flash hardware JPEG encoder omits FF 00 byte-stuffing.
     * Only preserve FF D9 (EOI) — everything else gets stuffed.
     * Don't preserve FF D0-D7 (restart markers) as the encoder
     * likely doesn't use them and these bytes are entropy data. */
    uint8_t *fixed = malloc(total + total / 2 + 16);
    if (!fixed) { free(swapped); return 0; }
    memcpy(fixed, swapped, sos_data);
    uint32_t fi = sos_data;
    for (uint32_t ei = sos_data; ei < total; ei++) {
        fixed[fi++] = swapped[ei];
        if (swapped[ei] == 0xFF) {
            if (ei + 1 < total) {
                uint8_t nxt = swapped[ei + 1];
                if (nxt == 0xD9) {
                    fixed[fi++] = 0xD9;
                    ei++;
                    break;
                } else if (nxt == 0x00) {
                    fixed[fi++] = 0x00;
                    ei++;
                } else {
                    fixed[fi++] = 0x00; /* stuff it */
                }
            } else {
                fixed[fi++] = 0x00; /* trailing FF — stuff it */
            }
        }
    }
    if (fi < 2 || fixed[fi-2] != 0xFF || fixed[fi-1] != 0xD9)
        { fixed[fi++] = 0xFF; fixed[fi++] = 0xD9; }

    int ok = mjp_decode_frame(dec, fixed, fi);
    free(fixed);
    free(swapped);
    return ok;
}

/* Write to ATAPI command register (reg 0x07) */
static void atapi_write_command(VFlash *vf, uint8_t cmd) {
    ATAPIState *a = &vf->atapi;

    switch (cmd) {
        case 0x08: /* DEVICE RESET */
            atapi_init(a);
            printf("[ATAPI] Device reset\n");
            break;

        case 0xA0: /* PACKET */
            a->state = ATAPI_STATE_CMD;
            a->packet_pos = 0;
            a->status = ATA_SR_DRDY | ATA_SR_DRQ;
            a->sect_count = ATAPI_IR_COD; /* command phase, from host */
            if (a->log_count < 200)
                printf("[ATAPI] PACKET command started\n");
            break;

        case 0xA1: { /* IDENTIFY PACKET DEVICE */
            uint8_t *buf = calloc(1, 512);
            /* Word 0: general config — ATAPI CD-ROM, 12-byte packets */
            buf[0] = 0x80; buf[1] = 0x85; /* 0x8580: removable, ATAPI, 12-byte cmd */
            /* Words 27-46: model string (40 ASCII chars, swapped pairs) */
            const char *model = "V.Flash CD-ROM                          ";
            for (int i = 0; i < 40; i += 2) {
                buf[54 + i]     = model[i + 1];
                buf[54 + i + 1] = model[i];
            }
            /* Word 49: capabilities — LBA, DMA */
            buf[98] = 0x00; buf[99] = 0x03;
            /* Word 53: validity */
            buf[106] = 0x06; buf[107] = 0x00;
            /* Word 63: multiword DMA */
            buf[126] = 0x07; buf[127] = 0x00;
            atapi_start_data_in(a, buf, 512);
            printf("[ATAPI] IDENTIFY PACKET DEVICE\n");
            break;
        }

        case 0xEC: /* IDENTIFY DEVICE — not valid for ATAPI, return error */
            a->status = ATA_SR_DRDY | ATA_SR_ERR;
            a->error = 0x04; /* ABRT */
            a->sect_count = ATAPI_IR_COD | ATAPI_IR_IO;
            /* Set signature for ATAPI device */
            a->lba_low = 0x01;
            a->byte_count_lo = 0x14;
            a->byte_count_hi = 0xEB;
            break;

        default:
            printf("[ATAPI] Unknown ATA command: 0x%02X\n", cmd);
            a->status = ATA_SR_DRDY | ATA_SR_ERR;
            a->error = 0x04; /* ABRT */
            break;
    }
}

/* Read 16-bit word from ATAPI data port */
static uint16_t atapi_read_data16(ATAPIState *a) {
    if (a->state != ATAPI_STATE_DATA_IN || !a->data_buf)
        return 0;
    uint16_t val = 0;
    if (a->data_pos < a->data_size)
        val = a->data_buf[a->data_pos++];
    if (a->data_pos < a->data_size)
        val |= (uint16_t)a->data_buf[a->data_pos++] << 8;
    if (a->data_pos >= a->data_size) {
        free(a->data_buf);
        a->data_buf = NULL;
        atapi_complete_ok(a);
    }
    return val;
}

/* Read 32-bit word from ATAPI data port */
static uint32_t atapi_read_data32(ATAPIState *a) {
    uint16_t lo = atapi_read_data16(a);
    uint16_t hi = atapi_read_data16(a);
    return ((uint32_t)hi << 16) | lo;
}

/* Write 16-bit word to ATAPI data port (for PACKET CDB) */
static void atapi_write_data16(VFlash *vf, uint16_t val) {
    ATAPIState *a = &vf->atapi;
    if (a->state == ATAPI_STATE_CMD) {
        /* Receiving 12-byte PACKET CDB, 2 bytes at a time */
        if (a->packet_pos < 12)
            a->packet[a->packet_pos++] = val & 0xFF;
        if (a->packet_pos < 12)
            a->packet[a->packet_pos++] = (val >> 8) & 0xFF;
        if (a->packet_pos >= 12) {
            a->status = ATA_SR_BSY;
            atapi_exec_packet(vf);
            a->log_count++;
        }
    }
}

/* Write 32-bit word to ATAPI data port */
static void atapi_write_data32(VFlash *vf, uint32_t val) {
    atapi_write_data16(vf, val & 0xFFFF);
    atapi_write_data16(vf, (val >> 16) & 0xFFFF);
}

/* Read ATAPI register */
static uint32_t atapi_read_reg(VFlash *vf, uint32_t reg) {
    ATAPIState *a = &vf->atapi;
    uint32_t val = 0;

    switch (reg) {
        case 0x00: val = atapi_read_data16(a); break;
        case 0x01: val = a->error; break;
        case 0x02: val = a->sect_count; break;
        case 0x03: val = a->lba_low; break;
        case 0x04: val = a->byte_count_lo; break;
        case 0x05: val = a->byte_count_hi; break;
        case 0x06: val = a->drive_head; break;
        case 0x07: val = a->status; break;
        case 0x0E: /* alt status (no side effects) */
        case 0x17:
        case 0x27:
            val = a->status; break;
        case 0x10: val = atapi_read_data16(a); break; /* 16-bit data */
        case 0x14: val = atapi_read_data32(a); break; /* 32-bit data */
        default:
            /* ZEVIO wrapper registers */
            if (reg < 256)
                val = a->zevio_regs[reg >> 2];
            break;
    }

    if (a->log_count < 200 && reg != 0x07 && reg != 0x0E && reg != 0x17 && reg != 0x27)
        printf("[ATAPI] Read reg 0x%02X = 0x%08X (PC=0x%08X)\n",
               reg, val, vf->cpu.r[15]);
    return val;
}

/* Write ATAPI register */
static void atapi_write_reg(VFlash *vf, uint32_t reg, uint32_t val) {
    ATAPIState *a = &vf->atapi;

    if (a->log_count < 200)
        printf("[ATAPI] Write reg 0x%02X = 0x%08X (PC=0x%08X)\n",
               reg, val, vf->cpu.r[15]);

    switch (reg) {
        case 0x00: atapi_write_data16(vf, val & 0xFFFF); break;
        case 0x01: a->features = val; break;
        case 0x02: a->sect_count = val; break;
        case 0x03: a->lba_low = val; break;
        case 0x04: a->byte_count_lo = val; break;
        case 0x05: a->byte_count_hi = val; break;
        case 0x06: a->drive_head = val; break;
        case 0x07: atapi_write_command(vf, val); break;
        case 0x0E: a->dev_ctrl = val;
                   if (val & 0x04) atapi_init(a); /* SRST */
                   break;
        case 0x10: atapi_write_data16(vf, val & 0xFFFF); break;
        case 0x14: atapi_write_data32(vf, val); break;
        default:
            /* ZEVIO wrapper registers */
            if (reg < 256)
                a->zevio_regs[reg >> 2] = val;
            break;
    }
}

/* Simple 4096-entry direct-mapped TLB for 1MB section mappings.
 * Indexed by VA bits [31:20]. Each entry stores the PA base (bits[31:20])
 * with bit 0 set as valid flag. Flushed when TTB or MMU control changes. */
/* UNDEF callback: copy ROM to RAM when BOOT.BIN redirect triggers.
 * At this point flash remap and SDRAM calibration are complete,
 * so copying ROM to RAM above 0x10000 is safe. This gives BOOT.BIN
 * init access to µMORE kernel code for task registration. */
static void undef_rom_copy(void *ctx) {
    VFlash *vf = ctx;
    if (!vf->rom || vf->rom_size == 0) return;
    uint32_t start = 0x10000; /* skip first 64KB (boot code area) */
    uint32_t copy_sz = vf->rom_size;
    if (copy_sz > VFLASH_RAM_SIZE) copy_sz = VFLASH_RAM_SIZE;
    for (uint32_t i = start; i < copy_sz; i += 4) {
        uint32_t rv = *(uint32_t*)(vf->rom + i);
        uint32_t mv = *(uint32_t*)(vf->ram + i);
        if (mv == 0 && rv != 0)
            *(uint32_t*)(vf->ram + i) = rv;
    }
    printf("[UNDEF-COPY] ROM → RAM[0x%X+]: %u KB (merge)\n",
           start, (copy_sz - start) / 1024);

    /* Copy µMORE kernel modules from ROM to RAM.
     * ROM init at 0x544 does: memcpy(RAM[0xAC000], ROM[0xAE010], 0xF95B0)
     * This copies ~1MB of µMORE data (task tables, function pointers, etc.)
     * from ROM's data section to SDRAM. Without this, ROM init hits a NULL
     * function pointer at 0xA9D80 → UNDEF. */
    uint32_t dst_off  = 0xAC000;
    uint32_t src_rom  = 0xAE010;
    /* Copy all ROM data from 0xAE010 to end of ROM content (0x1A75BC).
     * The original 0xF95B0 size doesn't cover task_table at 0x1A9440. */
    uint32_t mod_size = vf->rom_size - src_rom;
    if (src_rom + mod_size <= vf->rom_size && dst_off + mod_size <= VFLASH_RAM_SIZE) {
        memcpy(vf->ram + dst_off, vf->rom + src_rom, mod_size);
        printf("[UNDEF-COPY] ROM[0x%X] → RAM[0x%X]: %u KB (µMORE modules)\n",
               src_rom, dst_off,
               mod_size / 1024);
    }
}

#define TLB_SIZE 4096
static uint32_t tlb_cache[TLB_SIZE]; /* PA base | 1(valid). 0=invalid. */
static uint32_t tlb_ttb = 0;        /* TTB value when TLB was filled */

static inline void tlb_flush(void) {
    memset(tlb_cache, 0, sizeof(tlb_cache));
}

/* MMU virtual→physical address translation (ARM926EJ-S two-level page table)
 * Supports: section (1MB), coarse page table (4KB/64KB pages), fault.
 * Called on every memory access when MMU is enabled. */
static uint32_t mmu_translate(VFlash *vf, uint32_t va) {
    CP15 *cp = &vf->cpu.cp15;
    if (!cp->mmu_enabled)
        return va;

    /* Check for MCR c8 TLB invalidate */
    if (__builtin_expect(cp->tlb_flush_needed, 0)) {
        tlb_flush();
        cp->tlb_flush_needed = 0;
    }

    /* TLB lookup (section-only cache) */
    uint32_t l1_index = va >> 20;
    uint32_t tlb_entry = tlb_cache[l1_index];
    if (tlb_entry & 1) {
        /* TLB hit — reconstruct PA from cached section base */
        return (tlb_entry & 0xFFF00000u) | (va & 0x000FFFFFu);
    }

    /* Invalidate TLB if TTB changed (covers MCR c2/c8 writes) */
    if (__builtin_expect(cp->ttb != tlb_ttb, 0)) {
        tlb_flush();
        tlb_ttb = cp->ttb;
    }
    uint32_t l1_addr  = cp->ttb + (l1_index << 2);
    uint32_t l1_desc  = phys_read32(vf, l1_addr);
    uint32_t type     = l1_desc & 3;

    switch (type) {
        case 0: /* Fault — return VA unchanged (will hit unmapped handler) */
            return va;
        case 2: { /* Section (1MB) */
            uint32_t pa = (l1_desc & 0xFFF00000u) | (va & 0x000FFFFFu);
            /* Log first translation of VA 0 section — critical for boot model */
            if (l1_index == 0) {
                static int va0_logged = 0;
                if (!va0_logged) {
                    printf("[MMU] VA 0x%08X → PA 0x%08X (section L1[0]=0x%08X, TTB=0x%08X)\n",
                           va, pa, l1_desc, cp->ttb);
                    va0_logged = 1;
                }
            }
            /* Cache in TLB */
            tlb_cache[l1_index] = (l1_desc & 0xFFF00000u) | 1;
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
    /* NULL pointer dereference trap: when game reads [0x00-0x1F]
     * during gameplay, it's accessing a NULL entity pointer.
     * Return dummy vtable ptr for [0x00] so virtual calls go to stubs.
     * Return 1 ("ready") for other offsets to break polling loops. */
    if (addr < 0x20 && vf->boot_phase >= 900 &&
        (vf->cpu.cpsr & 0x1F) != 0x12 && /* not IRQ mode (vector fetch) */
        (vf->cpu.cpsr & 0x1F) != 0x11) { /* not FIQ mode */
        /* Initialize dummy vtable once at 0x10310000 */
        static int vtbl_init = 0;
        if (!vtbl_init) {
            /* Fill with ARM "MOV R0, #0; BX LR" stubs (return 0) */
            /* E3A00000 = MOV R0, #0; E12FFF1E = BX LR */
            for (int vi = 0; vi < 256; vi++) {
                *(uint32_t*)(vf->ram + 0x310000 + vi*8) = 0xE3A00000;
                *(uint32_t*)(vf->ram + 0x310004 + vi*8) = 0xE12FFF1E;
            }
            /* Vtable: array of pointers to stubs */
            for (int vi = 0; vi < 64; vi++) {
                *(uint32_t*)(vf->ram + 0x310800 + vi*4) = 0x10310000 + vi*8;
            }
            vtbl_init = 1;
        }
        if (addr < 4) return 0x10310800; /* vtable pointer */
        return 1; /* status/flag byte */
    }

    /* Fast path: RAM (most common — ~90% of all accesses) */
    if (__builtin_expect(addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE, 1)) {
        uint32_t roff = addr - VFLASH_RAM_BASE;
        /* Flash completion: µMORE loop checks [0x10BBCFF4] and [0x10BBD010].
         * Force bit 0 on both to signal operation complete. */
        /* (render context trace removed) */
        /* Detect game task reaching BOOT.BIN code (0x10C00000+) */
        if (vf->boot_phase >= 800 && roff >= 0xC00000 && roff < 0xE00000) {
            static int bootbin_access = 0;
            if (bootbin_access < 5) {
                printf("[GAME-CODE] BOOT.BIN read at [%08X] from PC=%08X\n",
                       addr, vf->cpu.r[15]);
                bootbin_access++;
            }
        }
        /* Detect framebuffer writes */
        if (vf->boot_phase >= 800 && roff >= 0x800000 && roff < 0x900000) {
            static int fb_read = 0;
            if (fb_read < 3)
                printf("[FB-ACCESS] Read [%08X] from PC=%08X\n", addr, vf->cpu.r[15]);
            fb_read++;
        }
        /* Ghidra: game task checks *[0x10B05A18] > 7 and *[0x10B05A1C] > 7. */
        if (roff == 0xB05A18 || roff == 0xB05A1C) {
            return 8;
        }
        /* Ghidra: game_tick checks *[0x10B009C4] != 0. */
        if (roff == 0xB009C4) return 1;
        /* Force game_mode to 3 (gameplay), prevent error mode 4. */
        if (roff == 0xB902C0) return 3;
        /* Ghidra: scheduler state at *[0x10BA63E0] must be 1 or 3. */
        if (roff == 0xBA63E0) return 3;
        /* Ghidra: FUN_10a08ed8 checks *[0x10B606C0] == 1 (scheduler pool active). */
        if (roff == 0xB606C0) return 1;
        /* Flash completion flags — always return with bit0 set. */
        if (roff == 0xBBCFF4 || roff == 0xBBD010) {
            return *(uint32_t*)(vf->ram + roff) | 1;
        }
        /* Trace ALL RAM reads from init task event_wait area. */
        if (vf->boot_phase >= 400) {
            uint32_t caller = vf->cpu.r[15];
            if (caller >= 0x10011000 && caller <= 0x10012000) {
                static int ew_trace = 0;
                if (ew_trace < 30) {
                    printf("[EW-TRACE] PC=%08X read [%08X]=%08X\n",
                           caller, addr, *(uint32_t*)(vf->ram + roff));
                    ew_trace++;
                }
            }
        }
        if (roff == 0x1C9C && vf->atapi.reboot_count >= 2) {
            static int rd_wp = 0;
            uint32_t v = *(uint32_t*)(vf->ram + 0x1C9C);
            if (rd_wp < 5)
                printf("[RD-1C9C] read 0x%08X PC=0x%08X reboot=%d\n",
                       v, vf->cpu.r[15], vf->atapi.reboot_count);
            rd_wp++;
        }
        return *(uint32_t*)(vf->ram + roff);
    }

    /* Physical address 0:
     * Before flash remap: 512KB read-only NOR flash
     * After flash remap: separate low_ram buffer (NOT aliased with SDRAM)
     * HLE mode (no ROM): direct SDRAM (vector table at 0x10000000) */
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped && addr < vf->rom_size)
            return *(uint32_t*)(vf->rom + addr);
        if (vf->rom_remapped && addr < sizeof(vf->low_ram))
            return *(uint32_t*)(vf->low_ram + addr);
        if (addr < VFLASH_RAM_SIZE)
            return *(uint32_t*)(vf->ram + addr);
        return 0;
    }

    /* High vector mirror at 0xFFFF0000–0xFFFFFFFF (ARM HIVEC, CR1.V=1)
     * Mirror of physical addr 0 (boot ROM). */
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;
        if (vf->has_rom && off + 3 < vf->rom_size)
            return *(uint32_t*)(vf->rom + off);
        return 0;
    }

    /* RAM */
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        /* Trace RAM reads from scheduler loop to find the blocking variable */
        uint32_t caller = vf->cpu.r[15];
        if (vf->boot_phase >= 300 && caller >= 0x10A1C540 && caller <= 0x10A1C5F0) {
            static int ram_trace = 0;
            uint32_t roff = addr - VFLASH_RAM_BASE;
            uint32_t val = *(uint32_t*)(vf->ram + roff);
            if (ram_trace < 20 && roff != 0xA1C590) { /* skip pool reads */
                printf("[RAM-TRACE] PC=%08X read [%08X]=%08X\n", caller, addr, val);
                ram_trace++;
            }
        }
        return *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE));
    }

    /* Internal SRAM / TCM */
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return *(uint32_t*)(vf->sram + (addr - VFLASH_SRAM_BASE));

    if (addr >= VFLASH_IO_BASE) {
        uint32_t off = addr - VFLASH_IO_BASE;

        /* Trace I/O reads from init task (SVC mode, PC in kernel area) */
        if (vf->boot_phase >= 400 && (vf->cpu.cpsr & 0x1F) == 0x13) {
            uint32_t caller = vf->cpu.r[15];
            if (caller >= 0x10010000 && caller < 0x10120000) {
                static int init_io = 0;
                if (init_io < 30) {
                    printf("[INIT-IO] read 0x%08X (off=%08X) PC=%08X\n",
                           addr, off, caller);
                    init_io++;
                }
            }
        }
        /* Trace I/O reads during FIQ/IRQ handler */
        if (((vf->cpu.cpsr & 0x1F) == 0x11 || (vf->cpu.cpsr & 0x1F) == 0x12) && vf->boot_phase >= 300) {
            static int fiq_io = 0;
            if (fiq_io < 30) {
                printf("[FIQ-IO] read 0x%08X (off=%08X) PC=%08X\n",
                       addr, off, vf->cpu.r[15]);
                fiq_io++;
            }
        }

        /* Primary IRQ + timers: 0x80000000+0x000–0x1FF */
        if (off < 0x200)
            return ztimer_read(&vf->timer, off);

        /* DMA engine at 0xA0000000 (off = 0x20000000).
         * Init task reads 0xC0/0xC4 for DMA channel status. */
        if (off >= 0x20000000u && off < 0x20001000u) {
            uint32_t dreg = off - 0x20000000u;
            switch (dreg) {
                case 0xC0: return 0x01; /* DMA channel 0 complete */
                case 0xC4: return 0x01; /* DMA channel 1 complete */
                default: return 0;
            }
        }

        /* V.Flash input controller at 0x900D0000 (off = 0x100D0000).
         * Init task reads 0x18 — possibly interrupt status or clock/power.
         * During game loop, return 0 to avoid confusing IRQ handler. */
        /* V.Flash input controller at 0x900D0000.
         * Register 0x00: button state (active-low: 0 = pressed).
         * Register 0x18: input IRQ status / ready flag.
         * Button mapping: bit0=Up, bit1=Down, bit2=Left, bit3=Right,
         * bit4=Red(Z), bit5=Yellow(X), bit6=Green(C), bit7=Blue(V),
         * bit8=Enter. Directly maps to SDL input bitmask. */
        if (off >= 0x100D0000u && off < 0x100D1000u) {
            uint32_t preg = off - 0x100D0000u;
            switch (preg) {
            case 0x00:
                /* Active-low GPIO: 0 = pressed, 1 = released */
                return (~vf->input) & 0x1FF;
            case 0x04:
                /* Edge detect: 1 if button state changed */
                return (vf->input != vf->input_prev) ? 0x1FF : 0;
            case 0x18:
                /* Input IRQ status (0 = no pending input IRQ) */
                return (vf->boot_phase >= 800) ? 0 : 0x01;
            default:
                return 0x01;
            }
        }

        /* Interrupt controller at 0xDC000000 (Firebird: interrupt.c).
         * Register layout (group = addr >> 8 & 3):
         * Group 0 (0x000-0x0FF): IRQ registers
         * Group 1 (0x100-0x1FF): FIQ registers
         * Per group:
         *   0x00: masked status (status & mask)
         *   0x04: raw status
         *   0x08: mask set
         *   0x0C: mask clear
         *   0x20: current highest priority interrupt (read-only)
         *   0x24: acknowledge (read: returns current int + lowers pri limit)
         *   0x28: end-of-interrupt (read: restores pri limit, may clear IRQ)
         *   0x2C: priority limit */
        if (off >= 0x5C000000u && off < 0x5C001000u) {
            uint32_t sreg = off - 0x5C000000u;
            int group = (sreg >> 8) & 3;
            int reg = sreg & 0xFF;

            /* Active interrupt sources: bit 0 = timer */
            uint32_t active = (vf->timer.irq.status & vf->timer.irq.enable) ? 1 : 0;

            if (group < 2) { /* IRQ (0) or FIQ (1) group */
                switch (reg) {
                    case 0x00: return active;   /* masked status */
                    case 0x04: return active;   /* raw status */
                    case 0x08: return 0xFFFFFFFF; /* mask (all enabled) */
                    case 0x0C: return 0xFFFFFFFF;
                    case 0x20: return active ? 0 : (uint32_t)-1; /* current int */
                    case 0x24: /* ACK: return current int, lower priority */
                        return active ? 0 : (uint32_t)-1; /* int 0 = timer */
                    case 0x28: /* EOI: clear interrupt line, return prev pri limit */
                        vf->timer.irq.status &= ~1; /* clear timer */
                        vf->timer.timer[0].irq_pending = 0;
                        return 0;
                    case 0x2C: return 0; /* priority limit */
                    default: return 0;
                }
            } else if (group == 2) {
                switch (reg) {
                    case 0x00: return 0xFFFFFFFF; /* noninverted */
                    case 0x04: return 0; /* sticky */
                    default: return 0;
                }
            }
            return 0;
        }

        /* PL111 LCD Controller at 0xC0000000 (off = 0x40000000) */
        if (off >= 0x40000000u && off < 0x40001000u) {
            uint32_t lreg = off - 0x40000000u;
            switch (lreg) {
                case 0x00: case 0x04: case 0x08: case 0x0C:
                    return vf->lcd.timing[lreg >> 2];
                case 0x10: return vf->lcd.upbase;
                case 0x14: return vf->lcd.lpbase;
                case 0x18: return vf->lcd.control;
                case 0x1C: return vf->lcd.imsc;
                case 0x20: return vf->lcd.ris;
                case 0x24: return vf->lcd.ris & vf->lcd.imsc; /* MIS */
                case 0x84: return 0x10; /* ZEVIO LCD DMA status: transfer complete (bit 4) */
                default: return 0;
            }
        }

        /* PL190 VIC at 0xDC000000 (off = 0x5C000000) */
        /* Secondary VIC for 16-bit reads: same as 32-bit handler */
        if (off >= 0x5C000000u && off < 0x5C001000u) {
            uint32_t active = (vf->timer.irq.status & vf->timer.irq.enable) ? 1 : 0;
            int reg = (off - 0x5C000000u) & 0xFF;
            switch (reg) {
                case 0x00: return active;
                case 0x04: return active;
                case 0x24: return active ? 0 : (uint32_t)-1;
                default: return 0;
            }
        }

        /* NOR Flash controller at 0xB8000000 (2MB). */
        if (off >= 0x38000000u && off < 0x38200000u && vf->has_rom) {
            uint32_t foff = off - 0x38000000u;

            /* Display controller regs 0x100-0x1FF: read-back stored values */
            if (foff >= 0x100 && foff < 0x200 && vf->boot_phase >= 800) {
                uint32_t rv = vf->dc_regs[foff - 0x100];
                return rv;
            }
            /* Display controller registers at 0xB8000700-0xB80007FF (reads). */
            if (foff >= 0x700 && foff < 0x800 && vf->boot_phase >= 800) {
                switch (foff) {
                case 0x770: return 1;    /* status: ready */
                case 0x774: return 0;    /* clear */
                case 0x784: return 0;    /* irq handler (none) */
                case 0x7A8: return 1;    /* enabled */
                case 0x7AC: return 0;    /* ID/status */
                case 0x7B0: return vf->lcd.upbase; /* framebuffer addr */
                default:    return 0;
                }
            }

            if (foff < 0x800) {
                /* NOR flash: µMORE writes data then polls to verify.
                 * Return last written value (flash write completes instantly).
                 * If nothing was written, return 0 (erased state). */
                if (foff < 0x10) {
                    /* Return last written value if any, else ROM data.
                     * Flash write verify: read-back must match written data. */
                    if (vf->flash_last_write)
                        return vf->flash_last_write;
                    /* No write yet — return ROM data (flash in read mode) */
                    if (foff < vf->rom_size)
                        return *(uint32_t*)(vf->rom + foff);
                    return 0xFFFFFFFF; /* erased flash */
                }
                switch (foff) {
                    case 0x34: return 0x40;   /* Status: ready */
                    case 0x40: return 1;
                }
                /* ROM data */
                if (foff < vf->rom_size)
                    return *(uint32_t*)(vf->rom + foff);
                return 0;
            }
            /* Flash window at 0xB8000800+N.
             * Always read from flash_buf (ROM init code), NOT from RAM.
             * The remap only controls where the DMA flush copies data to.
             * The SDRAM calibration test writes patterns to RAM[0x10000000+]
             * which overlaps with the flushed area — reading from RAM here
             * would return corrupted data (test patterns instead of code). */
            uint32_t boff = foff - 0x800;
            if (vf->flash_buf_dirty && boff + 3 < sizeof(vf->flash_buf))
                return *(uint32_t*)(vf->flash_buf + boff);
            /* Fallback: ROM data */
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
            return atapi_read_reg(vf, areg);
        }

        /* DMA / CD-ROM controller at 0x8FFF0000 (off = 0x0FFF0000) */
        if (off >= 0x0FFF0000u && off < 0x0FFF1000u) {
            uint32_t dreg = off - 0x0FFF0000u;
            switch (dreg) {
                case 0x08: return 0x01;  /* DMA status: complete */
                default:   return 0;
            }
        }

        /* SP804 Fast Timer at 0x90010000 (off = 0x10010000) */
        if (off >= 0x10010000u && off < 0x10011000u) {
            uint32_t treg = off - 0x10010000u;
            int idx = (treg >= 0x20) ? 1 : 0;
            uint32_t reg = (treg >= 0x20) ? treg - 0x20 : treg;
            Timer *t = &vf->timer.timer[idx];
            switch (reg) {
                case 0x00: return t->load;
                case 0x04: return t->count;
                case 0x08: return t->ctrl;
                case 0x10: return t->irq_pending ? 1 : 0;  /* RIS */
                case 0x14: return (t->irq_pending && (t->ctrl & 0x20)) ? 1 : 0; /* MIS */
                default: return 0;
            }
        }

        /* SP804 Timer 1 at 0x900C0000 (off = 0x100C0000) */
        if (off >= 0x100C0000u && off < 0x100C1000u) {
            uint32_t treg = off - 0x100C0000u;
            /* Map to timer[0] — use same structure */
            Timer *t = &vf->timer.timer[0];
            switch (treg) {
                case 0x00: return t->load;
                case 0x04: return t->count;
                case 0x08: return t->ctrl;
                case 0x10: return t->irq_pending ? 1 : 0;
                case 0x14: return (t->irq_pending && (t->ctrl & 0x20)) ? 1 : 0;
                default: return 0;
            }
        }

        /* Misc System Control at 0x900A0000 (off = 0x100A0000).
         * ROM uses 0x900A0F04 as a read/write config register. */
        if (off >= 0x100A0000u && off < 0x100B0000u) {
            uint32_t sreg = off - 0x100A0000u;
            switch (sreg) {
                case 0x00: return 0x01000010; /* Model ID */
                case 0x04: return 0;
                case 0x0C: {
                    uint32_t v = vf->misc_regs[0x0C >> 2];
                    if (v) printf("[BOOT-FLAG] Read 0x900A000C = 0x%08X (warm=%d)\n", v, (v>>1)&1);
                    return v;
                }
                case 0x18: {
                    /* Display status: bit25 = VBlank toggle.
                     * Render processing checks this to know when to draw.
                     * Toggle bit25 on each read to simulate VBlank. */
                    uint32_t v = vf->misc_regs[0x18 >> 2];
                    v ^= 0x02000000;
                    vf->misc_regs[0x18 >> 2] = v;
                    return v;
                }
                case 0x28: return 0x00000000; /* ASIC ID low */
                case 0x2C: return 0x00000000; /* ASIC ID high */
                default:
                    /* Read/write registers (0x900A0F04 etc.) */
                    if ((sreg >> 2) < 64)
                        return vf->misc_regs[sreg >> 2];
                    return 0;
            }
        }

        /* Power Management (PMU) at 0x900B0000 (off = 0x100B0000) */
        if (off >= 0x100B0000u && off < 0x100B1000u) {
            uint32_t preg = off - 0x100B0000u;
            switch (preg) {
                case 0x00: return vf->pmu_regs[0] ? vf->pmu_regs[0] : 0x00141002;
                case 0x04: return vf->pmu_regs[1]; /* wake_mask */
                case 0x08: return 0x2080;     /* PMU status: bit7=PLL locked */
                case 0x0C: return 0;
                case 0x14: return vf->pmu_regs[0x14>>2] | 0x01; /* PLL lock always set */
                case 0x18: return vf->pmu_regs[0x18>>2];
                case 0x20: return vf->pmu_regs[0x20>>2];
                case 0x24: return vf->pmu_regs[0] ? vf->pmu_regs[0] : 0x00141002;
                case 0x28: return 0x114;      /* ON key not pressed */
                default:
                    if ((preg >> 2) < 16) return vf->pmu_regs[preg >> 2];
                    return 0;
            }
        }

        /* Timers at 0x900C0000-0x900DFFFF handled above */

        /* UART stub: 0x5000–0x5FFF (HLE I/O region, not real APB) */
        if (off >= 0x5000 && off < 0x6000) {
            switch (off - 0x5000) {
                case 0x00: return 0;
                case 0x04: return 0x1;   /* TX_READY */
                case 0x08: return 0xFF;
                case 0x0C: return 0x1;
                default:   return 0;
            }
        }

        /* Unknown peripheral at 0xA1000000 (off = 0x21000000) */
        if (off >= 0x21000000u && off < 0x21100000u) {
            /* BOOT.BIN reads 0xA100001C and checks bit 7 (ready) and bit 0 (status).
             * Return 0x81 = ready + status OK. */
            return 0x81;
        }

        /* GPIO at 0x90000000 (off = 0x10000000) */
        if (off >= 0x10000000u && off < 0x10001000u) {
            uint32_t greg = off - 0x10000000u;
            int port = (greg >> 6) & 7;
            switch (greg & 0x3F) {
                case 0x10: return 0xFF;    /* direction: all output */
                case 0x14: return 0;       /* output data */
                case 0x18: return 0x1F;    /* input: all buttons released */
                default:   return 0;
            }
        }

        /* 0x90020000: UART or 2nd VIC/event controller on V.Flash.
         * Scheduler polls offset 0x14 checking bit 5 (pending event).
         * Return timer IRQ pending status as bit 5. */
        if (off >= 0x10020000u && off < 0x10080000u) { /* event controller banks 0-5 */
            uint32_t ureg = (off - 0x10020000u) & 0xFFFF; /* bank-relative offset */
            switch (ureg) {
                case 0x00: return vf->timer.irq.status & vf->timer.irq.enable;
                case 0x14: /* event status: bit5 = pending.
                     * Return always-pending for all banks to unblock tasks. */
                    return 0x20;
                case 0x18: return 0x90; /* UARTFR: TX empty */
                case 0xFE0: return 0x11;
                case 0xFE4: return 0x10;
                default:    return 0;
            }
        }

        /* Fastboot/scratch at 0x90030000 (off = 0x10030000) */
        if (off >= 0x10030000u && off < 0x10031000u) {
            uint32_t freg = off - 0x10030000u;
            if (freg == 0x14)
                return 0x20; /* bit5 set — scheduler dispatch check */
            return 0;
        }

        /* Watchdog at 0x90060000 (off = 0x10060000) */
        if (off >= 0x10060000u && off < 0x10061000u) {
            uint32_t wreg = off - 0x10060000u;
            switch (wreg) {
                case 0x00: return 0xFFFFFFFF; /* WdogLoad */
                case 0x04: return 0xFFFFFFFF; /* WdogValue */
                case 0x08: return 0;          /* WdogControl */
                default:   return 0;
            }
        }

        /* RTC at 0x90090000 (off = 0x10090000).
         * ROM uses 0x900900F0-0x900900FF as scratch storage (write then read back). */
        if (off >= 0x10090000u && off < 0x10091000u) {
            uint32_t rreg = off - 0x10090000u;
            /* Return stored value for scratch/control registers */
            if ((rreg >> 2) < 64 && vf->rtc_regs[rreg >> 2] != 0)
                return vf->rtc_regs[rreg >> 2];
            switch (rreg) {
                case 0x00:  { /* RTCDR: running time counter */
                    static uint32_t rtc_counter = 0x67000000;
                    return rtc_counter++;
                }
                case 0x14:  return 0;          /* RTCRIS */
                case 0xFE0: return 0x31;       /* PL031 PID0 */
                case 0xFE4: return 0x10;       /* PL031 PID1 */
                default:    return 0;
            }
        }

        /* SDRAM controller at 0x8FFF0000 — PID registers */
        if (off >= 0x0FFF0000u && off < 0x0FFF2000u) {
            uint32_t dreg = off - 0x0FFF0000u;
            switch (dreg) {
                case 0x00: return 0x80;   /* memc_status: ready */
                case 0x08: return 0x01;   /* DMA status: complete */
                case 0x0FE0: return 0x40; /* DMC-340 PID0 */
                case 0x0FE4: return 0x13; /* PID1 */
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
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped && addr + 1 < vf->rom_size)
            return *(uint16_t*)(vf->rom + addr);
        if (vf->rom_remapped && addr + 1 < sizeof(vf->low_ram))
            return *(uint16_t*)(vf->low_ram + addr);
        if (addr + 1 < VFLASH_RAM_SIZE)
            return *(uint16_t*)(vf->ram + addr);
        return 0;
    }
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;
        if (vf->has_rom && !vf->rom_remapped && off + 1 < vf->rom_size)
            return *(uint16_t*)(vf->rom + off);
        if (off + 1 < VFLASH_RAM_SIZE)
            return *(uint16_t*)(vf->ram + off);
        return 0;
    }
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
    /* NULL entity byte read trap */
    if (addr < 0x20 && vf->boot_phase >= 900) return 1;
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped && addr < vf->rom_size)
            return vf->rom[addr];
        if (vf->rom_remapped && addr < sizeof(vf->low_ram))
            return vf->low_ram[addr];
        if (addr < VFLASH_RAM_SIZE)
            return vf->ram[addr];
        return 0;
    }
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;
        if (vf->has_rom && !vf->rom_remapped && off < vf->rom_size)
            return vf->rom[off];
        if (off < VFLASH_RAM_SIZE)
            return vf->ram[off];
        return 0;
    }
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        uint32_t roff8 = addr - VFLASH_RAM_BASE;
        /* Force game_main start flag */
        if (roff8 == 0xBE49E0 && vf->boot_phase >= 800) return 1;
        return vf->ram[roff8];
    }
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE)
        return vf->sram[addr - VFLASH_SRAM_BASE];
    if (addr >= VFLASH_IO_BASE)
        return (uint8_t)(mem_read32(ctx, addr & ~3u) >> ((addr & 3) * 8));
    return 0;
}

static void mem_write32(void *ctx, uint32_t addr, uint32_t val) {
    VFlash *vf = ctx;
    addr = mmu_translate(vf, addr);

    /* Fast path: RAM write (most common) */
    if (__builtin_expect(addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE, 1)) {
        uint32_t roff = addr - VFLASH_RAM_BASE;
        /* µMORE dispatch table slot at 0x1C9C: BOOT.BIN checks bit7.
         * Force to 0x1880 ONLY after 2nd reboot (not during SDRAM cal). */
        if (roff == 0x1C9C && vf->atapi.reboot_count >= 2) {
            val = 0x00001880;
        }
        if (roff == 0x3585E0 && val != 3) {
            val = 3;
        }
        /* Protect code+data (0x10090000-0x10BF0000) from BSS zeroing. */
        if (val == 0 && roff >= 0x90000 && roff < 0xBF0000 &&
            !(roff >= 0xBBEAE0 && roff < 0xBE3C40) &&
            vf->boot_phase >= 800) {
            return; /* silently block */
        }
        *(uint32_t*)(vf->ram + roff) = val;
        return;
    }

    /* Physical addr 0-2MB:
     * With ROM: read-only, writes ignored.
     * HLE mode (no ROM): writable RAM for vector table and ROM stubs. */
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped) return;  /* ROM is read-only before remap */
        if (vf->rom_remapped && addr < sizeof(vf->low_ram)) {
            *(uint32_t*)(vf->low_ram + addr) = val;
            return;
        }
        if (addr < VFLASH_RAM_SIZE) {
            *(uint32_t*)(vf->ram + addr) = val;
            return;
        }
        return;
    }

    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        uint32_t roff = addr - VFLASH_RAM_BASE;
        /* Ghidra: game task wait counters at [0x10B05A18] and [0x10B05A1C].
         * Must be > 7 for game to proceed. Force minimum value of 8. */
        if ((roff == 0xB05A18 || roff == 0xB05A1C) && val < 8) {
            val = 8;
        }
        /* Block game_mode write to 4 (error mode). */
        if (roff == 0xB902C0 && val == 4) {
            val = 3;
        }
        /* GPU completion flag at [10BE3CA0] (render_ctx+0x60).
         * Only protect THIS SPECIFIC BYTE, not the whole 256-byte range.
         * Previous wide range (0xBE3C40-0xBE3D40) prevented render_processing
         * from managing its own state fields (all zeros became 1). */
        /* Force game_main start flag to 1 when read */
        if (roff == 0xBE49E0 && vf->boot_phase >= 800) {
            return 1;
        }
        if (roff == 0xBE3CA0 && val == 0 && vf->boot_phase >= 900) {
            val = 1; /* keep GPU completion at "done" */
        }
        /* Protect game callback pointer at [0x10BBD3C0].
         * µMORE game task init writes 0 here — prevent clearing. */
        if (roff == 0xBBD3C0 && val == 0 && vf->boot_phase >= 800) {
            return; /* don't clear callback */
        }
        /* Note: 0x10BC2BC0 is µMORE IRQ handler flag.
         * BLNE at 0x10A15CEC: if flag!=0 → IRQ return; if 0 → continue.
         * Not manipulated — let µMORE manage it naturally. */
        /* Watchpoint: track writes to scheduler entry */
        if (roff == 0x359660 || roff == 0x3596B0 || roff == 0x9CFEC) {
            static int wp_count = 0;
            if (wp_count < 10)
                printf("[WP-HEAP] ram[0x%06X] = 0x%08X PC=0x%08X\n",
                       roff, val, vf->cpu.r[15]);
            wp_count++;
        }
        /* Watchpoint: catch overwrites of render function at 10B265E8 */
        if (roff == 0xB265E8) {
            static int rw_count = 0;
            if (rw_count < 5) {
                printf("[WP-RENDER] ram[0xB265E8] = 0x%08X PC=0x%08X (was 0x%08X)\n",
                       val, vf->cpu.r[15],
                       *(uint32_t*)(vf->ram + 0xB265E8));
                rw_count++;
            }
        }
        *(uint32_t*)(vf->ram + roff) = val;
        return;
    }

    /* Internal SRAM / TCM */
    if (addr >= VFLASH_SRAM_BASE && addr < VFLASH_SRAM_BASE + VFLASH_SRAM_SIZE) {
        *(uint32_t*)(vf->sram + (addr - VFLASH_SRAM_BASE)) = val;
        return;
    }

    if (addr >= VFLASH_IO_BASE) {
        uint32_t off = addr - VFLASH_IO_BASE;

        /* Trace ALL I/O writes after boot_phase 400 (init task) */
        if (vf->boot_phase >= 400 && (vf->cpu.cpsr & 0x1F) == 0x13) {
            static int init_iow = 0;
            if (init_iow < 30) {
                printf("[INIT-WR] write 0x%08X = 0x%08X PC=%08X\n",
                       addr, val, vf->cpu.r[15]);
                init_iow++;
            }
        }
        /* Trace I/O writes during FIQ/IRQ handler */
        if (((vf->cpu.cpsr & 0x1F) == 0x11 || (vf->cpu.cpsr & 0x1F) == 0x12) && vf->boot_phase >= 300) {
            static int fiq_iow = 0;
            if (fiq_iow < 30) {
                printf("[FIQ-IO] write 0x%08X = 0x%08X (off=%08X) PC=%08X\n",
                       addr, val, off, vf->cpu.r[15]);
                fiq_iow++;
            }
        }

        /* Primary IRQ + timers */
        if (off < 0x200) { ztimer_write(&vf->timer, off, val); return; }

        /* Secondary interrupt controller at 0xDC000000 (off = 0x5C000000) */
        if (off >= 0x5C000000u && off < 0x5C001000u) {
            uint32_t sreg = off - 0x5C000000u;
            int group = (sreg >> 8) & 3;
            int reg = sreg & 0xFF;
            if (group < 2) {
                switch (reg) {
                    case 0x04: /* clear sticky/status bits */
                        vf->timer.irq.status &= ~val;
                        vf->timer.timer[0].irq_pending = 0;
                        break;
                    case 0x2C: /* priority limit restore (EOI completion) */
                        break;
                }
            }
            return;
        }

        /* PL111 LCD Controller write at 0xC0000000 */
        if (off >= 0x40000000u && off < 0x40001000u) {
            uint32_t lreg = off - 0x40000000u;
            switch (lreg) {
                case 0x00: case 0x04: case 0x08: case 0x0C:
                    vf->lcd.timing[lreg >> 2] = val; break;
                case 0x10:
                    vf->lcd.upbase = val;
                    printf("[LCD] Framebuffer base = 0x%08X\n", val);
                    break;
                case 0x14: vf->lcd.lpbase = val; break;
                case 0x18:
                    vf->lcd.control = val;
                    if (val & 1)
                        printf("[LCD] ENABLED: ctrl=0x%08X BPP=%u fb=0x%08X\n",
                               val, 1 << ((val >> 1) & 7), vf->lcd.upbase);
                    break;
                case 0x1C: vf->lcd.imsc = val; break;
                case 0x28: vf->lcd.ris &= ~val; break; /* ICR: clear interrupts */
                default:
                    /* PL111 palette RAM at 0x200-0x3FF */
                    if (lreg >= 0x200 && lreg < 0x400) {
                        uint32_t idx = (lreg - 0x200) / 2;
                        if (idx < 256) {
                            /* PL111 palette: each 32-bit write stores two 16-bit entries */
                            vf->lcd.palette[idx] = (uint16_t)(val & 0xFFFF);
                            if (idx + 1 < 256)
                                vf->lcd.palette[idx + 1] = (uint16_t)(val >> 16);
                            if (!vf->lcd.pal_written) {
                                printf("[LCD] First palette write: [%d]=0x%04X [%d]=0x%04X\n",
                                       idx, val & 0xFFFF, idx+1, val >> 16);
                            }
                            vf->lcd.pal_written++;
                        }
                    }
                    break;
            }
            return;
        }

        /* PL190 VIC write at 0xDC000000 */
        if (off >= 0x5C000000u && off < 0x5C001000u) {
            uint32_t vreg = off - 0x5C000000u;
            switch (vreg) {
                case 0x00C: vf->timer.irq.fiq_sel = val; break;    /* IntSelect */
                case 0x010: vf->timer.irq.enable |= val; break;    /* IntEnable (set) */
                case 0x014: vf->timer.irq.enable &= ~val; break;   /* IntEnClr */
                case 0x018: vf->timer.irq.status |= val; break;    /* SoftInt */
                case 0x01C: vf->timer.irq.status &= ~val; break;   /* SoftIntClr */
            }
            return;
        }

        /* ATAPI CD-ROM write at 0xAA000000 */
        if (off >= 0x2A000000u && off < 0x2A001000u) {
            uint32_t areg = off - 0x2A000000u;
            atapi_write_reg(vf, areg, val);
            return;
        }

        /* SP804 Fast Timer write at 0x90010000 */
        if (off >= 0x10010000u && off < 0x10011000u) {
            uint32_t treg = off - 0x10010000u;
            int idx = (treg >= 0x20) ? 1 : 0;
            uint32_t reg = (treg >= 0x20) ? treg - 0x20 : treg;
            Timer *t = &vf->timer.timer[idx];
            switch (reg) {
                case 0x00: t->load = val; t->count = val; break;  /* Load */
                case 0x08: t->ctrl = val; break;                   /* Control */
                case 0x0C: t->irq_pending = 0;                     /* IntClr */
                           ztimer_clear_irq(&vf->timer, IRQ_TIMER0 + idx);
                           break;
                case 0x18: t->load = val; break;                    /* BGLoad */
            }
            return;
        }
        /* SP804 Timer 1 write at 0x900C0000 */
        if (off >= 0x100C0000u && off < 0x100C1000u) {
            uint32_t treg = off - 0x100C0000u;
            Timer *t = &vf->timer.timer[0];
            switch (treg) {
                case 0x00: t->load = val; t->count = val; break;
                case 0x08: t->ctrl = val; break;
                case 0x0C: t->irq_pending = 0;
                           ztimer_clear_irq(&vf->timer, IRQ_TIMER0);
                           break;
                case 0x18: t->load = val; break;
            }
            return;
        }

        /* DMA controller write at 0x8FFF0000 */
        if (off >= 0x0FFF0000u && off < 0x0FFF1000u) {
            uint32_t dreg = off - 0x0FFF0000u;
            switch (dreg) {
                case 0x00: vf->dma_param_a = val; break;
                case 0x04: vf->dma_param_b = val; break;
                case 0x08:
                    /* DMA enable — HLE only: trigger BOOT.BIN load from disc.
                     * In ROM boot mode, 0x8FFF0008 is the real SDRAM controller
                     * and writes here are SDRAM calibration config — NOT DMA.
                     * Loading BOOT.BIN here would overwrite the memory test area
                     * and cause calibration to always fail (R8=0). */
                    if (val == 1 && !vf->has_rom && vf->cd && vf->cd->is_open) {
                        static int dma_call = 0;
                        dma_call++;
                        /* DMA at 0x8FFF is likely NOT CD-ROM but memory/flash DMA.
                         * HLE: load BOOT.BIN from disc on first call.
                         * Params A/B/C are flash controller config, not LBA. */
                        if (dma_call == 1) {
                            /* Debug: check IRQ handler BEFORE DMA copy */
                            uint32_t pre_irq = *(uint32_t*)(vf->ram + 0x199C);
                            printf("[DMA] Pre-DMA: RAM[0x199C]=0x%08X\n", pre_irq);
                            /* Copy ROM to RAM[0x10000] but ONLY areas that are
                             * still zero. BL init functions have already written
                             * exception handlers etc. to low RAM — don't overwrite. */
                            if (vf->rom && vf->rom_size > 0) {
                                uint32_t dest = 0x10000;
                                uint32_t sz = vf->rom_size;
                                if (dest + sz > VFLASH_RAM_SIZE) sz = VFLASH_RAM_SIZE - dest;
                                /* Only copy non-zero ROM words to unwritten RAM */
                                for (uint32_t i = 0; i < sz; i += 4) {
                                    uint32_t rom_val = *(uint32_t*)(vf->rom + i);
                                    uint32_t ram_val = *(uint32_t*)(vf->ram + dest + i);
                                    if (ram_val == 0 && rom_val != 0)
                                        *(uint32_t*)(vf->ram + dest + i) = rom_val;
                                }
                                printf("[DMA#%d] ROM: %u bytes → RAM[0x%X] (merge)\n",
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
                            /* SP804 timer and PL190 VIC now handle timer/IRQ
                             * properly — no need to force-enable here. ROM BL
                             * functions configure timers via SP804 registers. */
                            printf("[DMA] ROM uses SP804+PL190 for timer/IRQ\n");
                            /* Install HLE vector table + IRQ handler.
                             * ROM BL init may not install them in normal mode
                             * due to flash remap hybrid code path differences. */
                            /* Don't overwrite vectors — let BL init set them.
                             * With pure ROM remap, BL #1 installs proper handlers. */
                            printf("[DMA] Using ROM-installed vectors and handlers\n");
                            /* Mark that DMA is done — remap will be disabled at frame boundary */
                        }
                    }
                    break;
                case 0x0C: vf->dma_param_c = val; break;
                case 0x10: break;  /* config */
            }
            return;
        }

        /* NOR Flash controller at 0xB8000000.
         * ROM init copies code to the flash window at 0xB8000800+ (word-by-word),
         * then writes a remap value to 0xB8000800, then jumps to 0xB8000804.
         * We buffer all writes to 0xB8000800+ and flush to RAM on remap. */
        if (off >= 0x38000000u && off < 0x38200000u && vf->has_rom) {
            uint32_t foff = off - 0x38000000u;

            /* Display controller registers at 0xB8000100-0xB80007FF.
             * 0x100-0x1FF: video mode/timing registers (Incredibles uses these)
             * 0x700-0x7FF: VBlank callback, framebuffer, IRQ handler
             * Only during game phase to avoid breaking ROM flash polling. */
            if (foff >= 0x100 && foff < 0x200 && vf->boot_phase >= 800) {
                /* Store in a register array for read-back */
                vf->dc_regs[foff - 0x100] = val;
                return;
            }
            if (foff >= 0x700 && foff < 0x800 && vf->boot_phase >= 800) {
                static int dc_log = 0;
                if (dc_log < 20) {
                    printf("[DC] Write 0x%08X = 0x%08X PC=%08X\n",
                           addr, val, vf->cpu.r[15]);
                    dc_log++;
                }
                /* 0x7C0: VBlank callback address */
                if (foff == 0x7C0 && val >= 0x10000000 && val < 0x11000000)
                    vf->dc_vblank_cb = val;
                /* 0x7B0: framebuffer address */
                if (foff == 0x7B0 && val >= 0x10000000 && val < 0x11000000)
                    vf->lcd.upbase = val;
                /* 0x784: interrupt handler address */
                if (foff == 0x784 && val >= 0x10000000 && val < 0x11000000)
                    vf->dc_irq_handler = val;
                /* Absorb all display controller writes */
                return;
            }

            /* Track writes to flash data area (for status polling read-back) */
            if (foff < 0x800) {
                vf->flash_last_write = val;
                return; /* absorb flash commands/data writes */
            }
            if (foff >= 0x800 && foff < 0x800 + sizeof(vf->flash_buf)) {
                uint32_t boff = foff - 0x800;
                /* Buffer the write (but don't overwrite flash_buf[0] after preload) */
                if (boff == 0 && vf->flash_preloaded) {
                    /* Skip — preloaded kernel entry must survive */
                } else {
                    *(uint32_t*)(vf->flash_buf + boff) = val;
                }
                vf->flash_buf_dirty = 1;

                /* Write to reg 0x800 = remap trigger (only on first boot) */
                if (foff == 0x800 && val != 0 && !vf->flash_preloaded) {
                    vf->flash_remap = val;
                    /* Flush buffer to RAM at remap offset.
                     * The buffer contains ROM init code that was
                     * copied word-by-word from ROM[0x7F0..0xD00].
                     * First word (offset 0) is now the remap value itself. */
                    uint32_t flush_size = sizeof(vf->flash_buf);
                    if (val + flush_size > VFLASH_RAM_SIZE)
                        flush_size = VFLASH_RAM_SIZE - val;
                    memcpy(vf->ram + val, vf->flash_buf, flush_size);
                    printf("[FLASH] Remap=0x%X, flushed %u bytes to RAM[0x%X]\n",
                           val, flush_size, val);

                    /* Speed hack: patch SDRAM calibration table.
                     * The table at flash offset 0x2D0 (RAM[remap+0x2D0]) has 71 identical
                     * entries (all 0x16) followed by 0xFF terminator. Since emulated SDRAM
                     * always works, put terminator at entry 1 (skip entire calibration). */
                    /* Patch SDRAM table in flash_buf so subsequent flushes
                     * preserve the patch. Table is at buf offset 0x2D0. */
                    uint32_t tbl_boff = 0x2D0;
                    if (tbl_boff + 8 < sizeof(vf->flash_buf)) {
                        printf("[FLASH] Table before: [0x%X]=0x%08X [0x%X]=0x%08X [0x%X]=0x%08X\n",
                               val+tbl_boff, *(uint32_t*)(vf->flash_buf + tbl_boff),
                               val+tbl_boff+4, *(uint32_t*)(vf->flash_buf + tbl_boff+4),
                               val+tbl_boff+8, *(uint32_t*)(vf->flash_buf + tbl_boff+8));
                        /* Put terminator at entry 0 → skip entire calibration */
                        *(uint32_t*)(vf->flash_buf + tbl_boff) = 0x000000FF;
                        /* Also patch RAM (current flush already happened) */
                        uint32_t tbl_off = val + tbl_boff;
                        if (tbl_off + 4 < VFLASH_RAM_SIZE)
                            *(uint32_t*)(vf->ram + tbl_off) = 0x000000FF;
                        printf("[FLASH] Patched SDRAM table: 0xFF at flash_buf[0x%X] + RAM[0x%X]\n",
                               tbl_boff, tbl_off);
                    }

                    /* Pre-load BOOT.BIN into RAM so the ROM can jump to it
                     * after calibration. On real HW, the ROM uses the ATAPI
                     * controller to read BOOT.BIN from disc. We bypass ATAPI
                     * by loading it now, so when ROM jumps to 0x10C00000+
                     * the code is already in place. */
                    if (vf->cd && vf->cd->is_open && !vf->flash_preloaded) {
                        CDEntry boot_entry;
                        if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &boot_entry)) {
                            uint32_t dest = 0xC00000; /* RAM offset for 0x10C00000 */
                            uint32_t max_sz = VFLASH_RAM_SIZE - dest;
                            if (boot_entry.size < max_sz) max_sz = boot_entry.size;
                            int rd = cdrom_read_file(vf->cd, &boot_entry,
                                vf->ram + dest, 0, max_sz);
                            printf("[ROM-PRELOAD] BOOT.BIN: %d bytes at RAM[0x%X] (0x%08X)\n",
                                   rd, dest, VFLASH_RAM_BASE + dest);
                        }
                        /* Pre-load everything: exact same copies as ROM init BLs.
                         * Init function 0x288: 64KB from ROM[0x2000] → SDRAM[0x0]
                         * Init function 0x2B4 phase 1: ROM[0x12000] → SDRAM[0xFFF0]
                         * Init function 0x2B4 phase 2: ROM[0xC02C] → SDRAM[0x9FFD4]
                         * Also copy remaining ROM code for completeness. */
                        {
                            /* Init 0x288: kernel code */
                            memcpy(vf->ram, vf->rom + 0x2000, 0x10000);
                            /* Init 0x2B4 phase 1: kernel + core modules */
                            memcpy(vf->ram + 0xFFF0, vf->rom + 0x12000, 0x8FFE4);
                            /* Init 0x2B4 phase 2: more modules */
                            memcpy(vf->ram + 0x9FFD4, vf->rom + 0xC02C, 0xA1FE4);
                            /* Extra: module area (from previous working config) */
                            memcpy(vf->ram + 0xAC000, vf->rom + 0xAE010, 0xF95B0);
                            /* Copy ROM code to RAM (overwrites phase 1/2 where
                             * ROM has non-zero data = actual code). */
                            for (uint32_t ci = 0x1000; ci < vf->rom_size; ci += 4) {
                                uint32_t rv = *(uint32_t*)(vf->rom + ci);
                                if (rv != 0) *(uint32_t*)(vf->ram + ci) = rv;
                            }
                            /* Set sched_state=3 BEFORE ROM init runs.
                             * µMORE task_start (0x85E88) checks [0x103585E0]==3
                             * and refuses to register tasks if state != 3.
                             * On real HW this is set by earlier init stages. */
                            /* DON'T fill BSS in natural boot — real init handles it.
                             * BUT set sched_state=3 AFTER BSS clear but before task_start.
                             * Real init BLs: 0x5D0(BSS clear), then 0x704(MMU), then
                             * scheduler at 0x10010234 calls task_start which needs state=3.
                             * Patch: write sched_state directly to ROM pool area so it
                             * gets set when the init code runs. */
                            /* We'll intercept in the HLE handler instead — set flag */
                            /* DON'T create dummy idle task — real init will do it.
                             * Dummy task at 0x107FD000 was interfering with real init. */
                            if (0) { /* disabled for natural boot */
                                uint32_t task_off = 0x7FD000;
                                uint32_t stk_off  = 0x7FDF00;
                                /* Context switch at 0x10087634 does:
                                 * LDR SP,[R1]  → load SP from task+0x44
                                 * POP {R0}     → pop saved CPSR
                                 * MSR CPSR,R0  → restore CPSR
                                 * POP {R0-R12,LR,PC} → restore regs (15 words)
                                 * Plus PUSH{LR} at 0x10087618 before save.
                                 * Stack frame (bottom to top): LR, CPSR, R0..R12, LR, PC */
                                uint32_t sp_off = stk_off;
                                /* Work backwards from stack_top, push in reverse order */
                                /* The POP at 0x10087640 is LDMFD = POP: {R0-R12,LR,PC} = 15 regs */
                                sp_off -= 15 * 4; /* 15 regs */
                                uint32_t *regs = (uint32_t*)(vf->ram + sp_off);
                                for (int ri = 0; ri <= 12; ri++) regs[ri] = 0; /* R0-R12 */
                                regs[13] = 0x10FFF000; /* LR → idle */
                                regs[14] = 0x10FFF000; /* PC → idle */
                                /* POP {R0} at 0x10087638 pops CPSR */
                                sp_off -= 4;
                                *(uint32_t*)(vf->ram + sp_off) = 0x000000D3;
                                /* PUSH{LR} at 0x10087618 pushed LR */
                                sp_off -= 4;
                                *(uint32_t*)(vf->ram + sp_off) = 0x10FFF000;
                                /* Save SP (as VA) at task+0x44 */
                                uint32_t saved_sp = 0x10000000 + sp_off;
                                *(uint32_t*)(vf->ram + task_off + 0x44) = saved_sp;
                                /* Task list head */
                                *(uint32_t*)(vf->ram + 0x3585C0) = 0x10000000 + task_off;
                                printf("[TASK-INIT] Idle task at 0x%08X, SP=0x%08X\n",
                                       0x10000000 + task_off, saved_sp);
                            }
                            /* Skip SDRAM calibration — jump directly to kernel.
                             * Need to set up what boot code would have done:
                             * 1. Copy vector table from ROM to RAM (PA 0 → RAM after remap)
                             * 2. Enable MMU with page table from ROM
                             * 3. Set PC to kernel loop area */
                            vf->flash_preloaded = 1;
                            /* Keep rom_remapped=0: PA 0 reads ROM (needed for
                             * flash init pool values). After MMU enable by init
                             * function 0x704, VA 0 → SDRAM via page table. */

                            /* Build page table at RAM[0xA8000] (VA=0x100A8000).
                             * Boot code normally builds this dynamically. */
                            /* DON'T enable MMU — let calibration run with MMU off.
                             * Init function 0x704 in the flash code will build the
                             * page table and enable MMU naturally after calibration. */
                            vf->cpu.cp15.mmu_enabled = 0;
                            vf->cpu.cp15.domain = 0xFFFFFFFF;
                            printf("[ROM-PRELOAD] MMU OFF (init 0x704 will enable)\n");

                            /* Let boot continue naturally — DON'T skip calibration.
                             * SDRAM always works in emulator, so calibration will pass.
                             * The ROM code at 0x114 (LDR PC,=0xB8000804) will jump
                             * to the flash window, run PLL+SDRAM cal, then continue
                             * with init BLs → scheduler → full µMORE boot. */
                            /* Don't modify PC — let ROM continue from after the STR */
                            printf("[ROM-PRELOAD] Natural boot (no skip) — SDRAM should auto-pass\n");
                        }
                    }
                }
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
                        printf("[CDROM-DMA] LBA=%u cnt=%u dst=0x%08X phase=%d\n",
                               lba, count, dst, vf->boot_phase);

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

        /* GPIO write at 0x90000000 — silently accept */
        if (off >= 0x10000000u && off < 0x10001000u) return;

        /* Event controller / UART write at 0x90020000 */
        if (off >= 0x10020000u && off < 0x10080000u) { /* event controller banks 0-5 */
            uint32_t ureg = (off - 0x10020000u) & 0xFFFF;
            if (ureg == 0x00) {
                char c = (char)(val & 0xFF);
                if (c >= 0x20 || c == '\n')
                    fprintf(stderr, "%c", c);
            }
            /* Any write to event controller clears the event flag */
            if (ureg >= 0x08 && ureg <= 0x18)
                *(uint32_t*)(vf->ram + 0xFFE100) = 0;
            return;
        }

        /* Watchdog write at 0x90060000 — silently accept */
        if (off >= 0x10060000u && off < 0x10061000u) return;

        /* RTC write at 0x90090000 — store scratch registers */
        if (off >= 0x10090000u && off < 0x10091000u) {
            uint32_t rreg = off - 0x10090000u;
            /* Store ALL RTC/scratch writes (µMORE uses 0x8C for FIQ handler flag) */
            if ((rreg >> 2) < 64)
                vf->rtc_regs[rreg >> 2] = val;
            return;
        }

        /* Misc System Control write at 0x900A0000 — store all writes */
        if (off >= 0x100A0000u && off < 0x100B0000u) {
            uint32_t sreg = off - 0x100A0000u;
            if ((sreg >> 2) < 64)
                vf->misc_regs[sreg >> 2] = val;

            /* 0x900A0008: system reset trigger.
             * ROM callback at 0x1880 writes 1 here to reboot.
             * Simulate warm reboot: set warm boot flag (bit1 of
             * boot status at 0x900A000C) and restart from ROM addr 0. */
            if (sreg == 0x08 && (val == 1 || val == 2) && vf->has_rom) {
                vf->atapi.reboot_count++;
                printf("[REBOOT] Warm reboot #%d triggered (write 1 to 0x900A0008)\n",
                       vf->atapi.reboot_count);

                /* Log µMORE kernel state in RAM */
                printf("[REBOOT] IRQ handler RAM[0x873D0] = 0x%08X (expect 0x1A000002)\n",
                       *(uint32_t*)(vf->ram + 0x873D0));
                printf("[REBOOT] µMORE data RAM[0xACC78] = 0x%08X (task ptr)\n",
                       *(uint32_t*)(vf->ram + 0xACC78));
                /* Log what BOOT.BIN init wrote to key RAM locations */
                printf("[REBOOT] RAM[0xFFC8] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFFC8),
                       *(uint32_t*)(vf->ram + 0xFFCC),
                       *(uint32_t*)(vf->ram + 0xFFD0),
                       *(uint32_t*)(vf->ram + 0xFFD4));
                printf("[REBOOT] RAM[0xFF80] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFF80),
                       *(uint32_t*)(vf->ram + 0xFF84),
                       *(uint32_t*)(vf->ram + 0xFF88),
                       *(uint32_t*)(vf->ram + 0xFF8C));
                printf("[REBOOT] RAM[0xFF90] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFF90),
                       *(uint32_t*)(vf->ram + 0xFF94),
                       *(uint32_t*)(vf->ram + 0xFF98),
                       *(uint32_t*)(vf->ram + 0xFF9C));
                printf("[REBOOT] RAM[0xFFC0] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFFC0),
                       *(uint32_t*)(vf->ram + 0xFFC4),
                       *(uint32_t*)(vf->ram + 0xFFC8),
                       *(uint32_t*)(vf->ram + 0xFFCC));
                printf("[REBOOT] RAM[0xFFD0] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFFD0),
                       *(uint32_t*)(vf->ram + 0xFFD4),
                       *(uint32_t*)(vf->ram + 0xFFD8),
                       *(uint32_t*)(vf->ram + 0xFFDC));
                printf("[REBOOT] RAM[0xFFE0] = 0x%08X 0x%08X 0x%08X 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xFFE0),
                       *(uint32_t*)(vf->ram + 0xFFE4),
                       *(uint32_t*)(vf->ram + 0xFFE8),
                       *(uint32_t*)(vf->ram + 0xFFEC));
                printf("[REBOOT] BOOT[0x20] callback = 0x%08X\n",
                       *(uint32_t*)(vf->ram + 0xC00020));

                if (vf->atapi.reboot_count > 5) {
                    printf("[REBOOT] Too many reboots, stopping reboot loop\n");
                    return;
                }

                if (vf->atapi.reboot_count == 1) {
                    /* Copy ROM to RAM so µMORE kernel code is accessible.
                     * On real HW, BOOT.BIN init copies ROM to RAM via DMA.
                     * Without this, RAM[0x873D0] (IRQ handler) is all zeros. */
                    if (vf->rom && vf->rom_size > 0) {
                        uint32_t copy_sz = vf->rom_size;
                        if (copy_sz > VFLASH_RAM_SIZE) copy_sz = VFLASH_RAM_SIZE;
                        for (uint32_t i = 0; i < copy_sz; i += 4) {
                            uint32_t rv = *(uint32_t*)(vf->rom + i);
                            uint32_t mv = *(uint32_t*)(vf->ram + i);
                            if (mv == 0 && rv != 0)
                                *(uint32_t*)(vf->ram + i) = rv;
                        }
                        printf("[REBOOT] Copied ROM → RAM (%u KB, merge)\n", copy_sz / 1024);
                    }

                    /* First reboot: BOOT.BIN init has set up µMORE vectors and
                     * handler addresses at RAM[0xFF80-0xFFE4].
                     * ROM warm boot will jump to RAM[0xFFC8] (currently B .).
                     * Patch it to enable IRQs first, so µMORE's IRQ handler
                     * at 0x100873D0 can drive the scheduler. */

                    /* Write idle stub at high RAM address (0xFFF000) where
                     * BOOT.BIN init won't overwrite it. */
                    uint32_t p = 0xFFF000;
                    *(uint32_t*)(vf->ram + p) = 0xE10F0000; p += 4; /* MRS R0, CPSR */
                    *(uint32_t*)(vf->ram + p) = 0xE3C000C0; p += 4; /* BIC R0, #0xC0 */
                    *(uint32_t*)(vf->ram + p) = 0xE129F000; p += 4; /* MSR CPSR_cxsf, R0 */
                    *(uint32_t*)(vf->ram + p) = 0xEAFFFFFE; p += 4; /* B . */

                    /* Patch warm boot and callback to our high-address stub */
                    {
                        uint32_t wb = 0xFFC8;
                        *(uint32_t*)(vf->ram + wb + 0) = 0xE51FF004; /* LDR PC,[PC,-#4] */
                        *(uint32_t*)(vf->ram + wb + 4) = 0x10FFF000; /* idle stub addr */
                    }
                    printf("[REBOOT] Patched idle → 0x10FFF000\n");

                    /* Install proper IRQ wrapper at RAM[0xFF40].
                     * Saves regs, calls µMORE handler, clears timer IRQ,
                     * restores regs, returns from IRQ (SUBS PC,LR,#4). */
                    /* IRQ wrapper at high address (0xFFF040) */
                    {
                        uint32_t w = 0xFFF040;
                        *(uint32_t*)(vf->ram + w) = 0xE92D500F; w += 4; /* PUSH */
                        *(uint32_t*)(vf->ram + w) = 0xE3A00001; w += 4; /* MOV R0, #1 */
                        *(uint32_t*)(vf->ram + w) = 0xE3A01000; w += 4; /* MOV R1, #0 */
                        int32_t bl_off = (int32_t)(0x872A4 - (w + 8)) >> 2;
                        *(uint32_t*)(vf->ram + w) = 0xEB000000 | (bl_off & 0x00FFFFFF); w += 4;
                        *(uint32_t*)(vf->ram + w) = 0xE59F000C; w += 4; /* LDR R0, [PC,#12] */
                        *(uint32_t*)(vf->ram + w) = 0xE3A01001; w += 4; /* MOV R1, #1 */
                        *(uint32_t*)(vf->ram + w) = 0xE5801000; w += 4; /* STR R1, [R0] */
                        *(uint32_t*)(vf->ram + w) = 0xE8BD500F; w += 4; /* POP */
                        *(uint32_t*)(vf->ram + w) = 0xE25EF004; w += 4; /* SUBS PC, LR, #4 */
                        *(uint32_t*)(vf->ram + w) = 0x9001000C; w += 4; /* pool: timer IntClr */
                    }
                    printf("[REBOOT] IRQ wrapper at 0x10FFF040\n");

                    /* After warm boot, run BOOT.BIN.
                     * First reboot: let original callback (0x1880) run → triggers 2nd reboot.
                     * Second reboot: patch callback to idle (prevent loop).
                     * On 2nd run, BOOT.BIN should enter game mode (µMORE initialized). */
                    if (vf->atapi.reboot_count >= 2) {
                        /* Safe stub for callback */
                        *(uint32_t*)(vf->ram + 0xFFE080) = 0xE3A00000;
                        *(uint32_t*)(vf->ram + 0xFFE084) = 0xE12FFF1E;
                        *(uint32_t*)(vf->ram + 0xC00020) = 0x10FFE080;
                    }

                    /* Set warm boot entry to BOOT.BIN instead of idle loop */
                    {
                        uint32_t wb = 0xFFC8;
                        *(uint32_t*)(vf->ram + wb + 0) = 0xE51FF004; /* LDR PC,[PC,-#4] */
                        *(uint32_t*)(vf->ram + wb + 4) = 0x10C00010; /* BOOT.BIN entry */
                    }
                    printf("[REBOOT] Warm boot → BOOT.BIN init (callback patched to idle)\n");

                    /* Set up SP804 timer for periodic IRQs */
                    vf->timer.timer[0].load = 37500;
                    vf->timer.timer[0].count = 37500;
                    vf->timer.timer[0].ctrl = 0xE2;
                    vf->timer.irq.enable |= 0x01;
                    printf("[REBOOT] Timer0 configured: load=%u\n", vf->timer.timer[0].load);
                }

                if (vf->atapi.reboot_count > 5) {
                    printf("[REBOOT] Too many reboots, stopping\n");
                    return;
                }

                if (vf->atapi.reboot_count == 2) {
                    /* Reboot #2: SDRAM vectors at 0xFF80+ populated by reboot #1.
                     * Re-load BOOT.BIN, run init, then launch service entry. */
                    printf("[REBOOT] Reboot #2: BOOT.BIN init → service entry\n");
                    if (vf->cd && vf->cd->is_open) {
                        CDEntry boot_entry;
                        if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &boot_entry)) {
                            uint32_t dest = 0xC00000;
                            uint32_t max_sz = VFLASH_RAM_SIZE - dest;
                            if (boot_entry.size < max_sz) max_sz = boot_entry.size;
                            cdrom_read_file(vf->cd, &boot_entry,
                                vf->ram + dest, 0, max_sz);
                        }
                    }
                    printf("[REBOOT] Vectors: [0xFF9C]=%08X [0xFFBC]=%08X [0xFFDC]=%08X\n",
                           *(uint32_t*)(vf->ram+0xFF9C), *(uint32_t*)(vf->ram+0xFFBC),
                           *(uint32_t*)(vf->ram+0xFFDC));
                    *(uint32_t*)(vf->ram + 0xC00020) = 0x10FFF000;
                    {
                        uint32_t ttb_off = 0xA8000;
                        for (uint32_t mb = 0x100; mb <= 0x10F; mb++) {
                            uint32_t *l1 = (uint32_t*)(vf->ram + ttb_off + mb*4);
                            if ((*l1 & 3) == 0)
                                *l1 = (mb << 20) | 0xC0E;
                        }
                    }
                    vf->timer.timer[0].ctrl = 0x10;
                    vf->timer.timer[0].irq_pending = 0;
                    vf->timer.irq.status = 0;
                    vf->cpu.cp15.ttb = 0x100A8000;
                    vf->cpu.cp15.mmu_enabled = 1;
                    vf->cpu.cp15.control = 0x0000507D;
                    tlb_flush();
                    vf->cpu.null_trap_enabled = 1;
                    vf->cpu.cpsr = 0x000000D3;
                    vf->cpu.r[15] = 0x10C0011C;
                    vf->cpu.r[13] = 0x10FFE000;
                    vf->cpu.r[14] = 0x10FFF000;
                    vf->boot_phase = 100;
                    printf("[REBOOT] → BOOT.BIN init at 0x10C0011C\n");
                    return;
                }
                if (vf->atapi.reboot_count >= 3) {
                    printf("[REBOOT] Reboot #%d: checking task registrations\n",
                           vf->atapi.reboot_count);
                    printf("[REBOOT] Task list: [0x3585C0]=%08X sched_state: [0x3585E0]=%08X\n",
                           *(uint32_t*)(vf->ram+0x3585C0), *(uint32_t*)(vf->ram+0x3585E0));
                    printf("[REBOOT] Task mgr: [0xACC78]=%08X\n",
                           *(uint32_t*)(vf->ram+0xACC78));
                    /* Scan for task structs ($QCB magic = 0x42435124) */
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        if (*(uint32_t*)(vf->ram + a) == 0x42435124) {
                            uint32_t pc = *(uint32_t*)(vf->ram + a + 0x58);
                            uint32_t sp = *(uint32_t*)(vf->ram + a + 0x60);
                            printf("[REBOOT] Found $QCB at 0x%08X: saved_PC=%08X saved_SP=%08X\n",
                                   0x10000000+a, pc, sp);
                        }
                    }
                }

                /* Reboot #1: let ROM run from 0 with warm boot flag */
                vf->misc_regs[0x0C >> 2] |= 0x02;
                arm9_reset(&vf->cpu);
                vf->timer.timer[0].ctrl = 0x10;
                vf->timer.timer[0].irq_pending = 0;
                vf->timer.irq.status = 0;
                vf->timer.irq.fiq_sel = 0;
                vf->cpu.r[15] = 0x00000000;
                vf->cpu.cpsr = 0x000000D3;
                printf("[REBOOT] → ROM at 0x00000000 (warm boot flag set)\n");
            }
            return;
        }

        /* PMU write at 0x900B0000 — store and handle clock changes */
        if (off >= 0x100B0000u && off < 0x100B1000u) {
            uint32_t preg = off - 0x100B0000u;
            if ((preg >> 2) < 16)
                vf->pmu_regs[preg >> 2] = val;
            /* Writing to 0x900B000C triggers clock change (Nspire pmu_write) */
            if (preg == 0x0C && (val & 4)) {
                /* Clock change complete — raise power IRQ like Nspire */
                vf->pmu_regs[0x14 >> 2] |= 1; /* Set PLL lock bit */
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
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped) return;
        if (vf->rom_remapped && addr + 1 < sizeof(vf->low_ram))
            { *(uint16_t*)(vf->low_ram + addr) = val; return; }
        if (addr + 1 < VFLASH_RAM_SIZE) *(uint16_t*)(vf->ram + addr) = val;
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
    if (addr < 0x00200000u) {
        if (vf->has_rom && !vf->rom_remapped) return;
        if (vf->rom_remapped && addr < sizeof(vf->low_ram))
            { vf->low_ram[addr] = val; return; }
        if (addr < VFLASH_RAM_SIZE) vf->ram[addr] = val;
        return;
    }
    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        uint32_t roff8 = addr - VFLASH_RAM_BASE;
        /* Block game_mode byte write to 4 (error mode) */
        if (roff8 == 0xB902C0 && val == 4) val = 3;
        vf->ram[roff8] = val;
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
        if (stubs[i].fatal) {
            /* Fatal: infinite loop */
            ram_w32(vf, base +  0, 0xEAFFFFFEu);      /* B . (loop) */
            ram_w32(vf, base +  4, 0xEAFFFFFEu);
            ram_w32(vf, base +  8, 0xEAFFFFFEu);
            ram_w32(vf, base + 12, 0xEAFFFFFEu);
            ram_w32(vf, base + 16, 0xEAFFFFFEu);
        } else if (i == 5) {
            /* IRQ handler: clear timer, call task callback, return.
             * Uses a larger stub area (40 bytes = 10 words).
             *
             * STMFD SP!, {R0-R3, R12, LR}   ; save caller-saved regs
             * LDR R0, [PC, #+0x14]           ; R0 = 0x9001000C (timer IntClr)
             * MOV R1, #1
             * STR R1, [R0]                    ; clear timer IRQ
             * LDR R0, [PC, #+0x10]           ; R0 = task callback addr
             * CMP R0, #0
             * BLXNE R0                        ; call callback if non-zero
             * LDMFD SP!, {R0-R3, R12, LR}    ; restore regs
             * SUBS PC, LR, #4                ; return from IRQ
             * .word 0x9001000C               ; timer IntClr
             * .word <callback>               ; task #0 callback
             *
             * Note: we write callback address AFTER loading BOOT.BIN
             * in the HLE boot setup. For now, write 0 (no callback). */
            uint32_t irq_base = HLE_STUB_BASE + 5 * 20;
            ram_w32(vf, irq_base +  0, 0xE92D500Fu);  /* PUSH {R0-R3,R12,LR} */
            ram_w32(vf, irq_base +  4, 0xE59F0014u);  /* LDR R0, [PC, #+0x14] → +20 */
            ram_w32(vf, irq_base +  8, 0xE3A01001u);  /* MOV R1, #1 */
            ram_w32(vf, irq_base + 12, 0xE5801000u);  /* STR R1, [R0] */
            ram_w32(vf, irq_base + 16, 0xE59F0010u);  /* LDR R0, [PC, #+0x10] → +24 */
            ram_w32(vf, irq_base + 20, 0xE3500000u);  /* CMP R0, #0 */
            ram_w32(vf, irq_base + 24, 0x112FFF30u);  /* BLXNE R0 */
            ram_w32(vf, irq_base + 28, 0xE8BD500Fu);  /* POP {R0-R3,R12,LR} */
            ram_w32(vf, irq_base + 32, 0xE25EF004u);  /* SUBS PC, LR, #4 */
            ram_w32(vf, irq_base + 36, 0x9001000Cu);  /* data: timer IntClr */
            ram_w32(vf, irq_base + 40, 0x00000000u);  /* data: callback (patched later) */
            printf("[HLE] IRQ handler at 0x%08X (with task callback slot at +40)\n", irq_base);
            /* Skip the normal stub write */
            continue;
        } else {
            /* SWI/FIQ/RESET: MOVS PC, LR (return from exception) */
            ram_w32(vf, base +  0, 0xE1B0F00Eu);      /* MOVS PC, LR */
            ram_w32(vf, base +  4, 0xE1A00000u);      /* NOP */
            ram_w32(vf, base +  8, 0xE1A00000u);
            ram_w32(vf, base + 12, 0xE1A00000u);
            ram_w32(vf, base + 16, 0xE1A00000u);
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

                    /* Install ROM REL function stub at address 0x1880.
                     * All 6 tested games have [BOOT+0x20] = 0x1880 as the ROM callback.
                     * The real ROM function at 0x1880:
                     *   1. Writes PLL/SDRAM config to I/O registers
                     *   2. Writes 1 to 0x900A0008 (system reset → warm reboot)
                     *   3. Infinite loop (never returns)
                     *
                     * In HLE mode, addr 0 is RAM. Our stub does the I/O config
                     * writes, then instead of rebooting, enables IRQs and enters
                     * WFI loop. BOOT.BIN's three init functions (0x10C04368,
                     * 0x10C044A0, 0x10C043A0) already set up µMORE state before
                     * calling 0x1880, so the timer IRQ should wake the scheduler.
                     *
                     * Stub at 0x1880:
                     *   LDR  R4, =0x900B0000    ; PMU clocks
                     *   MOV  R5, #3
                     *   STR  R5, [R4]            ; write clock config
                     *   MRS  R0, CPSR
                     *   BIC  R0, R0, #0xC0       ; enable IRQ+FIQ
                     *   MSR  CPSR_c, R0
                     * .loop:
                     *   MCR  p15, 0, R0, c7, c0, 4  ; WFI (wait for interrupt)
                     *   B    .loop
                     *   .word 0x900B0000          ; data pool
                     */
                    /* ROM stub at 0x1880: PLL config, enable IRQ, idle loop.
                     * The WFI + branch loop must be robust: IRQ returns to
                     * LR_irq-4 which should land back in the loop.
                     * Layout: config code, then NOP sled + branch back. */
                    /* ROM stub at 0x1880:
                     * 1. Enable IRQs
                     * 2. Call VIC/task init at load_addr+0xC4A4 (programs VIC from task table)
                     * 3. Idle loop with timer IRQ handling
                     *
                     * +00: MRS R0, CPSR
                     * +04: BIC R0, R0, #0xC0       ; enable IRQ+FIQ
                     * +08: MSR CPSR_c, R0
                     * +0C: LDR LR, [PC, #+0x1C]    ; LR = idle_loop addr (+18)
                     * +10: LDR PC, [PC, #+0x1C]    ; jump to VIC init
                     * +14: NOP
                     * +18: NOP (idle loop start)
                     * +1C: NOP
                     * +20: NOP
                     * +24: NOP
                     * +28: B +18
                     * +2C: NOP
                     * +30: idle_loop addr (0x1898)  ; data for LR load
                     * +34: VIC init addr            ; data for PC load
                     */
                    uint32_t idle_loop = 0x1898;
                    uint32_t vic_init = load_addr + 0xC4A4;
                    uint32_t stub[] = {
                        0xE10F0000,   /* +00: MRS R0, CPSR */
                        0xE3C000C0,   /* +04: BIC R0, R0, #0xC0 */
                        0xE121F000,   /* +08: MSR CPSR_c, R0 */
                        0xE59FE01C,   /* +0C: LDR LR, [PC, #+0x1C] → +30 */
                        0xE59FF01C,   /* +10: LDR PC, [PC, #+0x1C] → +34 */
                        0xE1A00000,   /* +14: NOP */
                        0xE1A00000,   /* +18: NOP (idle loop) */
                        0xE1A00000,   /* +1C: NOP */
                        0xE1A00000,   /* +20: NOP */
                        0xE1A00000,   /* +24: NOP */
                        0xEAFFFFFA,   /* +28: B -6 → +18 */
                        0xE1A00000,   /* +2C: NOP */
                        idle_loop,    /* +30: data: idle loop addr */
                        vic_init,     /* +34: data: VIC init addr */
                    };
                    for (int i = 0; i < 14; i++)
                        *(uint32_t*)(vf->ram + 0x1880 + i * 4) = stub[i];
                    printf("[HLE] Installed ROM stub at 0x1880 (IRQ enable + VIC init + idle)\n");

                    /* Populate µMORE task table so VIC init has something to dispatch.
                     * Task table at 0x10B0DF00, 16 bytes per entry, up to 30 entries.
                     * Entry: +0=IRQ_addr/mask, +4=priority, +8=callback, +C=active.
                     * Set task #0 as timer IRQ handler (callback = VIC init addr). */
                    {
                        uint32_t base = load_addr - VFLASH_RAM_BASE;
                        /* Set task count to 1 */
                        *(uint32_t*)(vf->ram + base + 0xC004) = 1;

                        /* Task #0 entry at 0x10B0DF00 */
                        uint32_t task0 = 0xB0DF00;  /* RAM offset */
                        *(uint32_t*)(vf->ram + task0 + 0x00) = 0x00000001; /* IRQ mask: bit 0 (timer) */
                        *(uint32_t*)(vf->ram + task0 + 0x04) = 0;
                        *(uint32_t*)(vf->ram + task0 + 0x08) = load_addr + 0xC4A4; /* callback = VIC init */
                        *(uint32_t*)(vf->ram + task0 + 0x0C) = 1; /* active */
                        printf("[HLE] Task #0: mask=0x1 callback=0x%08X\n", load_addr + 0xC4A4);

                        /* IRQ callback = 0 (just clear timer, no dispatch).
                         * Game code will be called directly from ROM stub. */
                    }
                    /* Verify stub */
                    for (int si = 0; si < 13; si++) {
                        printf("[HLE]   [0x%04X] = 0x%08X\n", 0x1880+si*4,
                               *(uint32_t*)(vf->ram + 0x1880 + si*4));
                    }

                    /* Also set up SP804 timer to fire periodic IRQ.
                     * µMORE scheduler needs timer ticks. */
                    vf->timer.timer[0].load = 37500;  /* ~60Hz at 2.25MHz */
                    vf->timer.timer[0].count = 37500;
                    vf->timer.timer[0].ctrl = 0xE2;   /* enable, periodic, 32-bit, IntEnable */
                    printf("[HLE] SP804 timer0: load=%u ctrl=0x%02X (60Hz periodic)\n",
                           vf->timer.timer[0].load, vf->timer.timer[0].ctrl);

                    /* Set VIC to enable timer IRQ */
                    vf->timer.irq.enable = (1 << IRQ_TIMER0);
                    printf("[HLE] VIC IRQ enable: 0x%08X\n", vf->timer.irq.enable);

                    /* VIDEO TEST: Load first MJP frame from disc and display it.
                     * This proves the video pipeline works while µMORE scheduler
                     * is still being investigated. Searches for any .MJP file
                     * on disc and decodes the first JPEG frame. */
                    {
                        CDEntry mjp_entry;
                        static const char *mjp_names[] = {
                            "MAIN01.MJP", "MAIN.MJP", "CUTSCENE01.MJP", NULL
                        };
                        int mjp_found = 0;
                        for (int mi = 0; mjp_names[mi] && !mjp_found; mi++)
                            mjp_found = cdrom_find_file_any(vf->cd, mjp_names[mi], &mjp_entry);

                        if (mjp_found && mjp_entry.size > 100) {
                            printf("[HLE-VIDEO] Found MJP: %s (%u bytes)\n",
                                   mjp_entry.name, mjp_entry.size);
                            /* Read first 256KB to get the full first JPEG frame */
                            uint32_t read_sz = mjp_entry.size < 262144 ? mjp_entry.size : 262144;
                            uint8_t *mjpbuf = malloc(read_sz);
                            if (mjpbuf) {
                                int rd = cdrom_read_file(vf->cd, &mjp_entry, mjpbuf, 0, read_sz);
                                if (rd > 76) {
                                    /* MJP header: skip first 76 bytes, then byteswap JPEG data.
                                     * V.Flash MJP = MIAV container with byte-swapped JPEG frames. */
                                    uint8_t *jpeg = mjpbuf + 76;
                                    uint32_t jpeg_sz = rd - 76;

                                    /* Byteswap pairs */
                                    for (uint32_t bi = 0; bi + 1 < jpeg_sz; bi += 2) {
                                        uint8_t tmp = jpeg[bi];
                                        jpeg[bi] = jpeg[bi+1];
                                        jpeg[bi+1] = tmp;
                                    }

                                    /* Find JPEG SOI (FFD8) and EOI (FFD9) markers */
                                    int soi = -1, eoi = -1;
                                    for (uint32_t bi = 0; bi + 1 < jpeg_sz; bi++) {
                                        if (jpeg[bi] == 0xFF && jpeg[bi+1] == 0xD8 && soi < 0) soi = bi;
                                        if (jpeg[bi] == 0xFF && jpeg[bi+1] == 0xD9 && soi >= 0) { eoi = bi; break; }
                                    }

                                    if (soi >= 0) {
                                        uint8_t *jstart = jpeg + soi;
                                        uint32_t jlen = jpeg_sz - soi;
                                        /* Find SOS for byte-stuffing */
                                        uint32_t sos_data = jlen;
                                        uint32_t hp = 2;
                                        while (hp + 3 < jlen && jstart[hp] == 0xFF) {
                                            uint8_t m = jstart[hp+1];
                                            uint16_t mlen = ((uint16_t)jstart[hp+2]<<8)|jstart[hp+3];
                                            if (m == 0xDA) { sos_data = hp + 2 + mlen; break; }
                                            hp += 2 + mlen;
                                        }
                                        /* Re-insert byte-stuffing in entropy data */
                                        uint8_t *fixed = malloc(jlen + jlen/4 + 16);
                                        uint32_t fi = 0;
                                        memcpy(fixed, jstart, sos_data);
                                        fi = sos_data;
                                        for (uint32_t ei = sos_data; ei < jlen; ei++) {
                                            fixed[fi++] = jstart[ei];
                                            if (jstart[ei] == 0xFF && ei+1 < jlen) {
                                                uint8_t nxt = jstart[ei+1];
                                                if (nxt==0x00||(nxt>=0xD0&&nxt<=0xD7)||nxt==0xD9)
                                                    { fixed[fi++]=nxt; ei++; }
                                                else { fixed[fi++]=0x00; }
                                            }
                                        }
                                        if (fi<2||fixed[fi-2]!=0xFF||fixed[fi-1]!=0xD9)
                                            { fixed[fi++]=0xFF; fixed[fi++]=0xD9; }
                                        printf("[HLE-VIDEO] JPEG: raw=%u stuffed=%u\n", jlen, fi);
                                        if (mjp_decode_frame(vf->video, fixed, fi)) {
                                            memcpy(vf->framebuf, mjp_get_framebuf(vf->video),
                                                   VFLASH_SCREEN_W * VFLASH_SCREEN_H * 4);
                                            vf->vid.fb_dirty = 1;
                                            printf("[HLE-VIDEO] First frame decoded and displayed!\n");
                                        } else {
                                            printf("[HLE-VIDEO] JPEG decode failed\n");
                                        }
                                        free(fixed);
                                    } else {
                                        printf("[HLE-VIDEO] No JPEG SOI marker found in MJP data\n");
                                    }
                                }
                                free(mjpbuf);
                            }
                        } else {
                            printf("[HLE-VIDEO] No MJP file found on disc\n");
                        }
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
    /* Note: mmu_dump_pagetable available for debugging */
    ztimer_reset(&vf->timer);

    vf->cpu.mem_ctx      = vf;
    vf->cpu.mem_read32   = mem_read32;
    vf->cpu.mem_read16   = mem_read16;
    vf->cpu.mem_read8    = mem_read8;
    vf->cpu.mem_write32  = mem_write32;
    vf->cpu.mem_write16  = mem_write16;
    vf->cpu.mem_write8   = mem_write8;
    vf->cpu.undef_callback = undef_rom_copy;
    vf->cpu.hle_intercept  = hle_service_intercept;
    vf->cpu.hle_ctx        = vf;

    /* Initialize JIT compiler */
    vf->jit = jit_create(vf);

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
                /* DON'T load ROM to RAM[0]! On real HW, addr 0 is ROM (read-only)
                 * during boot, then remaps to SDRAM (zeroed). Our RAM[0] = SDRAM.
                 * ROM content is accessible via 0xB8000000 (flash controller).
                 * Boot code reads from addr 0 (ROM mirror) — we handle this
                 * by checking if addr < rom_size in mem_read. */
                uint32_t boot_size = 0; /* Don't copy ROM to RAM */
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

    atapi_init(&vf->atapi);

    printf("[VFlash] System ready\n");
    return vf;
}

void vflash_destroy(VFlash *vf) {
    if (!vf) return;
    cdrom_destroy(vf->cd);
    mjp_destroy(vf->video);
    audio_destroy(vf->audio);
    free(vf->atapi.data_buf);
    free(vf->ram);
    free(vf->mjp_player.data);
    free(vf->mjp_player.hdr);
    free(vf->sram);
    free(vf->rom);
    free(vf->ptx_list);
    free(vf->wav_list);
    free(vf->mjp_list);
    free(vf);
}

void vflash_init_audio(VFlash *vf) {
    audio_init_sdl(vf->audio);
}

void vflash_set_debug(VFlash *vf, int on) {
    vf->debug = on;
}

/* Play a WAV file from CD-ROM through the audio system */
static void play_wav_from_cd(VFlash *vf, CDEntry *entry) {
    if (!vf->audio || !entry || entry->size < 44) return;
    uint8_t *wav = malloc(entry->size);
    if (!wav) return;
    int rd = cdrom_read_file(vf->cd, entry, wav, 0, entry->size);
    if (rd > 44) {
        int16_t *samples = NULL;
        uint32_t count = 0, rate = 0;
        if (snd_decode(wav, (uint32_t)rd, &samples, &count, &rate)) {
            /* Resample to 44100 stereo if needed */
            if (rate == 22050) {
                /* Upsample 2x and mono→stereo */
                uint32_t out_count = count * 4; /* 2x rate * 2 channels */
                int16_t *out = malloc(out_count * sizeof(int16_t));
                if (out) {
                    for (uint32_t i = 0; i < count; i++) {
                        int16_t s = samples[i];
                        out[i*4+0] = s; out[i*4+1] = s; /* L,R */
                        out[i*4+2] = s; out[i*4+3] = s; /* L,R (dup) */
                    }
                    audio_push_samples(vf->audio, out, out_count);
                    free(out);
                }
            } else {
                audio_push_samples(vf->audio, samples, count);
            }
            free(samples);
        }
    }
    free(wav);
}

/* ============================================================
 * HLE µMORE service layer
 * Game dispatch table at 0x10C13854 routes to 8 BSS addresses.
 * We intercept execution at these addresses and handle the calls.
 * ============================================================ */

/* µMORE service addresses (from BOOT.BIN jump table at 0x10C1385C) */
#define UMORE_SVC_0  0x109A0868
#define UMORE_SVC_1  0x109A0950  /* most frequently called */
#define UMORE_SVC_2  0x109A0970
#define UMORE_SVC_3  0x109A0A38
#define UMORE_SVC_4  0x109A0A0C
#define UMORE_SVC_5  0x109A0A1C
#define UMORE_SVC_6  0x109A09AC
#define UMORE_SVC_7  0x109A09DC

static int hle_service_intercept(void *ctx, uint32_t addr) {
    VFlash *vf = ctx;
    ARM9 *cpu = &vf->cpu;

    /* Capture palette: if render code hits tile LUT functions,
     * R5/R6 holds the 32-bit ARGB palette pointer (256 entries × 4 bytes). */
    if ((addr == 0x10A26470 || addr == 0x10A2687C || addr == 0x10A26898) &&
        vf->lcd.pal_written == 0) {
        uint32_t pal_ptr = cpu->r[5];
        if (addr != 0x10A26470) pal_ptr = cpu->r[6]; /* R6 for 0x10A2687C/898 */
        if (pal_ptr >= 0x10000000 && pal_ptr + 1024 <= 0x10000000 + VFLASH_RAM_SIZE) {
            uint32_t p_off = pal_ptr - 0x10000000;
            printf("[PALETTE] Captured at 0x10A26470: R5=0x%08X\n", pal_ptr);
            /* Convert ARGB32 palette to XBGR1555 for PL111 */
            for (int i = 0; i < 256; i++) {
                uint32_t c = *(uint32_t*)(vf->ram + p_off + i*4);
                uint8_t r = (c >> 16) & 0xFF;
                uint8_t g = (c >> 8) & 0xFF;
                uint8_t b = c & 0xFF;
                vf->lcd.palette[i] = ((r >> 3) & 0x1F) |
                                     (((g >> 3) & 0x1F) << 5) |
                                     (((b >> 3) & 0x1F) << 10);
            }
            vf->lcd.pal_written = 256;
            printf("[PALETTE] e0=0x%08X e1=0x%08X e18=0x%08X e212=0x%08X\n",
                   *(uint32_t*)(vf->ram+p_off),
                   *(uint32_t*)(vf->ram+p_off+4),
                   *(uint32_t*)(vf->ram+p_off+18*4),
                   *(uint32_t*)(vf->ram+p_off+212*4));
        }
    }

    /* Render budget: decrement FIRST before any other intercepts. */
    if (vf->render_budget > 0) {
        vf->render_budget--;
        if (vf->render_budget == 0 &&
            addr >= 0x10A00000 && addr < 0x10C00000) {
            static int budget_log = 0;
            if (budget_log < 5)
                printf("[RENDER-BUDGET] Expired at PC=%08X, forcing return\n", addr);
            budget_log++;
            cpu->r[0] = 0;
            cpu->r[15] = 0x109D1CEC;
            cpu->r[13] = 0x10B8DA90;
            return 1;
        }
    }

    /* Intercept ANY zero-instruction call in µMORE service BSS range
     * (0x109A0000-0x109B0000) and nearby areas. Also catch the specific
     * dispatch table addresses and second-level dispatch addresses. */
    /* µMORE service handlers — log parameters to understand what game wants */
    {
        static const uint32_t svc_addrs[] = {
            0x109A0868, 0x109A0950, 0x109A0970, 0x109A0A38,
            0x109A0A0C, 0x109A0A1C, 0x109A09AC, 0x109A09DC,
            0x109A0A98, 0x109A0AC4, 0x109A0D2C, 0x109A0CE4, 0x109A0D08, 0
        };
        for (int si = 0; svc_addrs[si]; si++) {
            if (addr == svc_addrs[si]) {
                static int svc_log_counts[16] = {0};
                if (svc_log_counts[si] < 5) {
                    /* Read stack args too: [SP], [SP+4], [SP+8], [SP+12] */
                    uint32_t sp = cpu->r[13];
                    uint32_t s0 = cpu->mem_read32(cpu->mem_ctx, sp);
                    uint32_t s1 = cpu->mem_read32(cpu->mem_ctx, sp + 4);
                    uint32_t s2 = cpu->mem_read32(cpu->mem_ctx, sp + 8);
                    printf("[SVC-%02d] 0x%08X R0=%08X R1=%08X R2=%08X R3=%08X [SP]=%08X,%08X,%08X LR=%08X\n",
                           si, addr, cpu->r[0], cpu->r[1], cpu->r[2], cpu->r[3],
                           s0, s1, s2, cpu->r[14]);
                    svc_log_counts[si]++;
                }
                cpu->r[0] = 1;
                cpu->r[15] = cpu->r[14] & ~3u;
                return 1;
            }
        }
    }

    /* Skip bit7 wait loop after 2nd reboot.
     * BOOT.BIN loops at VA 0x18D4-0x18F0 checking bit7 of a status register.
     * Skip directly to 0x18F4 (code after loop). */
    if (addr == 0x18D4 && ((VFlash*)ctx)->atapi.reboot_count >= 2) {
        static int skip_count = 0;
        if (skip_count == 0)
            printf("[SKIP] Bit7 wait loop at 0x18D4 → skip to 0x18F4\n");
        skip_count++;
        if (skip_count > 100) { /* let it loop a few times first */
            cpu->r[15] = 0x18F4;
            return 1;
        }
    }

    /* Intercept sleep/wait functions.
     * 0x10011D88: static sleep function from ROM.
     * µMORE also generates dynamic event_wait wrappers.
     * Detect stuck-at-address pattern and skip after timeout. */
    if (addr == 0x10011D88) {
        static int sleep_log = 0;
        if (sleep_log < 10)
            printf("[SLEEP] 0x10011D88 from LR=%08X bp=%d\n",
                   cpu->r[14], ((VFlash*)ctx)->boot_phase);
        sleep_log++;
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* Skip µMORE WFI/sleep loops (Ghidra: do{}while(!ZR)) */
    if (addr == 0x10A739EC) {
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* (event loop intercept moved before BSS check) */

    /* (event init skip moved before BSS check) */

    /* Intercept µMORE event dispatch loop (0x10A17100).
     * Game task calls with R0=0, R1=0 (null callback).
     * Inject BOOT.BIN game entry as callback. */
    if (addr == 0x10A17100 && ((VFlash*)ctx)->boot_phase >= 800) {
        static int evl = 0;
        if (evl < 5) {
            printf("[EV-LOOP] R0=%08X R1=%08X LR=%08X\n",
                   cpu->r[0], cpu->r[1], cpu->r[14]);
            evl++;
        }
        if (cpu->r[1] == 0) {
            cpu->r[1] = 0x10CFAEA0;
            printf("[EV-LOOP] Injected game callback!\n");
        }
        return 0;
    }

    /* Skip blocking functions in game task entry.
     * FUN_109d1f28: peripheral wait with sleep loop (blocks forever)
     * FUN_10a18a04: event system init (blocks in event pump) */
    /* Skip blocking µMORE functions in game task entry.
     * Only skip the ones that BLOCK (sleep/wait loops).
     * Let non-blocking init functions run to populate game state. */
    /* Dump game state on first render call */
    /* Force render_ctx[0]=1 when render processing is called.
     * This makes FUN_10a89100 enter processing path instead of returning. */
    /* (budget check moved to top of function) */
    /* Skip RTOS task_notify (10A6FC60) during game phase.
     * Called after render_init and other subsystem inits. Gets stuck in
     * priority tree traversal because RTOS task scheduler not fully active. */
    if (addr == 0x10A6FC60 && ((VFlash*)ctx)->boot_phase >= 900) {
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* HLE software division — replace 82-instruction shift-subtract loop
     * with native C division. ~40x speedup per call.
     * ARM EABI: __aeabi_uidiv(R0, R1) → R0=quotient, R1=remainder
     * Multiple entry points for signed/unsigned/modulo variants. */
    if (addr >= 0x10A86E00 && addr <= 0x10A871C0 &&
        ((VFlash*)ctx)->boot_phase >= 100) {
        uint32_t dividend = cpu->r[0];
        uint32_t divisor = cpu->r[1];
        if (divisor == 0) {
            cpu->r[0] = 0xFFFFFFFF; /* div by zero → max */
            cpu->r[1] = dividend;
        } else if (addr >= 0x10A86E30 && addr < 0x10A86EA0) {
            /* Signed division */
            int32_t sq = (int32_t)dividend / (int32_t)divisor;
            int32_t sr = (int32_t)dividend % (int32_t)divisor;
            cpu->r[0] = (uint32_t)sq;
            cpu->r[1] = (uint32_t)sr;
        } else {
            /* Unsigned division (most common) */
            cpu->r[0] = dividend / divisor;
            cpu->r[1] = dividend % divisor;
        }
        cpu->r[2] = cpu->r[0]; /* some callers read R2 */
        cpu->r[3] = dividend;  /* R3 = original dividend */
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }

    /* HLE byte copy loops — multiple variants of byte-by-byte memcpy.
     * These 7-instruction loops run millions of times during rendering.
     * Replace with native memcpy for ~50x speedup per call. */
    if ((addr == 0x10A4F934 || addr == 0x10A51740 || addr == 0x10A711D0) &&
        ((VFlash*)ctx)->boot_phase >= 100) {
        VFlash *vf2 = (VFlash*)ctx;
        uint32_t src = cpu->r[2] - 0x10000000; /* R2 = source ptr */
        uint32_t dst = cpu->r[12] - 0x10000000; /* R12 = dest ptr */
        uint32_t cnt = cpu->r[4]; /* R4 = count */
        if (cnt > 0 && cnt < 0x100000 &&
            src < VFLASH_RAM_SIZE && dst < VFLASH_RAM_SIZE &&
            src + cnt <= VFLASH_RAM_SIZE && dst + cnt <= VFLASH_RAM_SIZE) {
            memmove(vf2->ram + dst, vf2->ram + src, cnt);
            cpu->r[4] = 0; /* counter = 0 (done) */
            cpu->r[2] = cpu->r[2] + cnt; /* advance src */
            cpu->r[12] = cpu->r[12] + cnt; /* advance dst */
            /* Skip to instruction after the BNE */
            cpu->r[15] = addr + (addr == 0x10A711D0 ? 0x20 : 0x18);
            return 1;
        }
    }
    /* HLE strcmp loop at 10A4DF10: byte-by-byte compare */
    if (addr == 0x10A4DF10 && ((VFlash*)ctx)->boot_phase >= 100) {
        VFlash *vf2 = (VFlash*)ctx;
        uint32_t s1 = cpu->r[0] - 0x10000000;
        uint32_t s2 = cpu->r[1] - 0x10000000;
        if (s1 < VFLASH_RAM_SIZE - 256 && s2 < VFLASH_RAM_SIZE - 256) {
            int result = strncmp((char*)vf2->ram + s1, (char*)vf2->ram + s2, 256);
            cpu->r[0] = result > 0 ? 1 : (result < 0 ? -1 : 0);
            cpu->cpsr = (cpu->cpsr & ~0xF0000000) | (result == 0 ? 0x40000000 :
                         result > 0 ? 0x20000000 : 0x80000000);
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
    }

    /* HLE memcpy at 10A00840: R0=dst, R1=src, R2=len → R0=dst */
    if (addr == 0x10A00840) {
        VFlash *vf2 = (VFlash*)ctx;
        uint32_t dst = cpu->r[0] - 0x10000000;
        uint32_t src = cpu->r[1] - 0x10000000;
        uint32_t len = cpu->r[2];
        if (dst < VFLASH_RAM_SIZE && src < VFLASH_RAM_SIZE &&
            dst + len <= VFLASH_RAM_SIZE && src + len <= VFLASH_RAM_SIZE && len < 0x100000) {
            /* Protect relocated game+render code from BSS clear overwrite */
            if (dst >= 0x9D0000 && dst < 0xB30000) {
                cpu->r[15] = cpu->r[14] & ~3u;
                return 1;
            }
            memmove(vf2->ram + dst, vf2->ram + src, len);
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
    }
    /* HLE memset at 10A4D500: R0=dst, R1=val, R2=len → R0=dst.
     * PROTECT RTOS code area (0x10A00000-0x10B00000) from being zeroed. */
    if (addr == 0x10A4D500) {
        VFlash *vf2 = (VFlash*)ctx;
        uint32_t dst = cpu->r[0] - 0x10000000;
        uint32_t val = cpu->r[1] & 0xFF;
        uint32_t len = cpu->r[2];
        if (dst < VFLASH_RAM_SIZE && dst + len <= VFLASH_RAM_SIZE && len < 0x100000) {
            /* Skip if trying to zero relocated game+render code area */
            if (val == 0 && dst >= 0x9D0000 && dst < 0xB30000) {
                static int prot_log = 0;
                if (prot_log++ < 3)
                    printf("[MEMSET-PROTECT] Blocked zero of RTOS area %08X+%X\n",
                           0x10000000+dst, len);
                cpu->r[15] = cpu->r[14] & ~3u;
                return 1;
            }
            memset(vf2->ram + dst, (int)val, len);
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
    }

    /* HLE render_init (10A881F0): set flags and return.
     * Native run triggers game BSS clear that zeros ALL code. */
    if (addr == 0x10A881F0 && ((VFlash*)ctx)->boot_phase >= 900) {
        VFlash *vf2 = (VFlash*)ctx;
        vf2->ram[0xBE3EA0] = 0;
        vf2->ram[0xBE3C80] = 0;
        vf2->ram[0xBE3E20] = 0;
        vf2->ram[0xBE3EC0] = 1;
        vf2->ram[0xBE3CA0] = 1;
        static int ri_log = 0;
        if (ri_log++ < 3) printf("[HLE-RENDER-INIT] Flags set\n");
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* 10A8CDE4: RTOS tree dispatch — skip (no valid queue state) */
    if (addr == 0x10A8CDE4 && ((VFlash*)ctx)->boot_phase >= 900) {
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* RTOS task dispatch (10A8CDE4): let it run natively.
     * The function checks [R5+8] (queue count) — if 0, returns immediately.
     * We ensure queues stay empty by clearing them after render_init. */
    /* Skip task_notify (10A6FC60) — still needs stub */
    if (addr == 0x10A6FC60 && ((VFlash*)ctx)->boot_phase >= 900) {
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* Render processing (10A89100): software render pipeline.
     * Runs natively but with instruction budget to prevent infinite loops
     * on dummy entity vtable stubs. Budget allows partial rendering. */
    if (addr == 0x10A89100 && ((VFlash*)ctx)->boot_phase >= 900) {
        static int r_log = 0;
        VFlash *vf2 = (VFlash*)ctx;
        if (r_log < 3) r_log++;
        /* Set dirty flag and populate display list */
        *(uint32_t*)(vf2->ram + 0xBE3C40) = 1;
        *(uint32_t*)(vf2->ram + 0xBE3C4C) = 3;
        /* Build display list from HLE alloc entities */
        if (*(uint32_t*)(vf2->ram + 0xBE3D24) == 0) {
            uint32_t arr = 0xB90000;
            int n = 0;
            for (uint32_t ea = 0x320000; ea < 0x390000 && n < 100; ea += 64) {
                if (*(uint32_t*)(vf2->ram+ea) == 0x10310800) {
                    *(uint32_t*)(vf2->ram+arr+n*4) = 0x10000000+ea;
                    n++;
                }
            }
            if (n > 0) {
                *(uint32_t*)(vf2->ram+0xBE3D24) = 0x10000000+arr;
                *(uint32_t*)(vf2->ram+0xBE3D28) = 100;
                *(uint32_t*)(vf2->ram+0xBE3D2C) = 120;
                *(uint32_t*)(vf2->ram+0xBE3D34) = 4;
                *(uint32_t*)(vf2->ram+0xBE3D38) = 0x10000000+arr+n*4;
                *(uint32_t*)(vf2->ram+0xBE3D40) = 0;
                vf2->ram[0xBE4BA0] = 1;
            }
        }
        vf2->render_budget = 50000000; /* 50M — let engine finish a full frame */
        return 0;
    }
    /* (budget check moved to top of function) */
    if (addr == 0x10B265E8 && ((VFlash*)ctx)->boot_phase >= 900) {
        /* Render function — real code copied from BOOT.BIN location.
         * Let it run natively — render_init is HLE'd, render_enable
         * will be set by render_init HLE + native caller code. */
        return 0;
    }


    /* 109D2754 (state machine): force object state to "done" (0x84).
     * The function checks [param_1 + 0x104] for state. States:
     * 0=idle, 1=loading, 2=ready→process, 0x84+=done→return.
     * By setting state=0x84, function returns immediately (already processed). */
    if (addr == 0x109D2754 && ((VFlash*)ctx)->boot_phase >= 800) {
        uint32_t obj = cpu->r[0];
        if (obj >= 0x10000000 && obj + 0x108 < 0x10000000 + VFLASH_RAM_SIZE) {
            *(uint32_t*)(((VFlash*)ctx)->ram + (obj - 0x10000000) + 0x104) = 0x84;
        }
        return 0; /* let it run — will return immediately with state=0x84 */
    }
    if (addr == 0x10A355A4 && ((VFlash*)ctx)->boot_phase >= 800 &&
        (cpu->r[0] < 0x10000000 || cpu->r[0] >= 0x11000000)) {
        static int inv_log = 0;
        if (inv_log < 10) {
            printf("[SKIP-INVPTR] %08X(R0=%08X) — invalid ptr, returning 0\n", addr, cpu->r[0]);
            inv_log++;
        }
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }

    /* Track key pre-loop function calls */
    if (((VFlash*)ctx)->boot_phase >= 900) {
        static int preloop_log = 0;
        if (preloop_log < 20 &&
            (addr == 0x109D5AFC || addr == 0x10A70324 || addr == 0x10A36A94 ||
             addr == 0x109DABD8 || addr == 0x109D26FC || addr == 0x10B263FC ||
             addr == 0x10B265E8 || addr == 0x109D3308)) {
            printf("[PRELOOP] PC=%08X R0=%08X LR=%08X\n", addr, cpu->r[0], cpu->r[14]);
            preloop_log++;
        }
    }

    /* Log vtable stub calls to find which virtual methods render uses */
    if (addr >= 0x10310000 && addr < 0x10310800 &&
        ((VFlash*)ctx)->boot_phase >= 900) {
        int slot = (addr - 0x10310000) / 8;
        static uint32_t stub_count[64] = {0};
        static int total_stubs = 0;
        stub_count[slot]++;
        total_stubs++;
        if (total_stubs <= 5 || (total_stubs % 1000 == 0)) {
            printf("[VSTUB] slot[%d] R0=%08X LR=%08X (total=%d)\n",
                   slot, cpu->r[0], cpu->r[14], total_stubs);
        }
        /* HLE draw: when vtable[2] is called with valid entity ptr,
         * draw a colored marker directly to display framebuffer */
        if (slot == 2 && cpu->r[0] >= 0x10320000 && cpu->r[0] < 0x10380000) {
            VFlash *vf2 = (VFlash*)ctx;
            uint32_t ent = cpu->r[0] - 0x10000000;
            int idx = (ent - 0x320000) / 64;
            int x = (idx * 37) % 300 + 10;
            int y = (idx * 23) % 220 + 10;
            /* Draw colored marker to ARGB32 framebuffer */
            uint32_t color = 0xFFFF0000 | ((idx * 17) & 0xFF) << 8;
            for (int dy = 0; dy < 10 && y+dy < VFLASH_SCREEN_H; dy++)
                for (int dx = 0; dx < 10 && x+dx < VFLASH_SCREEN_W; dx++)
                    vf2->framebuf[(y+dy)*VFLASH_SCREEN_W + (x+dx)] = color;
            vf2->vid.fb_dirty = 1;
        }
        return 0; /* let stub execute */
    }

    /* Init function table walker (109D26FC) — let it run.
     * 109D2754 (state machine) is still skipped to prevent loops.
     * Walker calls 21 functions that initialize render engine layers/state. */

    /* HLE memset/memclear (10A70324).
     * Game calls FUN_10a70324(dst, val, size) to clear 960KB render buffer.
     * Native execution takes many frames — HLE for instant clear. */
    if (addr == 0x10A70324 && ((VFlash*)ctx)->boot_phase >= 900) {
        uint32_t dst = cpu->r[0];
        uint32_t val = cpu->r[1];
        uint32_t size = cpu->r[2];
        if (dst >= 0x10000000 && dst + size <= 0x10000000 + VFLASH_RAM_SIZE && size <= 0x200000) {
            memset(((VFlash*)ctx)->ram + (dst - 0x10000000), val, size);
            printf("[HLE-MEMSET] %08X size=%X val=%02X\n", dst, size, val);
        }
        cpu->r[0] = dst;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }

    /* HLE game alloc (10A775E0, 10A77648) and free (10A776A8).
     * Bump allocator with dummy vtable at [+0] for safe virtual calls. */
    /* Let 10A775E0 (main alloc) run natively — it may create objects with
     * real vtables. Only HLE 10A77648 (variant that crashes on 40000010). */
    /* HLE native alloc (10A775E0): bump allocator WITHOUT dummy vtable.
     * Let the caller (VFF scene code) set whatever vtable it wants.
     * Native alloc gets stuck in uninitialized game heap. */
    if (addr == 0x10A775E0 && ((VFlash*)ctx)->boot_phase >= 800) {
        static uint32_t native_bump = 0x390000; /* separate from HLE alloc area */
        uint32_t size = cpu->r[0];
        if (size == 0) size = 64;
        if (size > 0x10000) size = 0x10000;
        VFlash *vf2 = (VFlash*)ctx;
        if (native_bump + size < 0x400000) {
            uint32_t ptr = 0x10000000 + native_bump;
            memset(vf2->ram + native_bump, 0, size);
            static int na_log = 0;
            if (na_log < 10) {
                printf("[HLE-ALLOC] 10A775E0(size=%u) → %08X LR=%08X\n",
                       size, ptr, cpu->r[14]);
                na_log++;
            }
            native_bump = (native_bump + size + 15) & ~15u;
            cpu->r[0] = ptr;
        } else {
            cpu->r[0] = 0;
        }
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    if (addr == 0x10A77648 &&
        ((VFlash*)ctx)->boot_phase >= 800) {
        static uint32_t bump = 0x320000;
        uint32_t size = cpu->r[0];
        if (size > 0 && size < 0x10000 && bump + size < 0x380000) {
            uint32_t ptr = 0x10000000 + bump;
            VFlash *va = (VFlash*)ctx;
            memset(va->ram + bump, 0, size);
            *(uint32_t*)(va->ram + bump) = 0x10310800; /* dummy vtable */
            va->ram[bump + 8] = 1;
            *(uint32_t*)(va->ram + bump + 0x104) = 0x84;
            bump = (bump + size + 15) & ~15u;
            cpu->r[0] = ptr;
        } else {
            cpu->r[0] = 0;
        }
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    if (addr == 0x10A776A8 && ((VFlash*)ctx)->boot_phase >= 800) {
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }
    /* Skip engine wait loops (BEQ/polling loops for entity/GPU state) */
    /* Skip GPU/render polling loops. These functions poll render state
     * variables that never get set without a running GPU/vsync system. */
    if (((VFlash*)ctx)->boot_phase >= 800 &&
        (addr == 0x10A20B30 ||
         (addr >= 0x10A73900 && addr <= 0x10A73A00))) {
        cpu->r[0] = 1;
        cpu->r[15] += 4;
        return 1;
    }

    /* Render setup (10B263FC) — let it run natively.
     * This function may populate render_ctx with frame data.
     * Pre-loop calls it until non-zero; we no longer skip it. */

    /* HLE display setup (10A36A94).
     * Calls kernel sleep and screen config. Skip — we set LCD ourselves. */
    if (addr == 0x10A36A94 && ((VFlash*)ctx)->boot_phase >= 900) {
        cpu->r[0] = 0;
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }


    /* CD-ROM init (FUN_10a363e8) — let it run natively.
     * With HLE service lookup/request above, it should work now. */

    /* Handle blocking functions during game init.
     * Some of these functions do useful work (CD-ROM loading) before blocking.
     * Strategy: skip ONLY the pure sleep/wait, let the others run. */
    if (((VFlash*)ctx)->boot_phase >= 800) {
        /* Sleep/wait — return immediately (HLE).
         * On real HW, sleep yields to scheduler which runs other tasks.
         * We can't enable IRQ here — it breaks Cars' boot flow. */
        if (addr == 0x10007304 ||  /* kernel sleep wrapper */
            addr == 0x100086F0 ||  /* kernel polling sleep loop */
            addr == 0x109D1F28) {  /* peripheral wait */
            cpu->r[0] = 0;
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
        /* B . (branch to self) — infinite wait loops in kernel.
         * These appear when µMORE scheduler tries to sleep the task.
         * Skip by returning to caller. */
        if (addr == 0x10A083AC) {  /* sleep (B .) */
            cpu->r[0] = 0;
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
        /* Functions that MAY do CD-ROM I/O — let them run.
         * 109D1F28: peripheral wait — runs init code then blocks
         * 10A176BC: event setup — registers event handlers then blocks */

        /* HLE event pump: 0x10138000-0x10139000 contains a 16-bit offset
         * table (µMORE event dispatch), NOT executable code.  Init task
         * should have populated it with function pointers but was HLE'd.
         * CPU lands here via indirect BL from game loop — just return. */
        if (addr >= 0x10138000 && addr < 0x10139000) {
            static int evp_log = 0;
            if (evp_log < 5) {
                printf("[HLE-EVPUMP] Skip data-as-code at 0x%08X LR=%08X\n",
                       addr, cpu->r[14]);
                evp_log++;
            }
            cpu->r[0] = 0;
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }
    }

    /* Intercept scheduler init: set sched_state=3 before task_start. */
    if (addr == 0x10085E50) {
        VFlash *vfp = (VFlash *)ctx;
        /* Fix guards for task_start */
        if (*(uint32_t*)(vfp->ram + 0x3585E0) == 0)
            *(uint32_t*)(vfp->ram + 0x3585E0) = 3;
        *(uint32_t*)(vfp->ram + 0x1A4F80) = 0;
        /* Write custom timer FIQ handler into RAM.
         * VIC dispatch table at RAM[0x1CBC] chains through wrappers.
         * Instead of populating the whole chain, write a direct handler
         * that clears timer + calls µMORE scheduler wakeup. */
        if (*(uint32_t*)(vfp->ram + 0x1CD8) == 0) {
            /* Write handler at RAM[0xFFE000] (safe high area) */
            uint32_t h = 0xFFE000;
            uint32_t p = h;
            /* PUSH {R0-R3, LR} */
            *(uint32_t*)(vfp->ram + p) = 0xE92D400F; p += 4;
            /* LDR R0, =0x9001000C (timer IntClr) */
            *(uint32_t*)(vfp->ram + p) = 0xE59F0018; p += 4;
            /* MOV R1, #1 */
            *(uint32_t*)(vfp->ram + p) = 0xE3A01001; p += 4;
            /* STR R1, [R0] — clear timer interrupt */
            *(uint32_t*)(vfp->ram + p) = 0xE5801000; p += 4;
            /* MOV R0, #1 (timer event ID) */
            *(uint32_t*)(vfp->ram + p) = 0xE3A00001; p += 4;
            /* BL 0x10084494-ish (scheduler notify — skip for now) */
            /* Just clear and return for now */
            /* POP {R0-R3, LR} */
            *(uint32_t*)(vfp->ram + p) = 0xE8BD400F; p += 4;
            /* MOV PC, LR (return to FIQ wrapper) */
            *(uint32_t*)(vfp->ram + p) = 0xE1A0F00E; p += 4;
            /* Pool: timer IntClr address */
            *(uint32_t*)(vfp->ram + p) = 0x9001000C; p += 4;

            /* Point VIC slot 7 directly to our handler (skip chain) */
            *(uint32_t*)(vfp->ram + 0x1CD8) = 0x10000000 + h;
            printf("[HLE] VIC dispatch[7] → custom handler at 0x%08X\n",
                   0x10000000 + h);
        }
        printf("[HLE] Cleared guards for task_start\n");
        return 0;
    }

    /* HLE FIQ/IRQ handler: timer interrupt reaches 0x10A15DA4 (unpopulated BSS).
     * µMORE never populated this handler — HLE it to:
     * 1. Clear timer IRQ
     * 2. Create game task TCB with entry 0x10C16CC0
     * 3. Enqueue in Nucleus ready list at 0x100AC030
     * 4. Return from exception so scheduler at 0x10087160 can dispatch */
    if (addr == 0x10A15DA4 && ((VFlash*)ctx)->boot_phase >= 800) {
        VFlash *vf = (VFlash *)ctx;
        static int fiq_count = 0;

        /* Clear timer IRQ */
        vf->timer.timer[0].irq_pending = 0;
        ztimer_clear_irq(&vf->timer, 0);
        vf->timer.irq.status &= ~1u;

        /* Create game task TCB at 0x10F00100 (once) */
        uint32_t tcb = 0xF00100;
        if (*(uint32_t*)(vf->ram + tcb) != 0x42435124) {
            memset(vf->ram + tcb, 0, 0x100);
            *(uint32_t*)(vf->ram + tcb + 0x00) = 0x42435124; /* $QCB magic */
            *(uint32_t*)(vf->ram + tcb + 0x04) = 0;          /* next = NULL (single task) */
            *(uint32_t*)(vf->ram + tcb + 0x08) = 1;          /* priority */
            *(uint32_t*)(vf->ram + tcb + 0x0C) = 1;          /* status: active */
            *(uint32_t*)(vf->ram + tcb + 0x1C) = 0x42435424; /* $QBT stack marker */
            *(uint32_t*)(vf->ram + tcb + 0x28) = 10;         /* task ID */
            *(uint32_t*)(vf->ram + tcb + 0x2C) = 0x10C16CC0; /* entry point */
            *(uint32_t*)(vf->ram + tcb + 0x30) = 0x10FFD000; /* stack base */
            *(uint32_t*)(vf->ram + tcb + 0x34) = 0x20000;    /* stack size 128KB */
            printf("[HLE-FIQ] Created game task TCB at 0x%08X entry=0x10C16CC0\n",
                   0x10000000 + (unsigned)tcb);
        }

        /* Mark task as ready with saved context for scheduler */
        *(uint32_t*)(vf->ram + tcb + 0x20) = 4;                  /* state: ready */
        *(uint32_t*)(vf->ram + tcb + 0x58) = 0x10C16CC0;         /* saved PC */
        *(uint32_t*)(vf->ram + tcb + 0x5C) = 0x00000013;         /* saved CPSR: SVC+IRQ en */
        *(uint32_t*)(vf->ram + tcb + 0x60) = 0x10FFD000+0x20000-4; /* saved SP (stack top) */

        /* Enqueue in Nucleus ready list at 0x100AC030 */
        *(uint32_t*)(vf->ram + 0xAC030) = 0x10000000 + tcb;

        /* Set scheduler dispatch flags */
        *(uint32_t*)(vf->ram + 0xBC2BC0) = 1; /* dispatch flag */
        *(uint32_t*)(vf->ram + 0x3585E0) = 3; /* sched_state = running */

        /* Return from exception: restore CPU to pre-interrupt state */
        {
            uint32_t mode = cpu->cpsr & 0x1F;
            if (mode == ARM9_MODE_FIQ || mode == ARM9_MODE_IRQ) {
                uint32_t return_pc = cpu->r[14] - 4;
                uint32_t return_cpsr = cpu->spsr;

                /* Save current exception mode bank */
                if (mode == ARM9_MODE_FIQ) {
                    cpu->r8_fiq = cpu->r[8]; cpu->r9_fiq = cpu->r[9];
                    cpu->r10_fiq = cpu->r[10]; cpu->r11_fiq = cpu->r[11];
                    cpu->r12_fiq = cpu->r[12];
                    cpu->r13_fiq = cpu->r[13]; cpu->r14_fiq = cpu->r[14];
                    cpu->spsr_fiq = cpu->spsr;
                } else {
                    cpu->r13_irq = cpu->r[13]; cpu->r14_irq = cpu->r[14];
                    cpu->spsr_irq = cpu->spsr;
                }

                /* Restore previous mode */
                cpu->cpsr = return_cpsr;
                switch (return_cpsr & 0x1F) {
                case ARM9_MODE_SVC:
                    cpu->r[13] = cpu->r13_svc; cpu->r[14] = cpu->r14_svc;
                    cpu->spsr = cpu->spsr_svc; break;
                case ARM9_MODE_IRQ:
                    cpu->r[13] = cpu->r13_irq; cpu->r[14] = cpu->r14_irq;
                    cpu->spsr = cpu->spsr_irq; break;
                case ARM9_MODE_FIQ:
                    cpu->r[8] = cpu->r8_fiq; cpu->r[9] = cpu->r9_fiq;
                    cpu->r[10] = cpu->r10_fiq; cpu->r[11] = cpu->r11_fiq;
                    cpu->r[12] = cpu->r12_fiq;
                    cpu->r[13] = cpu->r13_fiq; cpu->r[14] = cpu->r14_fiq;
                    cpu->spsr = cpu->spsr_fiq; break;
                default: break; /* USR/SYS: no banked regs */
                }

                /* Restore r[8]-r[12] if returning from FIQ (fix FIQ banking) */
                if (mode == ARM9_MODE_FIQ)
                    memcpy(cpu->r + 8, vf->fiq_saved_r8_12, 5 * sizeof(uint32_t));

                cpu->r[15] = return_pc;
            } else {
                /* Called via BL from normal code */
                cpu->r[0] = 0;
                cpu->r[15] = cpu->r[14] & ~3u;
            }
        }

        if (fiq_count < 10)
            printf("[HLE-FIQ] Timer IRQ #%d → task ready @0x%08X, return PC=%08X mode=%02X\n",
                   fiq_count, 0x10000000 + (unsigned)tcb, cpu->r[15], cpu->cpsr & 0x1F);
        fiq_count++;
        return 1;
    }

    if (addr >= 0x10190000 && addr < 0x10C00000) {
        /* Only intercept if memory contains zeros (unpopulated BSS).
         * If real code exists (written by BOOT.BIN init), let it execute. */
        {
            VFlash *vfp = (VFlash *)ctx;
            uint32_t roff = addr - 0x10000000;
            if (roff < VFLASH_RAM_SIZE && *(uint32_t*)(vfp->ram + roff) != 0)
                return 0; /* real code present, don't intercept */
        }
        static int hle_logged = 0;
        if (hle_logged < 200) {
            /* Only log unique addresses (not repeated int_disable) */
            static uint32_t last_addr = 0;
            static int repeat_count = 0;
            if (addr == last_addr) {
                repeat_count++;
                if (repeat_count <= 3 || (repeat_count % 100 == 0))
                    printf("[HLE] BSS call 0x%08X R0=0x%08X R1=0x%08X LR=0x%08X (x%d)\n",
                           addr, cpu->r[0], cpu->r[1], cpu->r[14], repeat_count);
            } else {
                if (repeat_count > 3)
                    printf("[HLE] ... (repeated %d times)\n", repeat_count);
                repeat_count = 1;
                last_addr = addr;
                printf("[HLE] BSS call 0x%08X R0=0x%08X R1=0x%08X LR=0x%08X\n",
                       addr, cpu->r[0], cpu->r[1], cpu->r[14]);
            }
            hle_logged++;
        }

        /* Log call chain for ALL BSS calls */
        {
            static int chain_logged = 0;
            if (chain_logged < 20) {
                VFlash *vfp = (VFlash *)ctx;
                uint32_t caller = (cpu->r[14] & ~3u) - 4;
                uint32_t caller_insn = 0;
                if (caller >= 0x10000000 && caller < 0x11000000)
                    caller_insn = *(uint32_t*)(vfp->ram + (caller - 0x10000000));
                printf("[HLE] Chain: caller=%08X insn=%08X → target=%08X R2=%08X R3=%08X\n",
                       caller, caller_insn, addr, cpu->r[2], cpu->r[3]);
                chain_logged++;
            }
        }

        /* HLE for specific µMORE services */

        /* 0x10A15CA0: int_disable() — saves CPSR, disables IRQ+FIQ.
         * Called as: old_cpsr = int_disable(); ... int_restore(old_cpsr);
         * R0 = current CPSR on first call, return saved CPSR value. */
        if (addr == 0x10A15CA0) {
            uint32_t old_cpsr = cpu->cpsr;
            cpu->cpsr |= 0xC0; /* disable IRQ + FIQ */
            cpu->r[0] = old_cpsr;
            cpu->r[15] = cpu->r[14] & ~3u;
            return 1;
        }

        /* (chain logging moved above) */

        /* Default: return safe stub address */
        {
            VFlash *vfp = (VFlash *)ctx;
            *(uint32_t*)(vfp->ram + 0xFFE080) = 0xE3A00000; /* MOV R0,#0 */
            *(uint32_t*)(vfp->ram + 0xFFE084) = 0xE12FFF1E; /* BX LR */
        }
        cpu->r[0] = 0x10FFE080; /* safe stub address */
        cpu->r[15] = cpu->r[14] & ~3u;
        return 1;
    }

    return 0; /* not handled */
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

    /* Re-enable timer at frame start if it was disabled.
     * µMORE services may disable the timer; re-enable for IRQ delivery. */
    if (vf->has_rom &&
        (vf->timer.timer[0].ctrl & 0x80) == 0 && /* not enabled */
        vf->boot_phase >= 300) { /* only after game launch */
        if (vf->timer.timer[0].load == 0)
            vf->timer.timer[0].load = 37500; /* 150MHz / 37500 = 4KHz */
        vf->timer.timer[0].ctrl = 0xE2; /* enabled, periodic, 32-bit, IRQ */
        vf->timer.timer[0].count = vf->timer.timer[0].load;
        vf->timer.irq.enable |= 0x01;
    }

    int done = 0;
    while (done < TOTAL) {
        int slice = (TOTAL - done < SLICE) ? (TOTAL - done) : SLICE;

        uint64_t cyc_before = vf->cpu.cycles;
        /* Use JIT when available and game is running, interpreter otherwise. */
        if (vf->jit && vf->boot_phase >= 100 &&
            vf->cpu.r[15] >= 0x10000000 && vf->cpu.r[15] < 0x11000000) {
            /* Force game flags in RAM before every JIT run */
            vf->ram[0xBE49E0] = 1;
            int ran = jit_run(vf->jit, slice);
            vf->cpu.cycles += ran;
        } else {
            arm9_run(&vf->cpu, slice);
        }
        uint32_t actual = (uint32_t)(vf->cpu.cycles - cyc_before);

        /* Trace boot flow: log when PC enters key regions */
        {
            static uint32_t last_region = 0;
        }

        ztimer_tick(&vf->timer, actual);
        done += (int)actual;

        /* Per-slice IRQ delivery: JIT doesn't check IRQ between instructions.
         * If timer IRQ is pending and CPSR allows, deliver it now.
         * This is critical for BOOT.BIN games where game code briefly enables
         * IRQ in kernel sleep then disables it — JIT misses the window. */
        if (vf->boot_phase >= 300 &&
            ztimer_irq_pending(&vf->timer) &&
            !(vf->cpu.cpsr & 0x80)) { /* IRQ not masked */
            if (vf->cpu.r13_irq < 0x10000000u || vf->cpu.r13_irq > 0x10FFFFFFu)
                vf->cpu.r13_irq = 0x10800000;
            arm9_irq(&vf->cpu);
        }

        /* PC escape check moved to per-frame RECOVER (not per-slice).
         * Per-slice was too aggressive — SpongeBob/Scooby crash at boot_phase 800
         * when valid init PCs get caught by escape heuristic mid-slice. */

        /* Detect kernel loop and enable timer + IRQ */
        if (vf->has_rom && vf->frame_count > 5) {
            uint32_t pc = vf->cpu.r[15];
            static int phase = 0;

            /* µMORE init halt at 0x109D1E40: B . with FIQ disabled.
             * Force FIQ enable so timer interrupt can dispatch tasks. */
            /* Detect µMORE init halt: B . (EAFFFFFE) in kernel area.
             * Address varies per game — detect by instruction pattern. */
            if (vf->boot_phase >= 300 &&
                pc >= 0x10090000 && pc < 0x10FFE000 && /* kernel+heap area, exclude idle */
                *(uint32_t*)(vf->ram + (pc - 0x10000000)) == 0xEAFFFFFE) {
                static int loop_1e40 = 0;
                if (loop_1e40 == 0) {
                    vf->cpu.cpsr &= ~0xC0;
                    vf->rtc_regs[0x8C >> 2] = 1;
                    /* Patch IRQ vector to use µMORE dispatch at 0x100873D0
                     * instead of int_disable at 0x10A15CA0. The vector table
                     * chain: ROM[0x18]→SDRAM[0xFF98]→SDRAM[0xFFB8]. */
                    *(uint32_t*)(vf->ram + 0xFFB8) = 0x100873D0;
                    printf("[SCHED] µMORE init halt — patched IRQ dispatch + enabling IRQ\n");
                }
                loop_1e40++;
                /* After 500 ticks at halt, µMORE services are stable.
                 * Launch game code directly since scheduler has no tasks. */
                /* Dump code at 0x1001158C after init task runs */
                if (loop_1e40 == 300 && vf->boot_phase >= 400) {
                    printf("[SCHED] Code at 0x1001158C:\n");
                    for (uint32_t da = 0x11580; da < 0x115C0; da += 4)
                        printf("[SCHED]  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                }
                if (loop_1e40 == 200) {
                    /* µMORE services registered. Run init task again.
                     * First run had sleep intercepted (instant return).
                     * Second run: let sleep-wake simulation work — init task
                     * should progress past sleep to game registration. */
                    printf("[SCHED] Re-running init task with sleep-wake\n");
                    /* Re-load BOOT.BIN */
                    if (vf->cd && vf->cd->is_open) {
                        CDEntry be;
                        if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &be)) {
                            uint32_t dest = 0xC00000;
                            uint32_t msz = VFLASH_RAM_SIZE - dest;
                            if (be.size < msz) msz = be.size;
                            cdrom_read_file(vf->cd, &be, vf->ram + dest, 0, msz);
                        }
                    }
                    /* Write game trampoline at 0x10FFE100.
                     * Called instead of warm reboot callback.
                     * Calls game init wrapper, then idles with IRQ. */
                    {
                        uint32_t t = 0xFFE100;
                        /* Game trampoline:
                         * LDR R0, =context  ; game context pointer
                         * LDR SP, =stack    ; fresh stack
                         * BL game_main      ; call game
                         * B .               ; idle after return
                         * pool: context, stack */
                        *(uint32_t*)(vf->ram+t+0x00) = 0xE59F0010; /* LDR R0,[PC,#+16] → pool[0x18] */
                        *(uint32_t*)(vf->ram+t+0x04) = 0xE59FD010; /* LDR SP,[PC,#+16] → pool[0x1C] */
                        *(uint32_t*)(vf->ram+t+0x08) = 0xE28FE004; /* ADR LR,+12 (return to B .) */
                        /* Call game init wrapper 0x10C16D84.
                         * Returns: 1 = success, 0/-1 = fail.
                         * We call in a loop until it returns 1, then
                         * call game_main with proper context. */
                        int32_t off = (int32_t)(0xC16D84 - (t+0x0C+8)) >> 2;
                        *(uint32_t*)(vf->ram+t+0x0C) = 0xEB000000|(off&0xFFFFFF); /* BL game_init_wrapper */
                        *(uint32_t*)(vf->ram+t+0x10) = 0xE10F0000; /* MRS R0,CPSR */
                        *(uint32_t*)(vf->ram+t+0x14) = 0xEAFFFFFE; /* B . (idle) */
                        /* Pool data */
                        *(uint32_t*)(vf->ram+t+0x18) = 0x10B668A0; /* game context */
                        *(uint32_t*)(vf->ram+t+0x1C) = 0x10FFD000; /* stack */
                        /* Also fill game context field that game_main checks */
                        *(uint8_t*)(vf->ram + 0xB668A0) = 1; /* state byte at context+0 */
                        *(uint8_t*)(vf->ram + 0xB668C4) = 1; /* context+0x24 */
                    }
                    /* Snapshot BEFORE bootstrap: count non-zero words in key areas */
                    {
                        int nz_heap = 0, nz_bss = 0, nz_high = 0;
                        for (uint32_t a = 0x400000; a < 0x800000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_heap++;
                        for (uint32_t a = 0x1A55B0; a < 0x400000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_bss++;
                        for (uint32_t a = 0x800000; a < 0xC00000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_high++;
                        printf("[SNAPSHOT-PRE] heap(0x400000-0x800000)=%d BSS(0x1A55B0-0x400000)=%d high(0x800000-0xC00000)=%d\n",
                               nz_heap, nz_bss, nz_high);
                    }
                    /* Patch callback to trampoline */
                    *(uint32_t*)(vf->ram + 0xC00020) = 0x10FFE100;
                    /* Set game init flags */
                    *(uint32_t*)(vf->ram + 0xBBAE40) = 5;
                    *(uint16_t*)(vf->ram + 0xBE49E0) = 1;
                    /* LCD */
                    vf->lcd.upbase = 0x10800000;
                    vf->lcd.control = 0x182B;
                    /* Launch init task (second run — with sleep-wake) */
                    vf->cpu.cpsr = 0x00000053; /* SVC, IRQ enabled */
                    vf->cpu.r[15] = 0x100113DC;
                    vf->cpu.r[13] = 0x101D42A8;
                    vf->cpu.r[14] = 0x10FFF000;
                    vf->boot_phase = 600; /* enable sleep-wake detection */
                }
            }

            /* Scan for Nucleus TCB structs (magic "TASK"=0x5441534B or "$QCB"=0x42435124) */
            if (vf->boot_phase == 600 && pc >= 0x10FFE100 && pc <= 0x10FFE120) {
                static int tcb_scanned = 0;
                if (!tcb_scanned) {
                    tcb_scanned = 1;
                    printf("[TCB-SCAN] Searching for Nucleus TCBs...\n");
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        uint32_t magic = *(uint32_t*)(vf->ram + a);
                        if (magic == 0x5441534B || magic == 0x42435124) {
                            char tag[5]; memcpy(tag, &magic, 4); tag[4]=0;
                            uint32_t status = *(uint8_t*)(vf->ram + a + 0x0C);
                            uint32_t sp = *(uint32_t*)(vf->ram + a + 0x18);
                            uint32_t entry = *(uint32_t*)(vf->ram + a + 0x2C);
                            uint32_t sstart = *(uint32_t*)(vf->ram + a + 0x1C);
                            printf("[TCB-SCAN] '%s' at 0x%08X: status=%d SP=%08X entry=%08X stack=%08X\n",
                                   tag, 0x10000000+a, status, sp, entry, sstart);
                        }
                    }
                }
            }
            if (vf->boot_phase == 600 && pc >= 0x10FFE100 && pc <= 0x10FFE120) {
                static int td = 0;
                if (!td) {
                    td = 1;
                    printf("[TRAMPOLINE] Game context at 0x10B668A0:\n");
                    for (int d = 0; d < 0x30; d += 16) {
                        printf("[TRAMPOLINE] +%02X: %08X %08X %08X %08X\n", d,
                               *(uint32_t*)(vf->ram+0xB668A0+d),
                               *(uint32_t*)(vf->ram+0xB668A0+d+4),
                               *(uint32_t*)(vf->ram+0xB668A0+d+8),
                               *(uint32_t*)(vf->ram+0xB668A0+d+12));
                    }
                    /* Also check $QCB */
                    printf("[TRAMPOLINE] Task list: [0x3585C0]=%08X\n",
                           *(uint32_t*)(vf->ram+0x3585C0));
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        if (*(uint32_t*)(vf->ram + a) == 0x42435124) {
                            uint32_t v20 = *(uint32_t*)(vf->ram+a+0x20);
                            printf("[TRAMPOLINE] $QCB at 0x%08X state=%08X\n",
                                   0x10000000+a, v20);
                        }
                    }
                    /* Snapshot AFTER bootstrap init funcs */
                    {
                        int nz_heap = 0, nz_bss = 0, nz_high = 0;
                        for (uint32_t a = 0x400000; a < 0x800000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_heap++;
                        for (uint32_t a = 0x1A55B0; a < 0x400000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_bss++;
                        for (uint32_t a = 0x800000; a < 0xC00000; a += 4)
                            if (*(uint32_t*)(vf->ram+a)) nz_high++;
                        printf("[SNAPSHOT-POST] heap=%d BSS=%d high=%d\n",
                               nz_heap, nz_bss, nz_high);
                        /* Find NEW non-zero words in high area (game data?) */
                        printf("[SNAPSHOT-POST] Non-zero ranges in 0x10B00000-0x10C00000:\n");
                        uint32_t run_start = 0;
                        int in_run = 0;
                        for (uint32_t a = 0xB00000; a < 0xC00000; a += 4) {
                            int nz = *(uint32_t*)(vf->ram+a) != 0;
                            if (nz && !in_run) { run_start = a; in_run = 1; }
                            if (!nz && in_run) {
                                printf("[SNAPSHOT-POST]  0x%08X-0x%08X (%d words)\n",
                                       0x10000000+run_start, 0x10000000+a, (a-run_start)/4);
                                in_run = 0;
                            }
                        }
                        if (in_run)
                            printf("[SNAPSHOT-POST]  0x%08X-0x10C00000\n", 0x10000000+run_start);
                    }
                    vf->boot_phase = 700;
                }
            }

            /* After TCB scan, launch the game task directly.
             * Match: trampoline idle (0x10FFE118), init sleep (0x1001158C),
             * or any B . in kernel area with boot_phase >= 600. */
            if (vf->boot_phase >= 600 &&
                ((pc == 0x10FFE118) || (pc == 0x1001158C) ||
                 (pc >= 0x10090000 && pc < 0x10FFE000 &&
                  *(uint32_t*)(vf->ram+(pc-0x10000000)) == 0xEAFFFFFE))) {
                static int game_start = 0;
                if (game_start == 100) {
                    /* Launch ALL tasks by calling their entries sequentially.
                     * Start with highest priority first (task #1), then game task last. */
                    printf("[TASK-LAUNCH] Scanning and launching tasks...\n");
                    /* First: launch task #1 (entry=109D1D30, priority 3, stack 8KB)
                     * This is likely the main service dispatcher task. */
                    uint32_t task1_a = 0;
                    uint32_t game_a = 0;
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        if (*(uint32_t*)(vf->ram+a) == 0x42435124 &&
                            *(uint32_t*)(vf->ram+a+0x1C) == 0x42435424) {
                            uint32_t entry = *(uint32_t*)(vf->ram+a+0x2C);
                            uint32_t id = *(uint32_t*)(vf->ram+a+0x28);
                            uint32_t sz = *(uint32_t*)(vf->ram+a+0x34);
                            if (entry >= 0x10090000) {
                                printf("[TASK-LAUNCH] Task #%d @%08X entry=%08X stack=%X\n",
                                       id, 0x10000000+a, entry, sz);
                                if (id == 1) task1_a = a;
                                if (sz >= 0x10000) game_a = a; /* 64KB+ = game */
                            }
                        }
                    }
                    /* Pick game task (largest stack, 128KB = task #10) */
                    uint32_t pick = game_a ? game_a : task1_a;
                    if (pick) {
                        uint32_t entry = *(uint32_t*)(vf->ram+pick+0x2C);
                        uint32_t sbase = *(uint32_t*)(vf->ram+pick+0x30);
                        uint32_t ssz = *(uint32_t*)(vf->ram+pick+0x34);
                        printf("[TASK-LAUNCH] → entry=%08X stack=%08X+%X\n",
                               entry, sbase, ssz);
                        vf->cpu.cpsr = 0x00000013;
                        vf->cpu.r[15] = entry;
                        vf->cpu.r[13] = sbase + ssz - 4;
                        vf->cpu.r[14] = 0x10FFF000;
                        /* Enable timer for task */
                        vf->timer.timer[0].load = 37500;
                        vf->timer.timer[0].count = 37500;
                        vf->timer.timer[0].ctrl = 0xE2;
                        vf->timer.irq.enable |= 0x01;
                        vf->boot_phase = 800;
                    }
                }
                game_start++;
            }

            /* TCB scan at init task sleep */
            if (vf->boot_phase >= 600 && pc == 0x1001158C) {
                static int tcbs = 0;
                if (!tcbs) {
                    tcbs = 1;
                    printf("[TCB-SCAN] Searching for Nucleus TCBs...\n");
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        uint32_t magic = *(uint32_t*)(vf->ram + a);
                        if (magic == 0x5441534B || magic == 0x42435124) {
                            uint32_t status = *(uint8_t*)(vf->ram+a+0x0C);
                            uint32_t sp = *(uint32_t*)(vf->ram+a+0x18);
                            uint32_t entry = *(uint32_t*)(vf->ram+a+0x2C);
                            uint32_t sstart = *(uint32_t*)(vf->ram+a+0x1C);
                            char tag[5]; memcpy(tag, vf->ram+a, 4); tag[4]=0;
                            printf("[TCB] '%s' @0x%08X st=%d SP=%08X entry=%08X stack=%08X\n",
                                   tag, 0x10000000+a, status, sp, entry, sstart);
                            /* Dump first 0x40 bytes */
                            for (int d = 0; d < 0x40; d += 16)
                                printf("[TCB]  +%02X: %08X %08X %08X %08X\n", d,
                                       *(uint32_t*)(vf->ram+a+d), *(uint32_t*)(vf->ram+a+d+4),
                                       *(uint32_t*)(vf->ram+a+d+8), *(uint32_t*)(vf->ram+a+d+12));
                        }
                    }
                }
            }

            /* Simulate µMORE sleep-wake: when init task hits B . (sleep),
             * skip it after timeout by returning to caller (PC = LR).
             * This simulates scheduler waking the task after sleep timer. */
            /* Simple debug: log when bp>=800 and PC anywhere */
            if (vf->boot_phase >= 800) {
                static int bp800 = 0;
                if (bp800 < 3) {
                    printf("[BP800] pc=%08X bp=%d frame=%llu\n",
                           pc, vf->boot_phase, vf->frame_count);
                    bp800++;
                }
            }
            /* Sleep-wake: skip B . in kernel area or trampoline idle */
            if (vf->boot_phase >= 800 &&
                pc >= 0x10010000 && pc < 0x10FFE000 &&
                *(uint32_t*)(vf->ram + (pc - 0x10000000)) == 0xEAFFFFFE) {
                static int sleep_skip = 0;
                static uint32_t last_sleep_pc = 0;
                if (pc != last_sleep_pc) {
                    /* New sleep address — reset counter */
                    printf("[SLEEP-WAKE] Init task sleep at 0x%08X LR=%08X\n",
                           pc, vf->cpu.r[14]);
                    last_sleep_pc = pc;
                    sleep_skip = 0;
                }
                sleep_skip++;
                if (sleep_skip > 0 && sleep_skip % 200 == 0) {
                    /* Skip B . by returning to caller (LR).
                     * This simulates µMORE scheduler waking the task. */
                    printf("[SLEEP-WAKE] Skipping B . at %08X → LR=%08X (iter %d)\n",
                           pc, vf->cpu.r[14], sleep_skip);
                    vf->cpu.r[0] = 0;
                    vf->cpu.r[15] = vf->cpu.r[14] & ~3u;
                }
            }

            /* POST-INIT disabled — bootstrap launched from init halt instead. */
            if (vf->boot_phase == 400 && pc == 0x1001158C) {
                static int post_init = 0;
                post_init++;
                if (post_init == 100) { /* let a few ticks pass */
                    printf("[POST-INIT] Init task complete → launching BOOT.BIN\n");
                    /* Re-load BOOT.BIN */
                    if (vf->cd && vf->cd->is_open) {
                        CDEntry be;
                        if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &be)) {
                            uint32_t dest = 0xC00000;
                            uint32_t msz = VFLASH_RAM_SIZE - dest;
                            if (be.size < msz) msz = be.size;
                            cdrom_read_file(vf->cd, &be, vf->ram + dest, 0, msz);
                        }
                    }
                    /* Run BOOT.BIN bootstrap at 0x10C00010.
                     * It will set up MMU, run init funcs, call callback.
                     * Callback = 0x1880 → warm reboot #3. */
                    vf->cpu.cpsr = 0x000000D3;
                    vf->cpu.r[15] = 0x10C00010;
                    vf->cpu.r[13] = 0x10C0425C; /* BOOT.BIN stack */
                    vf->cpu.r[14] = 0x1001158C; /* return to halt */
                    vf->boot_phase = 500;
                    printf("[POST-INIT] → BOOT.BIN at 0x10C00010\n");
                }
            }

            /* Dump scheduler dispatch code once */
            if (vf->boot_phase >= 300 && vf->frame_count == 150) {
                static int sched_dump = 0;
                if (!sched_dump) {
                    sched_dump = 1;
                    /* Dump game task entry code */
                    {
                        uint32_t best_a2 = 0, best_sz2 = 0;
                        for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                            if (*(uint32_t*)(vf->ram+a) == 0x42435124 &&
                                *(uint32_t*)(vf->ram+a+0x1C) == 0x42435424) {
                                uint32_t sz = *(uint32_t*)(vf->ram+a+0x34);
                                uint32_t entry = *(uint32_t*)(vf->ram+a+0x2C);
                                if (sz > best_sz2 && entry >= 0x10090000) {
                                    best_sz2 = sz; best_a2 = a;
                                }
                            }
                        }
                        if (best_a2) {
                            uint32_t entry = *(uint32_t*)(vf->ram+best_a2+0x2C);
                            uint32_t eoff = entry - 0x10000000;
                            printf("[GAME-ENTRY] Task entry at 0x%08X:\n", entry);
                            for (uint32_t d = 0; d < 0x80; d += 4)
                                printf("  %08X: %08X\n", entry+d, *(uint32_t*)(vf->ram+eoff+d));
                            /* Dump pool area (entry + 0x140 to entry + 0x160) */
                            printf("[GAME-ENTRY] Pool area:\n");
                            for (uint32_t d = 0x140; d < 0x170; d += 4)
                                printf("  %08X: %08X\n", entry+d, *(uint32_t*)(vf->ram+eoff+d));
                            /* Read context pointer */
                            uint32_t ctx_ptr = *(uint32_t*)(vf->ram+eoff+0x150);
                            printf("[GAME-ENTRY] Context ptr (pool+0x150) = 0x%08X\n", ctx_ptr);
                            if (ctx_ptr >= 0x10000000 && ctx_ptr < 0x11000000) {
                                uint32_t co = ctx_ptr - 0x10000000;
                                printf("[GAME-ENTRY] Context data:\n");
                                for (uint32_t d = 0; d < 0x20; d += 4)
                                    printf("  +%02X: %08X\n", d, *(uint32_t*)(vf->ram+co+d));
                            }
                        }
                    }
                    printf("[SCHED-CODE] Scheduler idle loop at 0x10A219xx:\n");
                    for (uint32_t da = 0xA21980; da < 0xA21A20; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                    printf("[SCHED-CODE] Dispatch area at 0x109D1B30:\n");
                    for (uint32_t da = 0x9D1B30; da < 0x9D1B80; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                    /* Also dump the event_wait/scheduler tick area */
                    printf("[SCHED-CODE] IRQ handler at 0x10A15CA0:\n");
                    for (uint32_t da = 0xA15CA0; da < 0xA15D20; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                    /* Task dispatch function */
                    printf("[SCHED-CODE] task_dispatch at 0x10A15D78:\n");
                    for (uint32_t da = 0xA15D78; da < 0xA15DF0; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                    /* Pool values for IRQ handler */
                    printf("[SCHED-CODE] Pool at 0x10A15E90:\n");
                    for (uint32_t da = 0xA15E90; da < 0xA15EB0; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                    /* Check dispatch flag value */
                    printf("[SCHED-CODE] Dispatch flag at [0x10BC2BC0]=%08X\n",
                           *(uint32_t*)(vf->ram + 0xBC2BC0));
                    /* Pools referenced by LDR [PC,-#xx] */
                    printf("[SCHED-CODE] Pools at 0x10A15C80:\n");
                    for (uint32_t da = 0xA15C80; da < 0xA15CA0; da += 4)
                        printf("  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                }
            }

            /* Flash operation completion: µMORE loop at 0x10A1C5xx polls
             * [0x10BBCFF4] bit 0. Set it periodically to simulate flash
             * write/erase completion. This flag is normally set by a flash
             * controller interrupt or DMA callback. */
            if (vf->boot_phase >= 300 && pc >= 0x10A1C540 && pc <= 0x10A1C590) {
                static int flash_wait = 0;
                flash_wait++;
                if (flash_wait > 20) {
                    *(uint32_t*)(vf->ram + 0xBBCFF4) |= 1;
                    flash_wait = 0;
                }
            }

            /* After µMORE scheduler runs for a while, force-enable IRQ.
             * The scheduler idle loop has IRQ disabled (CPSR bit7=1).
             * Timer IRQs are needed to dispatch tasks. */
            if (vf->boot_phase == 300 && vf->frame_count > 75 &&
                pc >= 0x10900000 && pc < 0x10B00000 &&
                (vf->cpu.cpsr & 0x80)) {
                vf->cpu.cpsr &= ~0x80; /* enable IRQ */
                vf->boot_phase = 301;
                printf("[SCHED] Force-enabled IRQ at frame %llu PC=0x%08X\n",
                       vf->frame_count, pc);
                /* Dump scheduler idle loop code */
                printf("[SCHED] Idle loop disassembly:\n");
                for (uint32_t da = 0xA21980; da < 0xA21A20; da += 4) {
                    uint32_t insn = *(uint32_t*)(vf->ram + da);
                    printf("[SCHED]  %08X: %08X\n", 0x10000000+da, insn);
                }
                /* Also dump the next loop area (detected at runtime) */
                printf("[SCHED] Area at 0x10A1C540:\n");
                for (uint32_t da = 0xA1C540; da < 0xA1C5A0; da += 4) {
                    uint32_t insn = *(uint32_t*)(vf->ram + da);
                    printf("[SCHED]  %08X: %08X\n", 0x10000000+da, insn);
                }
            }

            /* Dump scheduler loop once at frame 85 (disabled) */
            if (vf->boot_phase >= 300 && vf->boot_phase < 400 && vf->frame_count == 85) {
                vf->boot_phase = 400;
                printf("[SCHED] Loop dump at frame 85, PC=0x%08X:\n", pc);
                /* Dump around current PC */
                uint32_t dump_base = (pc - 0x10000000) & ~0x3F;
                for (uint32_t da = dump_base; da < dump_base + 0x80; da += 4) {
                    if (da < VFLASH_RAM_SIZE)
                        printf("[SCHED]  %08X: %08X\n", 0x10000000+da, *(uint32_t*)(vf->ram+da));
                }
            }

            /* µMORE kernel running: enable timer + FIQ after init.
             * Match ANY kernel address after frame 8 (init BLs done by then). */
            if (phase == 0 && vf->frame_count > 50 &&
                pc >= 0x10000000 && pc < 0x10100000 &&
                (vf->cpu.cpsr & 0x1F) == 0x13) { /* SVC mode */
                phase = 99;
                printf("[PHASE99] Triggered at frame %llu PC=%08X\n", vf->frame_count, pc);
                vf->timer.timer[0].load = 37500;
                vf->timer.timer[0].count = 37500;
                vf->timer.timer[0].ctrl = 0xE2;
                vf->timer.irq.enable |= 0x01;
                vf->cpu.cpsr &= ~0x80; /* enable IRQ only (FIQ when kernel is ready) */
                vf->cpu.r13_irq = 0x10800000;
                /* Redirect FIQ vector to our custom timer handler.
                 * FIQ vector at SDRAM[0x1C] does LDR PC,[0x94].
                 * Overwrite SDRAM[0x94] with our handler address.
                 * Handler at RAM[0xFFE000] clears timer + returns. */
                {
                    /* Simple IRQ handler: clear timer only.
                     * µMORE IRQ dispatch (0x872A4) crashes on uninitialized tables.
                     * Task dispatch needs a different approach. */
                    uint32_t h = 0xFFE000;
                    uint32_t p = h;
                    *(uint32_t*)(vf->ram + p) = 0xE92D500F; p += 4; /* PUSH {R0-R3,R12,LR} */
                    *(uint32_t*)(vf->ram + p) = 0xE59F000C; p += 4; /* LDR R0,[PC,#12] */
                    *(uint32_t*)(vf->ram + p) = 0xE3A01001; p += 4; /* MOV R1,#1 */
                    *(uint32_t*)(vf->ram + p) = 0xE5801000; p += 4; /* STR R1,[R0] clear */
                    *(uint32_t*)(vf->ram + p) = 0xE8BD500F; p += 4; /* POP {R0-R3,R12,LR} */
                    *(uint32_t*)(vf->ram + p) = 0xE25EF004; p += 4; /* SUBS PC,LR,#4 */
                    *(uint32_t*)(vf->ram + p) = 0x9001000C; p += 4; /* pool: timer IntClr */
                    /* Point FIQ vector pool to our handler */
                    *(uint32_t*)(vf->ram + 0x94) = 0x10000000 + h;
                    /* Overwrite BOTH IRQ and FIQ vectors in SDRAM
                     * to branch directly to our handler (skip dispatch chain) */
                    {
                        int32_t irq_off = (int32_t)(h - (0x18 + 8)) >> 2;
                        *(uint32_t*)(vf->ram + 0x18) = 0xEA000000 | (irq_off & 0xFFFFFF); /* B handler */
                        int32_t fiq_off = (int32_t)(h - (0x1C + 8)) >> 2;
                        *(uint32_t*)(vf->ram + 0x1C) = 0xEA000000 | (fiq_off & 0xFFFFFF); /* B handler */
                    }
                    /* Force timer to IRQ (not FIQ) */
                    vf->timer.irq.fiq_sel = 0;
                    printf("[IRQ] Direct vector → handler at 0x%08X\n", 0x10000000 + h);
                }
                /* Fix µMORE IRQ dispatcher: populate dispatch table entries.
                 * µMORE handler at 0x100873D0 reads [0x100857B8] for handler addr.
                 * Point it to our timer handler so dispatch works properly.
                 * Also keep SDRAM[0xFF98] redirect as fallback. */
                {
                    /* Populate dispatch table with our timer handler */
                    *(uint32_t*)(vf->ram + 0x857B8) = 0x10FFF040;
                    /* Also redirect SDRAM[0xFF98] as insurance */
                    int32_t b_off = (int32_t)(0xFFF040 - (0xFF98 + 8)) >> 2;
                    *(uint32_t*)(vf->ram + 0xFF98) = 0xEA000000 | (b_off & 0xFFFFFF);
                    *(uint32_t*)(vf->ram + 0xFFF080) = 0xE1B0F00E; /* MOVS PC,LR */
                }
                /* IRQ wrapper at 0xFFF040 */
                uint32_t w = 0xFFF040;
                *(uint32_t*)(vf->ram+w)=0xE92D500F; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE59F000C; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE3A01001; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE5801000; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE8BD500F; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE25EF004; w+=4;
                *(uint32_t*)(vf->ram+w)=0x9001000C; w+=4;
                printf("[KERN] Timer + IRQ enabled, kernel at 0x%08X\n", pc);
                printf("[KERN] Heap: [0x359660]=%08X [0x3596B0]=%08X\n",
                       *(uint32_t*)(vf->ram + 0x359660),
                       *(uint32_t*)(vf->ram + 0x3596B0));
                printf("[KERN] Tasks: list_head[0x3585C0]=%08X sched_state[0x3585E0]=%08X\n",
                       *(uint32_t*)(vf->ram + 0x3585C0),
                       *(uint32_t*)(vf->ram + 0x3585E0));
                printf("[KERN] Task mgr[0xACC78]=%08X [0x358600]=%08X\n",
                       *(uint32_t*)(vf->ram + 0xACC78),
                       *(uint32_t*)(vf->ram + 0x358600));
                /* Check if heap was used (allocated blocks) */
                uint32_t heap_start = *(uint32_t*)(vf->ram + 0x359660);
                if (heap_start >= 0x10000000 && heap_start < 0x11000000) {
                    uint32_t hoff = heap_start - 0x10000000;
                    uint32_t blk0 = *(uint32_t*)(vf->ram + hoff);
                    uint32_t blk1 = *(uint32_t*)(vf->ram + hoff + 4);
                    printf("[KERN] Heap block: [%08X]=%08X next=%08X (free=%d size=%08X)\n",
                           heap_start, blk0, blk1, blk0 & 1, blk0 & ~1u);
                }
            }

            /* Phase 100: launch BOOT.BIN init to populate µMORE services. */
            if (phase == 99 && vf->frame_count > 55) {
                phase = 100;
                vf->boot_phase = 100; /* sync with reboot handler path */
                /* Fix page table: identity map ALL RAM */
                uint32_t ttb_off = vf->cpu.cp15.ttb;
                if (ttb_off >= 0x10000000) ttb_off -= 0x10000000;
                if (ttb_off + 0x4000 < VFLASH_RAM_SIZE) {
                    for (uint32_t mb = 0x100; mb <= 0x10F; mb++) {
                        uint32_t *l1 = (uint32_t*)(vf->ram + ttb_off + mb*4);
                        if ((*l1 & 3) == 0)
                            *l1 = (mb << 20) | 0xC0E;
                    }
                    tlb_flush();
                }
                /* LCD handle */
                uint32_t h = 0xBF0000, fb = 0x800000;
                memset(vf->ram + h, 0, 0x1000);
                *(uint32_t*)(vf->ram + h + 0x5C) = 0x10000000 + fb;
                *(uint32_t*)(vf->ram + 0xBC0A40) = 0x10000000 + h;
                vf->lcd.upbase = 0x10000000 + fb;
                vf->lcd.control = 0x182B;
                /* Install idle stub at 0xFFF000 (IRQ-enabled infinite loop) */
                {
                    uint32_t p = 0xFFF000;
                    *(uint32_t*)(vf->ram + p) = 0xE10F0000; p += 4; /* MRS R0, CPSR */
                    *(uint32_t*)(vf->ram + p) = 0xE3C000C0; p += 4; /* BIC R0, #0xC0 */
                    *(uint32_t*)(vf->ram + p) = 0xE129F000; p += 4; /* MSR CPSR_cxsf, R0 */
                    *(uint32_t*)(vf->ram + p) = 0xEAFFFFFE; p += 4; /* B . */
                }
                /* BSS stays as zeros — NULL trap handles function calls.
                 * Data reads from BSS correctly return 0. */
                /* Patch BOOT.BIN callback to idle (prevent reboot loop).
                 * BOOT.BIN header[0x20] = callback addr, originally 0x1880
                 * (ROM code that triggers warm reboot). Redirect to idle. */
                *(uint32_t*)(vf->ram + 0xC00020) = 0x10FFF000;
                /* Disable timer during BOOT.BIN init */
                vf->timer.timer[0].ctrl = 0x10;
                vf->timer.timer[0].irq_pending = 0;
                vf->timer.irq.status = 0;
                /* Run BOOT.BIN init to register µMORE services.
                 * Entry at 0x10C0011C does init functions then calls
                 * callback → idle at 0x10FFF000. */
                {
                    int n5a = 0;
                    for (uint32_t a = 0xC00000; a < 0xE00000 && a + 4 <= VFLASH_RAM_SIZE; a += 4)
                        if (*(uint32_t*)(vf->ram + a) == 0x5A5A5A5A) n5a++;
                    /* If 5A5A5A5A still present, ROM relocation was bypassed.
                     * Apply it ourselves: scan BOOT.BIN for 5A5A5A5A literal pool
                     * entries and patch them with base+offset (0x10C00000 + file_offset).
                     * This is what the ROM's loader does during CD read. */
                    if (n5a > 100) {
                        /* ROM 5A5A5A5A relocation: the ROM patches all 5A entries
                         * during CD read. The algorithm generates a 4096-entry
                         * lookup table: value = (index << 20) | 0x0DF2.
                         * Index assignment: sequential from +0x4000, skipping
                         * non-5A gaps, wrapping to +0x0260 after +0x425F.
                         * All games share the same BOOT.BIN 5A layout. */
                        int patched = 0, idx = 0;
                        /* Phase 1: +0x4000 to end of data (includes +0x476C) */
                        for (uint32_t a = 0x4000; a < 0xC000; a += 4) {
                            if (*(uint32_t*)(vf->ram + 0xC00000 + a) == 0x5A5A5A5A) {
                                *(uint32_t*)(vf->ram + 0xC00000 + a) =
                                    ((uint32_t)idx << 20) | 0x0DF2;
                                idx++; patched++;
                            }
                        }
                        /* Phase 2: +0x0260 to +0x3FFF (wrap) */
                        for (uint32_t a = 0x260; a < 0x4000; a += 4) {
                            if (*(uint32_t*)(vf->ram + 0xC00000 + a) == 0x5A5A5A5A) {
                                *(uint32_t*)(vf->ram + 0xC00000 + a) =
                                    ((uint32_t)idx << 20) | 0x0DF2;
                                idx++; patched++;
                            }
                        }
                        /* Header padding (+0x24-0x25F): leave as 5A (not patched by ROM) */
                        printf("[BOOT-RELOC] Patched %d entries (idx 0-%d, formula: idx<<20|0xDF2)\n",
                               patched, idx - 1);
                    }
                }
                vf->cpu.null_trap_enabled = 1;
                vf->cpu.cpsr = 0x000000D3; /* SVC, IRQ+FIQ disabled */
                vf->cpu.r[15] = 0x10C0011C;
                vf->cpu.r[13] = 0x10FFE000;
                vf->cpu.r[14] = 0x10FFF000;
                printf("[BOOT] Running BOOT.BIN init at 0x10C0011C (callback→idle)\n");
            }

            /* Phase 101: BOOT.BIN init done → idle reached → launch service entry.
             * Triggered by either static phase==100 OR boot_phase==100 (from reboot handler).
             * Guard: skip if AUTO-LAUNCH already advanced boot_phase past this. */
            if ((phase == 100 || vf->boot_phase == 100) &&
                vf->boot_phase < 800 &&
                (pc == 0x10FFF000 || pc == 0x10FFF00C)) {
                phase = 101; /* sync both */
                phase = 101;
                printf("[BOOT] BOOT.BIN init complete → idle at 0x%08X\n", pc);
                /* Early backup: RTOS code should be intact at this point.
                 * Do it unconditionally (even if later backup exists). */
                {
                    if (!vf->rtos_backup) vf->rtos_backup = malloc(VFLASH_RAM_SIZE);
                    if (vf->rtos_backup) {
                        memcpy(vf->rtos_backup, vf->ram, VFLASH_RAM_SIZE);
                    }
                }
                /* Check if BSS service table got populated */
                uint32_t svc = *(uint32_t*)(vf->ram + 0x9A0950);
                printf("[BOOT] BSS[0x109A0950] = 0x%08X (%s)\n",
                       svc, svc == 0xE12FFF1E ? "BX LR stub" : "populated!");
                /* Check µMORE service entry from warm boot vector */
                {
                    uint32_t se = *(uint32_t*)(vf->ram + 0xFFCC);
                    if (se >= 0x10000000 && se < 0x11000000) {
                        uint32_t se_off = se - 0x10000000;
                        printf("[BOOT] Service entry [0x%08X] = %08X %08X %08X %08X\n", se,
                               *(uint32_t*)(vf->ram+se_off), *(uint32_t*)(vf->ram+se_off+4),
                               *(uint32_t*)(vf->ram+se_off+8), *(uint32_t*)(vf->ram+se_off+12));
                    } else {
                        printf("[BOOT] Service entry [0x%08X] (out of SDRAM range)\n", se);
                    }
                }
                /* Check warm boot vectors */
                printf("[BOOT] [0x10FFC8] = %08X %08X %08X %08X\n",
                       *(uint32_t*)(vf->ram+0xFFC8), *(uint32_t*)(vf->ram+0xFFCC),
                       *(uint32_t*)(vf->ram+0xFFD0), *(uint32_t*)(vf->ram+0xFFD4));
                /* Check idle stub integrity after BOOT.BIN init */
                printf("[BOOT] Idle stub: [0xFFF000]=%08X [0xFFF004]=%08X [0xFFF008]=%08X [0xFFF00C]=%08X\n",
                       *(uint32_t*)(vf->ram + 0xFFF000), *(uint32_t*)(vf->ram + 0xFFF004),
                       *(uint32_t*)(vf->ram + 0xFFF008), *(uint32_t*)(vf->ram + 0xFFF00C));
                printf("[BOOT] TTB=0x%08X MMU=%d\n", vf->cpu.cp15.ttb, vf->cpu.cp15.mmu_enabled);
                /* Check BOOT.BIN integrity at game launch */
                printf("[BOOT] RAM[0xC16CC0] = %08X (expect E52DE004)\n",
                       *(uint32_t*)(vf->ram + 0xC16CC0));
                printf("[BOOT] RAM[0xC16D30] = %08X (expect E5933000)\n",
                       *(uint32_t*)(vf->ram + 0xC16D30));
                /* Check task state after BOOT.BIN init */
                printf("[BOOT] Task list: [0x3585C0]=%08X sched_state: [0x3585E0]=%08X\n",
                       *(uint32_t*)(vf->ram+0x3585C0), *(uint32_t*)(vf->ram+0x3585E0));
                printf("[BOOT] Heap: [0x359660]=%08X\n", *(uint32_t*)(vf->ram+0x359660));
                /* Scan for $QCB task structs after BOOT.BIN init */
                {
                    int qcb_count = 0;
                    for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                        if (*(uint32_t*)(vf->ram + a) == 0x42435124) { /* $QCB */
                            printf("[BOOT] $QCB at 0x%08X:\n", 0x10000000+a);
                            for (int d = 0; d < 0x80; d += 16) {
                                printf("[BOOT]  +%02X: %08X %08X %08X %08X\n", d,
                                       *(uint32_t*)(vf->ram+a+d),
                                       *(uint32_t*)(vf->ram+a+d+4),
                                       *(uint32_t*)(vf->ram+a+d+8),
                                       *(uint32_t*)(vf->ram+a+d+12));
                            }
                            qcb_count++;
                        }
                    }
                    printf("[BOOT] Found %d $QCB task structs\n", qcb_count);
                    /* Save best game task TCB (largest stack with valid entry) */
                    {
                        uint32_t b_a=0, b_sz=0;
                        for (uint32_t a2 = 0x100000; a2 < 0xC00000; a2 += 4) {
                            if (*(uint32_t*)(vf->ram+a2) == 0x42435124 &&
                                *(uint32_t*)(vf->ram+a2+0x1C) == 0x42435424) {
                                uint32_t e2 = *(uint32_t*)(vf->ram+a2+0x2C);
                                uint32_t s2 = *(uint32_t*)(vf->ram+a2+0x34);
                                if (s2 > b_sz && e2 >= 0x10090000) { b_sz=s2; b_a=a2; }
                            }
                        }
                        if (b_a) {
                            vf->saved_game_entry = *(uint32_t*)(vf->ram+b_a+0x2C);
                            vf->saved_game_stack = *(uint32_t*)(vf->ram+b_a+0x30);
                            vf->saved_game_stack_sz = b_sz;
                            printf("[BOOT] Saved game TCB: entry=%08X stack=%08X+%X\n",
                                   vf->saved_game_entry, vf->saved_game_stack, b_sz);
                        }
                    }
                    /* Set task dispatch flag at 0x10BC2BC0.
                     * IRQ handler checks *(0x10BC2BC0) != 0 before calling
                     * task_dispatch at 0x10A15D78. */
                    *(uint32_t*)(vf->ram + 0xBC2BC0) = 1;
                    printf("[BOOT] Set task dispatch flag [0x10BC2BC0]=1\n");

                    /* If we found the BOOT.BIN task struct, populate it */
                    if (*(uint32_t*)(vf->ram + 0xB908E0) == 0x42435124) {
                        printf("[BOOT] Populating $QCB at 0x10B908E0 with game entry\n");
                        /* Set task as ready with saved context pointing to BOOT.BIN.
                         * Try different offsets for entry point and stack. */
                        uint32_t t = 0xB908E0;
                        /* Try common µMORE TCB layouts:
                         * +0x04 = next task pointer
                         * +0x08 = priority/flags
                         * +0x0C = entry point
                         * +0x10 = stack pointer
                         * +0x14 = state (0=inactive, 1=ready, 2=running)
                         */
                        *(uint32_t*)(vf->ram + t + 0x08) = 1;          /* priority */
                        *(uint32_t*)(vf->ram + t + 0x0C) = 0x10C0011C; /* entry = BOOT.BIN init */
                        *(uint32_t*)(vf->ram + t + 0x10) = 0x10FFD000; /* stack */
                        *(uint32_t*)(vf->ram + t + 0x14) = 1;          /* state = ready */
                        /* Also try TI-Nspire style: saved context at end */
                        *(uint32_t*)(vf->ram + t + 0x58) = 0x10C0011C; /* saved PC */
                        *(uint32_t*)(vf->ram + t + 0x5C) = 0x000000D3; /* saved CPSR */
                        *(uint32_t*)(vf->ram + t + 0x60) = 0x10FFD000; /* saved SP */
                        /* Register in task list */
                        *(uint32_t*)(vf->ram + 0x3585C0) = 0x10B908E0;
                    }
                }
                /* Service entry populated. Read address from warm boot vector
                 * SDRAM[0xFFCC] (second word of LDR PC,[PC] + pool pair). */
                /* Read service entry from warm boot vector.
                 * SDRAM[0xFFC8] = LDR PC,[PC] which loads from [0xFFD0].
                 * SDRAM[0xFFD0] = service entry address. */
                uint32_t svc_entry = *(uint32_t*)(vf->ram + 0xFFD0);
                if (svc_entry < 0x10000000 || svc_entry >= 0x11000000) {
                    /* Try [0xFFCC] as fallback */
                    svc_entry = *(uint32_t*)(vf->ram + 0xFFCC);
                    if (svc_entry < 0x10000000 || svc_entry >= 0x11000000)
                        svc_entry = 0x109D11E0; /* hardcoded fallback */
                }
                printf("[BOOT] Service entry at 0x%08X — calling directly\n", svc_entry);
                printf("[BOOT] FIQ vectors: [0xFF9C]=%08X [0xFFBC]=%08X\n",
                       *(uint32_t*)(vf->ram+0xFF9C), *(uint32_t*)(vf->ram+0xFFBC));
                /* Don't re-patch SDRAM[0xFF98] — let native µMORE FIQ handler run.
                 * Set FIQ handler flag at RTC[0x8C] for ROM FIQ dispatch. */
                vf->rtc_regs[0x8C >> 2] = 1;
                /* Re-enable MMU with full RAM identity map */
                vf->cpu.cp15.ttb = 0x100A8000;
                vf->cpu.cp15.mmu_enabled = 1;
                vf->cpu.cp15.control = 0x0000507D;
                {
                    uint32_t ttb_off = 0xA8000;
                    for (uint32_t mb = 0x100; mb <= 0x10F; mb++) {
                        uint32_t *l1 = (uint32_t*)(vf->ram + ttb_off + mb*4);
                        if ((*l1 & 3) == 0)
                            *l1 = (mb << 20) | 0xC0E;
                    }
                    tlb_flush();
                }
                /* Enable timer (don't touch fiq_sel — µMORE manages it) */
                vf->timer.timer[0].load = 37500;
                vf->timer.timer[0].count = 37500;
                vf->timer.timer[0].ctrl = 0xE2;
                vf->timer.irq.enable |= 0x01;
                /* Jump to service entry with R0=2 (warm boot mode) */
                vf->cpu.cpsr = 0x000000D3;
                vf->cpu.r[15] = svc_entry;
                vf->cpu.r[13] = 0x10FFD000;
                vf->cpu.r[14] = 0x10FFF000;
                vf->cpu.r[0] = 2; /* warm boot indicator */
                vf->cpu.r13_irq = 0x10800000;
                vf->cpu.null_trap_enabled = 1;
                /* Register a game task in the µMORE task list.
                 * Task struct layout: $TCB magic, entry point, stack, state.
                 * Use heap area for the task struct (0x104000+). */
                {
                    uint32_t task = 0xF00000; /* task struct in high RAM */
                    memset(vf->ram + task, 0, 0x100);
                    *(uint32_t*)(vf->ram + task + 0x00) = 0x42435124; /* $QCB magic */
                    *(uint32_t*)(vf->ram + task + 0x08) = 1;          /* flags: active */
                    *(uint32_t*)(vf->ram + task + 0x20) = 4;          /* state: ready */
                    /* Saved context at +0x58 */
                    *(uint32_t*)(vf->ram + task + 0x58) = 0x10CFAEA0; /* saved PC: game main */
                    *(uint32_t*)(vf->ram + task + 0x5C) = 0x00000053; /* saved CPSR: SVC+IRQ en */
                    *(uint32_t*)(vf->ram + task + 0x60) = 0x10FFD000; /* saved SP */
                    /* Register in task list */
                    *(uint32_t*)(vf->ram + 0x3585C0) = 0x10000000 + task;
                    *(uint32_t*)(vf->ram + 0xACC78) = 0x10000000 + task;
                    printf("[BOOT] Registered game task at 0x%08X\n", 0x10000000 + task);
                }

                /* Set flash completion flags that µMORE scheduler polls.
                 * Loop at 0x10A1C5xx reads [0x10BBCFF4/F8] and [0x10BBD00C]. */
                *(uint32_t*)(vf->ram + 0xBBCFF4) = 1;
                *(uint32_t*)(vf->ram + 0xBBCFF8) = 1;
                *(uint32_t*)(vf->ram + 0xBBD00C) |= 1;

                vf->boot_phase = 300;
                printf("[BOOT] → 0x%08X (R0=2, SP=0x10FFD000)\n", svc_entry);
            }

            /* Phase 101b: old game launch code (kept for reference) */
            if (0) {
                /* BOOT.BIN init may have overwritten game code area.
                 * Re-load from disc to ensure integrity. */
                if (vf->cd && vf->cd->is_open) {
                    CDEntry boot_entry;
                    if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &boot_entry)) {
                        uint32_t dest = 0xC00000;
                        uint32_t max_sz = VFLASH_RAM_SIZE - dest;
                        if (boot_entry.size < max_sz) max_sz = boot_entry.size;
                        cdrom_read_file(vf->cd, &boot_entry,
                            vf->ram + dest, 0, max_sz);
                        printf("[BOOT] Re-loaded BOOT.BIN (%u bytes)\n", max_sz);
                    }
                }
                /* After BOOT.BIN init, µMORE IRQ handler is initialized.
                 * SDRAM[0xFF98] was set to LDR PC,[PC,#0x18] → SDRAM[0xFFB8]
                 * which points to µMORE dispatch at 0x100873D0.
                 * Let the native µMORE handler run (don't re-patch). */
                printf("[BOOT] Using µMORE native IRQ handler: SDRAM[0xFF98]=%08X → [0xFFB8]=%08X\n",
                       *(uint32_t*)(vf->ram + 0xFF98), *(uint32_t*)(vf->ram + 0xFFB8));

                /* Set init flags so game code progresses:
                 * [0x10BBAE40] = 5: game_init_poll returns 1 (ready)
                 * [0x10BE49E0] = 1: game main doesn't skip (state != 0) */
                *(uint32_t*)(vf->ram + 0xBBAE40) = 5;
                *(uint16_t*)(vf->ram + 0xBE49E0) = 1;
                printf("[GAME] Set init flags: [0x10BBAE40]=5 [0x10BE49E0]=1\n");

                /* Launch game main function (0x10CFAEA0).
                 * This is the full game entry: prologue → init → game loop.
                 * Function: PUSH {R4-R11,LR}; ...; BL game_init; game_loop */
                vf->cpu.cpsr = 0x000000D3;
                vf->cpu.r[15] = 0x10CFAEA0;
                vf->cpu.r[13] = 0x10FFD000; /* plenty of stack */
                vf->cpu.r[14] = 0x10FFF000;
                printf("[GAME] Launching game main at 0x10CFAEA0\n");
            }

            /* Phase 102: game init done → set up event system + IRQ */
            if (phase == 101 && (pc == 0x10FFF000 || pc == 0x10FFF00C)) {
                phase = 102;

                /* Create synthetic event context for the game dispatcher.
                 * Dispatcher at 0x10C1377C takes R0=type, R1=context.
                 * Callers load context from [0x10B909C0].
                 * Context structure: +108 (0x6C) = event data pointer.
                 * Event data: +12=field, +16=field, +20=field, +24=field. */
                {
                    uint32_t ctx_base = 0xF00000; /* context struct */
                    uint32_t evt_base = 0xF01000; /* event data */
                    memset(vf->ram + ctx_base, 0, 0x2000);
                    /* context + 0x6C = pointer to event data */
                    *(uint32_t*)(vf->ram + ctx_base + 0x6C) = 0x10000000 + evt_base;
                    /* Fill event data with initial values.
                     * Fields at evt+12, +16, +20, +24 become R0/args for services.
                     * Also fill other potential fields the game reads. */
                    *(uint32_t*)(vf->ram + evt_base + 0x00) = 0x00000001; /* flags/type */
                    *(uint32_t*)(vf->ram + evt_base + 0x04) = 0x10800000; /* framebuffer? */
                    *(uint32_t*)(vf->ram + evt_base + 0x08) = 0x00000140; /* width=320 */
                    *(uint32_t*)(vf->ram + evt_base + 0x0C) = 0x00000001; /* field+12: enable/count */
                    *(uint32_t*)(vf->ram + evt_base + 0x10) = 0x00000140; /* field+16: width=320 */
                    *(uint32_t*)(vf->ram + evt_base + 0x14) = 0x000000F0; /* field+20: height=240 */
                    *(uint32_t*)(vf->ram + evt_base + 0x18) = 0x10800000; /* field+24: fb addr */
                    /* Store context pointer where game expects it */
                    *(uint32_t*)(vf->ram + 0xB909C0) = 0x10000000 + ctx_base;
                    printf("[EVENT] Context at 0x%08X, events at 0x%08X\n",
                           0x10000000 + ctx_base, 0x10000000 + evt_base);
                }

                printf("[EVENT] Event pump active (C-side, per-frame)\n");

                vf->timer.timer[0].load = 37500;
                vf->timer.timer[0].count = 37500;
                vf->timer.timer[0].ctrl = 0xE2;
                vf->timer.irq.enable |= 0x01;
                printf("[GAME] Init done → event system + IRQ enabled\n");
            }

            /* Phase 300: BOOT.BIN done → idle → launch game init */
            if ((phase == 99 || phase == 200) && vf->frame_count > 110 &&
                (pc == 0x10FFF00C || pc == 0x10FFF000)) {
                phase = 300;
                printf("[GAME] BOOT.BIN init done! Launching game at 0x10C16CC0\n");
                /* Check if BOOT.BIN populated BSS service functions */
                printf("[GAME] BSS[0x109A0950]=%08X BSS[0x109A0868]=%08X\n",
                       *(uint32_t*)(vf->ram + 0x9A0950),
                       *(uint32_t*)(vf->ram + 0x9A0868));
                vf->cpu.null_trap_enabled = 1;
                vf->cpu.r[15] = 0x10C16CC0;
                vf->cpu.r[13] = 0x10FFE000;
                vf->cpu.r[14] = 0x10FFF000;
                vf->cpu.cpsr  = 0x000000D3; /* SVC, IRQ off during init */
                /* Re-enable timer for game */
                vf->timer.timer[0].load = 37500;
                vf->timer.timer[0].count = 37500;
                vf->timer.timer[0].ctrl = 0xE2;
                vf->timer.irq.enable |= 0x01;
                /* Don't touch fiq_sel — µMORE manages it */
            }

            /* Phase 200: force task launch after scheduler stabilizes.
             * Scheduler can't dispatch tasks (IRQ chain broken).
             * Directly switch CPU to task entry point. */
            if (phase == 99 && vf->frame_count > 100 &&
                pc >= 0x10000000 && pc < 0x10100000) {
                phase = 200;
                /* Task TCB at 0x100AC0E0:
                 * entry=0x100113DC, stack=0x101D42A8, size=0x4000 */
                uint32_t task_entry = 0x100113DC;
                uint32_t task_stack = 0x101D42A8;
                vf->cpu.r[15] = task_entry;
                vf->cpu.r[13] = task_stack;
                vf->cpu.r[14] = 0x10095B54; /* return to scheduler idle */
                vf->cpu.cpsr  = 0x00000053; /* SVC, IRQ enabled */
                vf->cpu.null_trap_enabled = 1;
                printf("[TASK-LAUNCH] Forcing task at 0x%08X SP=0x%08X\n",
                       task_entry, task_stack);
            }

            /* Phase 1: scheduler loop detected → re-copy modules + redirect */
            if (phase == 0 && (pc >= 0x1007FFC0 && pc <= 0x10080060)) {
                phase = 1;
                /* Re-copy modules erased by BSS clear + set sched_state */
                memcpy(vf->ram + 0xAC000, vf->rom + 0xAE010, 0xF95B0);
                memcpy(vf->ram + 0x9FFD4, vf->rom + 0xC02C, 0xA1FE4);
                *(uint32_t*)(vf->ram + 0x3585E0) = 3;

                /* Create large task struct with context at +0x58 for dispatch.
                 * Handler calls 0x86AE4 which reads task+0x58 as saved context.
                 * We set PC at +0x58 to BOOT.BIN entry for game init. */
                {
                    uint32_t task = 0xFFF100;
                    memset(vf->ram + task, 0, 0x100);
                    *(uint32_t*)(vf->ram + task + 0x00) = 0x42435124; /* $QCB */
                    *(uint32_t*)(vf->ram + task + 0x08) = 1;          /* flags */
                    *(uint32_t*)(vf->ram + task + 0x14) = 0;          /* counter */
                    *(uint32_t*)(vf->ram + task + 0x20) = 4;          /* state: ready */
                    /* Saved context at +0x58 (ARM register dump for task switch) */
                    *(uint32_t*)(vf->ram + task + 0x58) = 0x10C0011C; /* saved PC */
                    *(uint32_t*)(vf->ram + task + 0x5C) = 0x00000013; /* saved CPSR */
                    *(uint32_t*)(vf->ram + task + 0x60) = 0x10FFE000; /* saved SP */

                    *(uint32_t*)(vf->ram + 0x1A9440) = 0x10000000 + task;
                    *(uint32_t*)(vf->ram + 0x3585C0) = 0x10000000 + task;
                    *(uint32_t*)(vf->ram + 0xACC78) = 0x10000000 + task;
                    printf("[SCHED] Phase 1: task with BOOT.BIN entry\n");
                }
                vf->cpu.r[15] = 0x10FFF000;
                vf->cpu.r[13] = 0x10FFE000;
            }

            /* Phase 2: kernel entry returned to scheduler loop or idle */
            if (phase == 1 && ((pc >= 0x10FFF000 && pc <= 0x10FFF010) ||
                               (pc >= 0x1007FFC0 && pc <= 0x10080060))) {
                phase = 2;
                printf("[SCHED] Phase 2: BOOT.BIN init done, installing IRQ handler\n");

                /* Dump task state */
                printf("[SCHED] task_table[0..3] = %08X %08X %08X %08X\n",
                       *(uint32_t*)(vf->ram+0x1A9440), *(uint32_t*)(vf->ram+0x1A9444),
                       *(uint32_t*)(vf->ram+0x1A9448), *(uint32_t*)(vf->ram+0x1A944C));
                printf("[SCHED] sched_state=%08X task_list=%08X task_mgr=%08X\n",
                       *(uint32_t*)(vf->ram+0x3585E0), *(uint32_t*)(vf->ram+0x3585C0),
                       *(uint32_t*)(vf->ram+0xACC78));

                /* Install IRQ wrapper at 0xFFF040 (BOOT.BIN can't overwrite) */
                uint32_t w = 0xFFF040;
                *(uint32_t*)(vf->ram+w)=0xE92D500F; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE3A00001; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE3A01000; w+=4;
                { int32_t bo=(int32_t)(0x872A4-(w+8))>>2;
                  *(uint32_t*)(vf->ram+w)=0xEB000000|(bo&0xFFFFFF); w+=4; }
                *(uint32_t*)(vf->ram+w)=0xE59F000C; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE3A01001; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE5801000; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE8BD500F; w+=4;
                *(uint32_t*)(vf->ram+w)=0xE25EF004; w+=4;
                *(uint32_t*)(vf->ram+w)=0x9001000C; w+=4;

                /* Re-patch IRQ vector (BOOT.BIN init overwrote 0xFFDC) */
                *(uint32_t*)(vf->ram+0xFFDC) = 0x10FFF040;

                /* Timer + VIC */
                if (vf->timer.timer[0].load == 0) {
                    vf->timer.timer[0].load = 37500;
                    vf->timer.timer[0].count = 37500;
                    vf->timer.timer[0].ctrl = 0xE2;
                    vf->timer.irq.enable |= 0x01;
                }

                /* Set scheduler state and jump to kernel code */
                *(uint32_t*)(vf->ram + 0x3585E0) = 3;

                /* Fill zero RAM with BX LR safety net.
                 * Covers: low ROM BSS (0x10000-0x80000), BSS (0x1A55B0+), heap. */
                {
                    int filled = 0;
                    /* Low area: skip vectors/trampolines (0-0xFFFF) */
                    for (uint32_t a = 0x10000; a < 0x80000; a += 4)
                        if (*(uint32_t*)(vf->ram + a) == 0)
                            { *(uint32_t*)(vf->ram + a) = 0xE12FFF1E; filled++; }
                    /* BSS + heap */
                    for (uint32_t a = 0x1A55B0; a < 0xC00000; a += 4)
                        if (*(uint32_t*)(vf->ram + a) == 0)
                            { *(uint32_t*)(vf->ram + a) = 0xE12FFF1E; filled++; }
                    printf("[SCHED] Filled %d zero words with BX LR\n", filled);
                }

                /* Fix page table: identity map ALL 16MB RAM.
                 * BOOT.BIN page table has FAULT for many VA ranges
                 * (0x10A, 0x10C, etc). Game code needs full access. */
                {
                    uint32_t ttb_off = vf->cpu.cp15.ttb - 0x10000000;
                    int fixed = 0;
                    for (uint32_t mb = 0x100; mb <= 0x10F; mb++) {
                        uint32_t *l1 = (uint32_t*)(vf->ram + ttb_off + mb * 4);
                        if ((*l1 & 3) == 0) { /* FAULT → add section mapping */
                            *l1 = (mb << 20) | 0xC0E; /* section, cacheable, bufferable, AP=full */
                            fixed++;
                        }
                    }
                    tlb_flush();
                    printf("[SCHED] Fixed %d FAULT entries in page table → identity\n", fixed);
                }
                /* LCD handle: [BC0A40] → handle_ptr, handle[0x5C] = framebuf.
                 * LCD init reads [handle+0x5C] and adds 0x20000 for palette.
                 * Also fill other fields that game code may read. */
                {
                    uint32_t h = 0xBF0000, fb = 0x800000;
                    memset(vf->ram + h, 0, 0x1000);
                    memset(vf->ram + fb, 0, 0x40000);
                    *(uint32_t*)(vf->ram + h + 0x5C) = 0x10000000 + fb;
                    /* Fill more fields to prevent NULL dereferences */
                    *(uint32_t*)(vf->ram + h + 0x00) = 0x10000000 + h + 0x100; /* sub-struct */
                    *(uint32_t*)(vf->ram + h + 0x04) = 320;  /* width */
                    *(uint32_t*)(vf->ram + h + 0x08) = 240;  /* height */
                    *(uint32_t*)(vf->ram + h + 0x0C) = 16;   /* bpp */
                    *(uint32_t*)(vf->ram + h + 0x10) = 640;  /* stride */
                    *(uint32_t*)(vf->ram + 0xBC0A40) = 0x10000000 + h;
                    vf->lcd.upbase = 0x10000000 + fb;
                    vf->lcd.control = 0x182B;
                    printf("[SCHED] LCD handle fb@0x10%06X\n", fb);
                }
                /* Jump to game init with NULL trap enabled */
                vf->cpu.null_trap_enabled = 1;
                vf->cpu.cpsr = 0x00000093;
                vf->cpu.r[15] = 0x10C16CC0;
                vf->cpu.r[13] = 0x10FFE000;
                vf->cpu.r[14] = 0x10FFF000; /* return → idle */
                /* Clear pending IRQ so it doesn't fire immediately */
                vf->timer.timer[0].irq_pending = 0;
                vf->timer.irq.status = 0;
                printf("[SCHED] Phase 2: → GAME INIT at 0x10C16CC0 (IRQ disabled)\n");
            }
        }

        /* Deliver IRQ or FIQ if pending and CPSR allows */
        if (vf->frame_count > 1 &&
            (ztimer_irq_pending(&vf->timer) || ztimer_fiq_pending(&vf->timer))) {
            if (vf->cpu.r13_irq < 0x10000000u || vf->cpu.r13_irq > 0x10FFFFFFu)
                vf->cpu.r13_irq = 0x10800000;
            /* Deliver as FIQ if fiq_sel routes timer there, else IRQ */
            int use_fiq = ztimer_fiq_pending(&vf->timer);
            static int irq_logged = 0;
            if (!irq_logged) {
                printf("[IRQ] First %s! PC=0x%08X CPSR=0x%08X MMU=%d fiq_sel=0x%X\n",
                       use_fiq ? "FIQ" : "IRQ",
                       vf->cpu.r[15], vf->cpu.cpsr, vf->cpu.cp15.mmu_enabled,
                       vf->timer.irq.fiq_sel);
                irq_logged = 1;
            }
            /* Log IRQ delivery details after µMORE init */
            if (vf->boot_phase >= 300) {
                static int irq2_log = 0;
                if (irq2_log < 30 && !(vf->cpu.cpsr & 0x80)) { /* only when IRQ enabled */
                    printf("[IRQ2] Delivering %s: PC=%08X CPSR=%08X\n",
                           use_fiq ? "FIQ" : "IRQ", vf->cpu.r[15], vf->cpu.cpsr);
                    printf("[IRQ2] SDRAM[0xFF98]=%08X [0xFF9C]=%08X [0xFFB8]=%08X [0xFFDC]=%08X\n",
                           *(uint32_t*)(vf->ram+0xFF98),
                           *(uint32_t*)(vf->ram+0xFF9C),
                           *(uint32_t*)(vf->ram+0xFFB8),
                           *(uint32_t*)(vf->ram+0xFFDC));
                    printf("[IRQ2] RTC[0x8C]=%08X (FIQ handler flag)\n",
                           vf->rtc_regs[0x8C >> 2]);
                    printf("[IRQ2] VIC: status=%08X enable=%08X fiq_sel=%08X\n",
                           vf->timer.irq.status, vf->timer.irq.enable, vf->timer.irq.fiq_sel);
                    irq2_log++;
                }
            }
            {
                static int del_log = 0;
                if (del_log < 5 && vf->boot_phase >= 300) {
                    printf("[DELIVER] use_fiq=%d fiq_pending=%d irq_pending=%d fiq_sel=%08X\n",
                           use_fiq, ztimer_fiq_pending(&vf->timer), ztimer_irq_pending(&vf->timer),
                           vf->timer.irq.fiq_sel);
                    del_log++;
                }
            }
            if (use_fiq) {
                memcpy(vf->fiq_saved_r8_12, vf->cpu.r + 8, 5 * sizeof(uint32_t));
                arm9_fiq(&vf->cpu);
            } else
                arm9_irq(&vf->cpu);
        }
    }

    /* HLE scheduler tick: when timer fires during game phase, create game
     * task TCB and enqueue in Nucleus ready list so the native scheduler
     * at 0x10087160 can dispatch it.  The custom IRQ handler at 0x10FFE000
     * only clears the timer — this HLE adds the missing scheduler work. */
    if (vf->boot_phase >= 800 && vf->timer.timer[0].irq_pending == 0 &&
        (vf->timer.irq.status & 1)) {
        /* Timer just fired (irq_pending cleared by handler, status still set).
         * Alternatively, trigger on every frame after boot_phase 800. */
    }
    /* Per-frame scheduler tick — ensure game task is in ready list.
     * Only active during boot_phase 800 (before game loop takes over at 900). */
    if (vf->boot_phase == 800 && vf->has_rom) {
        static int sched_tick_init = 0;
        uint32_t tcb = 0xF00100;

        /* Create game task TCB once */
        if (!sched_tick_init) {
            sched_tick_init = 1;
            memset(vf->ram + tcb, 0, 0x100);
            *(uint32_t*)(vf->ram + tcb + 0x00) = 0x42435124; /* $QCB magic */
            *(uint32_t*)(vf->ram + tcb + 0x04) = 0x10000000 + tcb; /* next = self (circular) */
            *(uint32_t*)(vf->ram + tcb + 0x08) = 1;          /* priority */
            *(uint32_t*)(vf->ram + tcb + 0x0C) = 1;          /* status: active */
            *(uint32_t*)(vf->ram + tcb + 0x1C) = 0x42435424; /* $QBT stack marker */
            *(uint32_t*)(vf->ram + tcb + 0x20) = 4;          /* state: NU_READY */
            *(uint32_t*)(vf->ram + tcb + 0x28) = 10;         /* task ID */
            *(uint32_t*)(vf->ram + tcb + 0x2C) = 0x10C16CC0; /* entry point */
            *(uint32_t*)(vf->ram + tcb + 0x30) = 0x10FFD000; /* stack base */
            *(uint32_t*)(vf->ram + tcb + 0x34) = 0x20000;    /* stack size 128KB */
            /* Saved context for scheduler to restore (Nucleus TCB layout) */
            *(uint32_t*)(vf->ram + tcb + 0x58) = 0x10C16CC0;         /* saved PC */
            *(uint32_t*)(vf->ram + tcb + 0x5C) = 0x00000013;         /* saved CPSR: SVC */
            *(uint32_t*)(vf->ram + tcb + 0x60) = 0x10FFD000+0x20000-4; /* saved SP */
            printf("[HLE-SCHED] Created game task TCB at 0x%08X entry=0x10C16CC0\n",
                   0x10000000 + (unsigned)tcb);
        }

        /* Keep task in ready list at 0x100AC030 every frame */
        *(uint32_t*)(vf->ram + 0xAC030) = 0x10000000 + tcb;
        *(uint32_t*)(vf->ram + 0xBC2BC0) = 1; /* dispatch flag */
        *(uint32_t*)(vf->ram + 0x3585E0) = 3; /* sched_state = running */
    }

    /* Don't force-raise timer IRQ — timer fires naturally via ztimer_tick.
     * Force-raising caused IRQ storm (re-armed after every clear). */

    /* (task state dump moved to phase 2 above) */
    if (0 && vf->frame_count == 10 && vf->has_rom) {
        /* Also scan RAM for non-zero near task areas */
        int nz = 0;
        for (uint32_t a = 0x3585A0; a < 0x358700; a += 4)
            if (*(uint32_t*)(vf->ram+a)) nz++;
        printf("[POST-INIT] Non-zero words in 0x3585A0-0x358700: %d\n", nz);
    }

    /* Don't clear framebuffer every frame — keep last displayed content.
     * On real hardware, the display controller holds the last frame.
     * Only clear if explicitly requested or on first frame. */
    if (vf->frame_count == 0)
        memset(vf->framebuf, 0,
               VFLASH_SCREEN_W * VFLASH_SCREEN_H * sizeof(uint32_t));

    /* Don't clear IRQ status — µMORE C code polls it for timer events.
     * Timer tick naturally sets pending bits; µMORE reads and clears them. */

    /* Fix LR for µMORE scheduler. ROM uses LDR PC (not BL) to
     * jump to scheduler at 0xB8000820. This doesn't set LR, so
     * scheduler always returns to 0xB8000820 (re-entering itself).
     * Patch: when CPU is in scheduler code area (0x10010234-0x10010284),
     * set LR to 0xB8000824 (next ROM instruction after LDR PC). */
    /* Patch ROM[0x138] (flash remap LDR PC) to include LR setup.
     * Write BL-equivalent: MOV LR,PC then LDR PC at RAM[0x138-0x13C].
     * This only needs to happen once. */
    /* No remap disable needed — flash reads always from ROM. */

    if ((vf->frame_count % 10) == 0) {
        printf("[Frame %lu] PC=0x%08X CPSR=0x%08X R7=0x%08X R8=0x%08X R9=0x%08X"
               " T0:load=%u ctrl=0x%X cnt=%u IRQen=0x%X IRQst=0x%X\n",
                (unsigned long)vf->frame_count, vf->cpu.r[15],
                vf->cpu.cpsr, vf->cpu.r[7], vf->cpu.r[8], vf->cpu.r[9],
                vf->timer.timer[0].load, vf->timer.timer[0].ctrl,
                vf->timer.timer[0].count, vf->timer.irq.enable, vf->timer.irq.status);
    }
    /* Display PTX image from disc on frame 0 (instant splash screen) */
    if (vf->frame_count == 0 && vf->has_rom && vf->cd && vf->cd->is_open) {
        CDEntry ptx_entry;
        static const char *ptx_names[] = {
            "_101KW_CHEETAH.PTX", "_101KW_PIC.PTX", "_KWF101.PTX",
            "_FG_CHI_E01.PTX", "_KWF301.PTX", NULL
        };
        int ptx_found = 0;
        for (int pi = 0; ptx_names[pi] && !ptx_found; pi++)
            ptx_found = cdrom_find_file_any(vf->cd, ptx_names[pi], &ptx_entry);

        if (ptx_found && ptx_entry.size > 100) {
            uint8_t *ptx_buf = malloc(ptx_entry.size);
            if (ptx_buf) {
                int rd = cdrom_read_file(vf->cd, &ptx_entry, ptx_buf, 0, ptx_entry.size);
                if (rd > 44) {
                    uint32_t hdr_sz = *(uint32_t*)ptx_buf;
                    if (hdr_sz >= 12 && hdr_sz <= 256 && hdr_sz < (uint32_t)rd) {
                        /* PTX stores 2 interleaved images (scene + sprite) at stride 512.
                         * Even rows = scene image (A), odd rows = sprite overlay (B).
                         * Display scene image scaled to fill 320×240 display. */
                        uint32_t pw = 512;
                        uint32_t total_rows = ((uint32_t)rd - hdr_sz) / (pw * 2);
                        uint32_t src_h = total_rows / 2;
                        uint32_t src_w = pw < VFLASH_SCREEN_W ? pw : VFLASH_SCREEN_W;
                        const uint16_t *px = (const uint16_t*)(ptx_buf + hdr_sz);
                        for (uint32_t dy = 0; dy < VFLASH_SCREEN_H; dy++) {
                            uint32_t sy = dy * src_h / VFLASH_SCREEN_H;
                            for (uint32_t dx = 0; dx < src_w; dx++) {
                                uint16_t p = px[sy * 2 * pw + dx]; /* even rows */
                                uint8_t b5 = ((p >> 10) & 0x1F), g5 = ((p >> 5) & 0x1F), r5 = (p & 0x1F);
                                uint8_t r = (r5 << 3) | (r5 >> 2);
                                uint8_t g = (g5 << 3) | (g5 >> 2);
                                uint8_t b = (b5 << 3) | (b5 >> 2);
                                vf->framebuf[dy * VFLASH_SCREEN_W + dx] =
                                    0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                            }
                        }
                        printf("[PTX] Displayed %s (%ux%u → %ux%u XBGR1555)\n",
                               ptx_entry.name, src_w, src_h,
                               VFLASH_SCREEN_W, VFLASH_SCREEN_H);
                    }
                }
                free(ptx_buf);
            }
        }
    }

    /* PTX gallery: scan disc for all PTX files, navigate with Left/Right */
    if (!vf->ptx_loaded && vf->cd && vf->cd->is_open) {
        vf->ptx_loaded = 1;
        vf->ptx_list = calloc(256, sizeof(CDEntry));
        vf->ptx_count = 0;
        /* Recursive scan: list all dirs, then scan each for PTX */
        uint8_t pvd[2048];
        if (cdrom_read_sector(vf->cd, 16, pvd)) {
            uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
            uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
            /* Two-level scan: root → subdir → files (up to 1024 per dir) */
            CDEntry l1[64];
            int n1 = cdrom_list_dir(vf->cd, root_lba, root_size, l1, 64);
            for (int i = 0; i < n1 && vf->ptx_count < 256; i++) {
                if (!l1[i].is_dir) continue;
                CDEntry *l2 = malloc(1024 * sizeof(CDEntry));
                if (!l2) continue;
                int n2 = cdrom_list_dir(vf->cd, l1[i].lba, l1[i].size, l2, 1024);
                for (int j = 0; j < n2 && vf->ptx_count < 256; j++) {
                    char *dot = strrchr(l2[j].name, '.');
                    if (dot && strcasecmp(dot, ".PTX") == 0)
                        vf->ptx_list[vf->ptx_count++] = l2[j];
                }
                free(l2);
            }
        }
        printf("[PTX-GALLERY] Found %d PTX images on disc\n", vf->ptx_count);

        /* Also scan for WAV files */
        vf->wav_list = calloc(1024, sizeof(CDEntry));
        vf->wav_count = 0;
        if (cdrom_read_sector(vf->cd, 16, pvd)) {
            uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
            uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
            CDEntry l1w[64];
            int n1w = cdrom_list_dir(vf->cd, root_lba, root_size, l1w, 64);
            for (int iw = 0; iw < n1w && vf->wav_count < 1024; iw++) {
                if (!l1w[iw].is_dir) continue;
                CDEntry *l2w = malloc(1024 * sizeof(CDEntry));
                if (!l2w) continue;
                int n2w = cdrom_list_dir(vf->cd, l1w[iw].lba, l1w[iw].size, l2w, 1024);
                for (int jw = 0; jw < n2w && vf->wav_count < 1024; jw++) {
                    char *dot = strrchr(l2w[jw].name, '.');
                    if (dot && strcasecmp(dot, ".WAV") == 0)
                        vf->wav_list[vf->wav_count++] = l2w[jw];
                }
                free(l2w);
            }
        }
        printf("[WAV] Found %d WAV sound files\n", vf->wav_count);

        /* Also scan for MJP files */
        vf->mjp_list = calloc(64, sizeof(CDEntry));
        vf->mjp_count = 0;
        if (cdrom_read_sector(vf->cd, 16, pvd)) {
            uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
            uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
            CDEntry l1m[64];
            int n1m = cdrom_list_dir(vf->cd, root_lba, root_size, l1m, 64);
            for (int im = 0; im < n1m && vf->mjp_count < 64; im++) {
                if (!l1m[im].is_dir) continue;
                CDEntry *l2m = malloc(256 * sizeof(CDEntry));
                if (!l2m) continue;
                int n2m = cdrom_list_dir(vf->cd, l1m[im].lba, l1m[im].size, l2m, 256);
                for (int jm = 0; jm < n2m && vf->mjp_count < 64; jm++) {
                    char *dot = strrchr(l2m[jm].name, '.');
                    if (dot && strcasecmp(dot, ".MJP") == 0)
                        vf->mjp_list[vf->mjp_count++] = l2m[jm];
                }
                free(l2m);
            }
        }
        printf("[MJP] Found %d MJP video files\n", vf->mjp_count);
    }
    /* Navigate gallery with Left/Right keys (only when video not playing) */
    if (vf->ptx_count > 0 && vf->cd && vf->cd->is_open && !vf->mjp_player.playing && vf->boot_phase < 900) {
        uint32_t pressed = vf->input & ~vf->input_prev;
        int new_idx = vf->ptx_index;
        if (pressed & VFLASH_BTN_RIGHT) new_idx = (vf->ptx_index + 1) % vf->ptx_count;
        if (pressed & VFLASH_BTN_LEFT)  new_idx = (vf->ptx_index + vf->ptx_count - 1) % vf->ptx_count;
        if (new_idx != vf->ptx_index || vf->frame_count == 0) {
            vf->ptx_index = new_idx;
            CDEntry *e = &vf->ptx_list[vf->ptx_index];
            uint8_t *ptx_buf = malloc(e->size);
            if (ptx_buf) {
                int rd = cdrom_read_file(vf->cd, e, ptx_buf, 0, e->size);
                if (rd > 44) {
                    uint32_t hdr_sz = *(uint32_t*)ptx_buf;
                    if (hdr_sz >= 12 && hdr_sz <= 256 && hdr_sz < (uint32_t)rd) {
                        uint32_t pw = 512;
                        uint32_t total_rows = ((uint32_t)rd - hdr_sz) / (pw * 2);
                        uint32_t src_h = total_rows / 2;
                        uint32_t src_w = pw < VFLASH_SCREEN_W ? pw : VFLASH_SCREEN_W;
                        const uint16_t *px = (const uint16_t*)(ptx_buf + hdr_sz);
                        for (uint32_t dy = 0; dy < VFLASH_SCREEN_H; dy++) {
                            uint32_t sy = dy * src_h / VFLASH_SCREEN_H;
                            for (uint32_t dx = 0; dx < src_w; dx++) {
                                uint16_t p = px[sy * 2 * pw + dx];
                                uint8_t b5 = ((p >> 10) & 0x1F), g5 = ((p >> 5) & 0x1F), r5 = (p & 0x1F);
                                uint8_t r = (r5 << 3) | (r5 >> 2);
                                uint8_t g = (g5 << 3) | (g5 >> 2);
                                uint8_t b = (b5 << 3) | (b5 >> 2);
                                vf->framebuf[dy * VFLASH_SCREEN_W + dx] =
                                    0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                            }
                        }
                        printf("[PTX-GALLERY] %d/%d: %s (%ux%u)\n",
                               vf->ptx_index + 1, vf->ptx_count, e->name, src_w, src_h);
                        /* Play corresponding WAV (use PTX index mod wav_count) */
                        if (vf->wav_count > 0 && vf->audio && vf->audio->initialized) {
                            int wi = vf->ptx_index % vf->wav_count;
                            play_wav_from_cd(vf, &vf->wav_list[wi]);
                        }
                    }
                }
                free(ptx_buf);
            }
        }
    }

    /* MJP video player — Z=next video, X=stop, auto-load on frame 1.
     * Disabled during game phase to prevent video flickering over game scene. */
    if (vf->cd && vf->cd->is_open && vf->mjp_count > 0 && vf->boot_phase < 900) {
        uint32_t pressed = vf->input & ~vf->input_prev;
        int load_mjp = 0;
        if (pressed & VFLASH_BTN_RED) {
            /* Z = next video (works even during playback) */
            vf->mjp_index = (vf->mjp_index + 1) % vf->mjp_count;
            /* Stop current video first */
            if (vf->mjp_player.playing) {
                vf->mjp_player.playing = 0;
                free(vf->mjp_player.data); vf->mjp_player.data = NULL;
                free(vf->mjp_player.hdr); vf->mjp_player.hdr = NULL;
            }
            load_mjp = 1;
        }
        if (pressed & VFLASH_BTN_YELLOW) {
            /* X = stop video */
            if (vf->mjp_player.playing) {
                vf->mjp_player.playing = 0;
                free(vf->mjp_player.data); vf->mjp_player.data = NULL;
                free(vf->mjp_player.hdr); vf->mjp_player.hdr = NULL;
            }
        }
        if (load_mjp) {
            CDEntry mjp_entry = vf->mjp_list[vf->mjp_index];
        if (mjp_entry.size > 100) {
            printf("[MJP-PLAY] Loading %s (%u bytes)...\n", mjp_entry.name, mjp_entry.size);
            uint32_t load_sz = mjp_entry.size < 16*1024*1024 ? mjp_entry.size : 16*1024*1024;
            vf->mjp_player.data = malloc(load_sz);
            if (vf->mjp_player.data) {
                int rd = cdrom_read_file(vf->cd, &mjp_entry, vf->mjp_player.data, 0, load_sz);
                vf->mjp_player.data_size = (uint32_t)rd;
                vf->mjp_player.chunk_off = 0x40;
                vf->mjp_player.frame_rate = 1;
                vf->mjp_player.frame_skip = 0;
                /* Read video dimensions from MIAV header */
                if (rd >= 0x1C) {
                    vf->mjp_player.vid_w = *(uint16_t*)(vf->mjp_player.data + 0x18);
                    vf->mjp_player.vid_h = *(uint16_t*)(vf->mjp_player.data + 0x1A);
                }

                /* Extract I-frame header (SOI + DQT + DHT + SOF0, before SOS) */
                if (rd >= 0x4C && memcmp(vf->mjp_player.data + 0x40, "00dc", 4) == 0) {
                    uint32_t f0sz = *(uint32_t*)(vf->mjp_player.data + 0x48);
                    if (f0sz > (uint32_t)(rd - 0x4C)) f0sz = rd - 0x4C;
                    /* Pair-swap to parse header */
                    uint8_t *tmp = malloc(f0sz + 2);
                    for (uint32_t i = 0; i + 1 < f0sz; i += 2)
                        { tmp[i] = vf->mjp_player.data[0x4C+i+1]; tmp[i+1] = vf->mjp_player.data[0x4C+i]; }
                    uint32_t hp = 2, sos_pos = f0sz;
                    while (hp + 3 < f0sz && tmp[hp] == 0xFF) {
                        uint8_t m = tmp[hp+1];
                        uint16_t ml = ((uint16_t)tmp[hp+2]<<8)|tmp[hp+3];
                        if (m == 0xDA) { sos_pos = hp; break; }
                        hp += 2 + ml;
                    }
                    vf->mjp_player.hdr = malloc(sos_pos);
                    memcpy(vf->mjp_player.hdr, tmp, sos_pos);
                    vf->mjp_player.hdr_len = sos_pos;
                    free(tmp);
                    printf("[MJP-PLAY] Header: %u bytes (tables before SOS)\n", sos_pos);
                }
                vf->mjp_player.playing = 1;
                printf("[MJP-PLAY] Playing %u bytes of video\n", vf->mjp_player.data_size);
            }
        }
        } /* end load_mjp block */
    }

    /* Auto-play background WAVs when no MJP is playing */
    if (vf->wav_count > 0 && vf->audio && vf->audio->initialized &&
        !vf->mjp_player.playing && (vf->frame_count % 120) == 60) {
        /* Check if audio buffer is nearly empty */
        uint32_t buffered = (vf->audio->write_pos >= vf->audio->read_pos)
            ? vf->audio->write_pos - vf->audio->read_pos
            : vf->audio->buf_size - vf->audio->read_pos + vf->audio->write_pos;
        if (buffered < 44100) { /* less than 0.5 seconds of audio */
            play_wav_from_cd(vf, &vf->wav_list[vf->wav_auto_idx % vf->wav_count]);
            vf->wav_auto_idx++;
        }
    }

    /* Decode next MJP frame if player is active (stop during game phase) */
    if (vf->mjp_player.playing && vf->boot_phase >= 900) {
        vf->mjp_player.playing = 0;
        free(vf->mjp_player.data); vf->mjp_player.data = NULL;
        free(vf->mjp_player.hdr);  vf->mjp_player.hdr = NULL;
    }
    if (vf->mjp_player.playing) {
        if (++vf->mjp_player.frame_skip >= vf->mjp_player.frame_rate) {
            vf->mjp_player.frame_skip = 0;
            uint8_t *d = vf->mjp_player.data;
            uint32_t dsz = vf->mjp_player.data_size;
            uint32_t co = vf->mjp_player.chunk_off;

            /* Process audio chunks (01wb) before next video frame */
            while (co + 12 < dsz && memcmp(d + co, "00dc", 4) != 0) {
                uint32_t csz = *(uint32_t*)(d + co + 8);
                if (csz == 0 || csz > 2000000) break;
                if (memcmp(d + co, "01wb", 4) == 0 && csz >= 4) {
                    /* IMA ADPCM audio: byte-swap then decode */
                    uint8_t *abuf = malloc(csz);
                    if (abuf) {
                        for (uint32_t i = 0; i + 1 < csz; i += 2)
                            { abuf[i] = d[co+12+i+1]; abuf[i+1] = d[co+12+i]; }
                        audio_decode_ima_adpcm(vf->audio, abuf, csz);
                        free(abuf);
                    }
                }
                co += 12 + csz;
                if (co & 1) co++;
            }

            if (co + 12 < dsz && memcmp(d + co, "00dc", 4) == 0) {
                uint32_t csz = *(uint32_t*)(d + co + 8);
                if (csz > 0 && csz < 2000000 && co + 12 + csz <= dsz) {
                    if (mjp_decode_raw(vf->video, d + co + 12, csz,
                                       vf->mjp_player.hdr, vf->mjp_player.hdr_len)) {
                        /* Scale decoded frame to fill 320×240 display.
                         * Use actual JPEG dimensions (not MIAV header). */
                        uint32_t *src = mjp_get_framebuf(vf->video);
                        int sw = vf->video->width;
                        int src_w = vf->video->decoded_w ? vf->video->decoded_w : sw;
                        int src_h = vf->video->decoded_h ? vf->video->decoded_h : vf->video->height;
                        if (src_w > sw) src_w = sw;
                        if (src_h > vf->video->height) src_h = vf->video->height;
                        for (int dy = 0; dy < VFLASH_SCREEN_H; dy++) {
                            int sy = dy * src_h / VFLASH_SCREEN_H;
                            for (int dx = 0; dx < VFLASH_SCREEN_W; dx++) {
                                int sx = dx * src_w / VFLASH_SCREEN_W;
                                vf->framebuf[dy * VFLASH_SCREEN_W + dx] = src[sy * sw + sx];
                            }
                        }
                        vf->vid.fb_dirty = 1;
                    }
                    co += 12 + csz;
                    if (co & 1) co++;
                }
            }

            vf->mjp_player.chunk_off = co;
            /* Loop: restart from beginning */
            if (co + 12 >= dsz)
                co = 0x40;
        }
    }

    /* Fix task table pointer if it's garbage (contains ROM code instead of pointer) */
    if (vf->frame_count == 2 && vf->has_rom) {
        uint32_t tbl_ptr = *(uint32_t*)(vf->ram + 0xC08);
        printf("[TASK] Table pointer at RAM[0xC08] = 0x%08X\n", tbl_ptr);
        if (tbl_ptr < 0x10000000 || tbl_ptr > 0x10FFFFFF) {
            /* Create dummy task table at RAM[0x7F0000] (safe area near top)
             * with one entry that has non-zero value to match scheduler check */
            uint32_t task_tbl = 0x7F0000;
            for (int i = 0; i < 64; i++)
                *(uint32_t*)(vf->ram + task_tbl + i * 4) = 0x00000001;
            *(uint32_t*)(vf->ram + 0xC08) = 0x10000000 + task_tbl;
            printf("[TASK] Installed dummy task table at 0x%08X\n", 0x10000000 + task_tbl);
        }
    }

    /* VFF scene viewer: load and display VFF scene on Enter/Up/Down.
     * VFF format: header (0x400) + sections (code, framebuf, data).
     * Only active when video is not playing and game loop is not running. */
    if (vf->cd && vf->cd->is_open && !vf->mjp_player.playing && vf->boot_phase < 900) {
        static int vff_loaded = 0;
        static CDEntry vff_list[32];
        static int vff_count = 0, vff_index = 0;
        uint32_t pressed = vf->input & ~vf->input_prev;

        /* Scan for VFF files on first use */
        if (!vff_loaded) {
            vff_loaded = 1;
            uint8_t pvd[2048];
            if (cdrom_read_sector(vf->cd, 16, pvd)) {
                uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
                uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
                CDEntry l1[64];
                int n1 = cdrom_list_dir(vf->cd, root_lba, root_size, l1, 64);
                for (int i = 0; i < n1 && vff_count < 32; i++) {
                    if (!l1[i].is_dir) continue;
                    CDEntry *l2 = malloc(256 * sizeof(CDEntry));
                    if (!l2) continue;
                    int n2 = cdrom_list_dir(vf->cd, l1[i].lba, l1[i].size, l2, 256);
                    for (int j = 0; j < n2 && vff_count < 32; j++) {
                        char *dot = strrchr(l2[j].name, '.');
                        if (dot && strcasecmp(dot, ".VFF") == 0)
                            vff_list[vff_count++] = l2[j];
                    }
                    free(l2);
                }
                printf("[VFF] Found %d VFF scene files\n", vff_count);
            }
        }

        /* Navigate VFF scenes with Up/Down, load on Enter */
        if (vff_count > 0) {
            static int vff_fmt = 0; /* 0=XBGR1555, 1=byte-swap, 2=RGB565, 3=stride512 */
            static int vff_sec = -1; /* -1=auto, 0/1/2=specific section */
            int reload = 0;
            if (pressed & VFLASH_BTN_DOWN)
                { vff_index = (vff_index + 1) % vff_count; reload = 1; }
            if (pressed & VFLASH_BTN_UP)
                { vff_index = (vff_index + vff_count - 1) % vff_count; reload = 1; }
            if (pressed & VFLASH_BTN_GREEN)
                { vff_fmt = (vff_fmt + 1) % 4; reload = 1; }
            if (pressed & VFLASH_BTN_BLUE)
                { vff_sec = (vff_sec + 1) % 4; if (vff_sec == 3) vff_sec = -1; reload = 1; }
            if (vf->frame_count == 3) reload = 1;

            if (reload) {
                CDEntry *e = &vff_list[vff_index];
                uint8_t hdr[0x400];
                int rd = cdrom_read_file(vf->cd, e, hdr, 0, 0x400);
                if (rd >= 0x60 && hdr[0]=='v' && hdr[1]=='f' && hdr[2]=='D' && hdr[3]=='0') {
                    uint32_t nsec = *(uint32_t*)(hdr + 0x2C);
                    if (nsec > 8) nsec = 8;
                    /* Build section offset table */
                    uint32_t sec_off[8], sec_sz[8];
                    uint32_t cur = 0x400;
                    for (uint32_t si = 0; si < nsec; si++) {
                        sec_off[si] = cur;
                        sec_sz[si] = *(uint32_t*)(hdr + 0x34 + si * 0x10);
                        cur += sec_sz[si];
                    }
                    /* Pick section: auto = largest, or user-selected */
                    int pick = 0;
                    if (vff_sec >= 0 && (uint32_t)vff_sec < nsec) {
                        pick = vff_sec;
                    } else {
                        uint32_t best = 0;
                        for (uint32_t si = 0; si < nsec; si++)
                            if (sec_sz[si] > best) { best = sec_sz[si]; pick = (int)si; }
                    }
                    uint32_t frame_sz = 320 * 240 * 2;
                    int stride = (vff_fmt == 3) ? 512 : 320;
                    if (vff_fmt == 3) frame_sz = 512 * 240 * 2;
                    /* Composite display: background (last section) + foreground.
                     * Last section is usually a solid color fill.
                     * Middle section has XBGR1555 with 0=transparent. */
                    {
                        uint32_t frame_px = 320 * 240;
                        uint32_t frame_bytes = frame_px * 2;

                        /* Step 1: fill background from last section */
                        if (nsec >= 2 && sec_sz[nsec-1] >= 4) {
                            uint8_t bg[4];
                            cdrom_read_file(vf->cd, e, bg, sec_off[nsec-1], 4);
                            uint16_t bgc = bg[0] | (bg[1] << 8);
                            uint8_t br = (bgc & 0x1F) << 3;
                            uint8_t bg_ = ((bgc >> 5) & 0x1F) << 3;
                            uint8_t bb = ((bgc >> 10) & 0x1F) << 3;
                            uint32_t bgpx = 0xFF000000|((uint32_t)br<<16)|((uint32_t)bg_<<8)|bb;
                            for (uint32_t pi = 0; pi < frame_px; pi++)
                                vf->framebuf[pi] = bgpx;
                        }

                        /* VFF foreground data is tiled/compressed — can't display raw.
                         * Only the background fill color is directly usable. */
                        printf("[VFF] %d/%d: %s (bg+fg composite)\n",
                               vff_index+1, vff_count, e->name);
                    }
                    if (vf->wav_count > 0 && vf->audio && vf->audio->initialized) {
                        int wi = (vff_index * 7) % vf->wav_count;
                        play_wav_from_cd(vf, &vf->wav_list[wi]);
                    }
                }
            }
        }
    }

    /* Scan for BOOT.BIN callback pointers in µMORE area */
    if (vf->has_rom && vf->frame_count == 110 && vf->boot_phase >= 100) {
        printf("[CB-SCAN] Scanning for BOOT.BIN pointers in µMORE area...\n");
        /* Dump full SDRAM for Ghidra analysis */
        {
            FILE *df = fopen("/tmp/vflash_ram.bin", "wb");
            if (df) {
                fwrite(vf->ram, 1, VFLASH_RAM_SIZE, df);
                fclose(df);
                printf("[DUMP] SDRAM dumped to /tmp/vflash_ram.bin (%d MB)\n",
                       VFLASH_RAM_SIZE / (1024*1024));
            }
        }
        int found = 0;
        for (uint32_t a = 0x90000; a < 0xC00000; a += 4) {
            uint32_t v = *(uint32_t*)(vf->ram + a);
            if (v >= 0x10C00000 && v < 0x10E00000) {
                printf("[CB-SCAN] [0x%08X] = 0x%08X (BOOT.BIN ptr)\n",
                       0x10000000+a, v);
                found++;
                if (found > 30) { printf("[CB-SCAN] ... (truncated)\n"); break; }
            }
        }
        printf("[CB-SCAN] Found %d BOOT.BIN pointers\n", found);
        /* Inject BOOT.BIN game callback via Ghidra-discovered pointer.
         * FUN_10a18d9c stores callback at *[0x10BBD3C0] and userdata at *[0x10BC2640].
         * Game task calls FUN_10a18a04(0,0) which sets these to NULL.
         * We write BOOT.BIN game entry here so event dispatch calls it. */
        *(uint32_t*)(vf->ram + 0xBBD3C0) = 0x10CFAEA0; /* game_main entry */
        printf("[CB-INJECT] Set *[0x10BBD3C0] = 0x10CFAEA0 (game callback)\n");
        /* Ghidra: game task waits for *[0x10B05A18] > 7 AND *[0x10B05A1C] > 7.
         * These are service registration counters. Set to 8 to unblock. */
        *(uint32_t*)(vf->ram + 0xB05A18) = 8;
        *(uint32_t*)(vf->ram + 0xB05A1C) = 8;
        printf("[CB-INJECT] Set service counters to 8 (unblock game task wait)\n");
        /* Dump area around first cluster at 0x1009D160-0x1009D190 */
        printf("[CB-SCAN] Context around first cluster:\n");
        for (uint32_t d = 0x9D160; d < 0x9D1C0; d += 16)
            printf("[CB-SCAN] %08X: %08X %08X %08X %08X\n", 0x10000000+d,
                   *(uint32_t*)(vf->ram+d), *(uint32_t*)(vf->ram+d+4),
                   *(uint32_t*)(vf->ram+d+8), *(uint32_t*)(vf->ram+d+12));
    }

    /* Initialize kernel service dispatch vtable at [1000C76C].
     * ROM function at ROM+A1C8 fills this but gets skipped in our boot flow.
     * Contains function pointers for service lookup (+0x44) and request (+0x30). */
    if (vf->has_rom && vf->boot_phase >= 100 && *(uint32_t*)(vf->ram+0xC7B0) == 0) {
        *(uint32_t*)(vf->ram + 0xC798) = 0x10007A14;
        *(uint32_t*)(vf->ram + 0xC79C) = 0x10007878; /* service request */
        *(uint32_t*)(vf->ram + 0xC7A0) = 0x10007AB8;
        *(uint32_t*)(vf->ram + 0xC7A4) = 0x10007558;
        *(uint32_t*)(vf->ram + 0xC7A8) = 0x10007614;
        *(uint32_t*)(vf->ram + 0xC7AC) = 0x100077AC;
        *(uint32_t*)(vf->ram + 0xC7B0) = 0x10007E68; /* service lookup */
        printf("[VTABLE] Kernel service dispatch table initialized\n");
    }

    /* Auto-launch game task after µMORE init stabilizes. */
    if (vf->has_rom && vf->frame_count == 75 && vf->boot_phase >= 100) {
        /* Find game task: scan for $QCB with largest stack.
         * Relaxed check — don't require $QBT sub-magic (some games don't have it). */
        uint32_t entry = 0, sbase = 0, ssz = 0;
        {
            uint32_t best_a = 0, best_sz = 0;
            for (uint32_t a = 0x100000; a < 0xC00000; a += 4) {
                if (*(uint32_t*)(vf->ram+a) == 0x42435124) { /* $QCB */
                    uint32_t e = *(uint32_t*)(vf->ram+a+0x2C);
                    uint32_t s = *(uint32_t*)(vf->ram+a+0x34);
                    if (e >= 0x10090000 && e < 0x10E00000 && s > best_sz) {
                        best_sz = s; best_a = a;
                    }
                }
            }
            if (best_a) {
                entry = *(uint32_t*)(vf->ram+best_a+0x2C);
                sbase = *(uint32_t*)(vf->ram+best_a+0x30);
                ssz = best_sz;
            }
        }
        if (!entry) {
            /* Let kernel scheduler run — it will dispatch tasks natively.
             * Service entry is at [0xFFCC] = 0x10087160 (kernel scheduler). */
            entry = *(uint32_t*)(vf->ram + 0xFFCC);
            if (entry < 0x10000000 || entry >= 0x10100000)
                entry = 0x10087160;
            sbase = 0x10B6DB28;
            ssz = 0x20000;
            printf("[AUTO-LAUNCH] Using service dispatcher 0x%08X\n", entry);
        }
        {
            printf("[AUTO-LAUNCH] Game task: entry=%08X stack=%08X+%X\n", entry, sbase, ssz);
            /* Launch game task ENTRY — let init run with blocking skipped.
             * Only sleep/wait are skipped; mutex init runs normally. */
            /* Launch game task at service entry — let init run naturally.
             * BSS clear is handled by write protection + code restore. */
            /* Create game task TCB at 0x10F00100 BEFORE launching scheduler,
             * so the scheduler finds it in the ready list immediately. */
            {
                uint32_t tcb = 0xF00100;
                memset(vf->ram + tcb, 0, 0x100);
                *(uint32_t*)(vf->ram + tcb + 0x00) = 0x42435124; /* $QCB magic */
                *(uint32_t*)(vf->ram + tcb + 0x04) = 0x10000000 + tcb; /* next = self (circular) */
                *(uint32_t*)(vf->ram + tcb + 0x08) = 1;          /* priority */
                *(uint32_t*)(vf->ram + tcb + 0x0C) = 1;          /* status: active */
                *(uint32_t*)(vf->ram + tcb + 0x1C) = 0x42435424; /* $QBT stack marker */
                *(uint32_t*)(vf->ram + tcb + 0x20) = 4;          /* state: NU_READY */
                *(uint32_t*)(vf->ram + tcb + 0x28) = 10;         /* task ID */
                *(uint32_t*)(vf->ram + tcb + 0x2C) = 0x10C16CC0; /* entry point */
                *(uint32_t*)(vf->ram + tcb + 0x30) = 0x10FFD000; /* stack base */
                *(uint32_t*)(vf->ram + tcb + 0x34) = 0x20000;    /* stack size 128KB */
                /* Saved context at +0x58 for scheduler context switch */
                *(uint32_t*)(vf->ram + tcb + 0x58) = 0x10C16CC0;         /* saved PC */
                *(uint32_t*)(vf->ram + tcb + 0x5C) = 0x00000013;         /* saved CPSR: SVC */
                *(uint32_t*)(vf->ram + tcb + 0x60) = 0x10FFD000+0x20000-4; /* saved SP */
                /* Enqueue in Nucleus ready list at 0x100AC030 */
                *(uint32_t*)(vf->ram + 0xAC030) = 0x10000000 + tcb;
                *(uint32_t*)(vf->ram + 0xBC2BC0) = 1; /* dispatch flag */
                *(uint32_t*)(vf->ram + 0x3585E0) = 3; /* sched_state = running */
                *(uint32_t*)(vf->ram + 0x3585C0) = 0x10000000 + tcb; /* task list head */
                *(uint32_t*)(vf->ram + 0xACC78)  = 0x10000000 + tcb; /* task mgr pointer */
                printf("[AUTO-LAUNCH] Created game TCB at 0x%08X entry=0x10C16CC0\n",
                       0x10000000 + (unsigned)tcb);
            }
            /* Launch game task directly at 0x10C16CC0.
             * The native scheduler at 0x10087160 can't dispatch because its
             * µMORE service dependencies are in BSS (unpopulated).
             * HLE timer tick (per-frame) handles re-scheduling. */
            entry = 0x10C16CC0;
            /* Set game state flags:
             * [0x10B00A64] bit1: per-tick poll advances (µMORE services set this)
             * [0x10BBAE40] = 5: game ready (poll returns 1, enters game loop) */
            *(uint32_t*)(vf->ram + 0xB00A64) |= 2;
            *(uint32_t*)(vf->ram + 0xBBAE40) = 5;
            printf("[AUTO-LAUNCH] → Game entry at 0x%08X (direct, flag[B00A64]=%08X)\n",
                   entry, *(uint32_t*)(vf->ram + 0xB00A64));
            vf->cpu.r[15] = entry;
            vf->cpu.cpsr = 0x00000013; /* SVC, IRQ enabled */
            vf->cpu.r[13] = sbase + ssz - 4;
            vf->cpu.r[14] = 0x10FFF000;
            vf->timer.timer[0].load = 37500;
            vf->timer.timer[0].count = 37500;
            vf->timer.timer[0].ctrl = 0xE2;
            vf->timer.irq.enable |= 0x01;
            vf->boot_phase = 800;
            /* Backup ENTIRE RAM — game BSS clear will zero code+data.
             * We restore everything when BSS clear is detected. */
            if (!vf->rtos_backup) {
                vf->rtos_backup = malloc(VFLASH_RAM_SIZE);
                if (vf->rtos_backup) {
                    memcpy(vf->rtos_backup, vf->ram, VFLASH_RAM_SIZE);
                    printf("[RAM-BACKUP] Saved full 16MB RAM state\n");
                    /* Dump backup for analysis */
                    FILE *bf = fopen("/tmp/vflash_backup.bin", "wb");
                    if (bf) { fwrite(vf->rtos_backup, 1, VFLASH_RAM_SIZE, bf); fclose(bf); }
                    printf("[RAM-BACKUP] [109D1BD0]=%08X [109D11E0]=%08X\n",
                           *(uint32_t*)(vf->ram + 0x9D1BD0),
                           *(uint32_t*)(vf->ram + 0x9D11E0));
                }
                /* Flush JIT cache so all blocks get recompiled with
                 * write protection enabled (boot_phase >= 800) */
                if (vf->jit) {
                    jit_flush(vf->jit);
                    printf("[JIT] Flushed cache for write protection\n");
                }
            }
        }
    }

    /* Restore RTOS code if game zeroed it — do it silently without JIT flush.
     * The game's BSS clear zeros the RTOS area every frame via STR loops.
     * We restore it back but DON'T flush JIT — the JIT blocks are still valid
     * since the code content is restored to the same values. */
    /* Detect BSS clear loop: PC advances through 109D0000-10A10000
     * zeroing everything. When detected, restore code and skip ahead. */
    if (vf->rtos_backup && vf->boot_phase >= 800) {
        uint32_t pc = vf->cpu.r[15];

        /* Per-frame sanity: fix CPSR if mode bits are invalid.
         * Game code or HLE stubs can corrupt CPSR to 0x00 (invalid mode).
         * Force SVC mode (0x13) with IRQ+FIQ disabled. */
        {
            uint32_t mode = vf->cpu.cpsr & 0x1F;
            if (mode != 0x10 && mode != 0x11 && mode != 0x12 &&
                mode != 0x13 && mode != 0x17 && mode != 0x1B && mode != 0x1F) {
                vf->cpu.cpsr = 0x000000D3; /* SVC, I+F disabled */
            }
        }

        /* Per-frame RECOVER: redirect escaped PC to game loop.
         * Replaces the old per-slice escape check which was too aggressive
         * (SpongeBob/Scooby crashed at boot_phase 800 during init).
         * Per-frame gives init code a full frame to settle before checking. */
        if (vf->boot_phase >= 900) {
            int escaped = (pc >= 0xB8000000u) ||
                          (pc < 0x10000000u && pc > 0x1000u) ||
                          (pc >= 0x11000000u && pc < 0xB8000000u) ||
                          (pc < 0x10090000 && pc >= 0x10000000) ||
                          (pc > 0x10F00000 && pc < 0x11000000 &&
                           pc != 0x10FFF00C && pc != 0x10FFF000);
            if (escaped) {
                static int recover_log = 0;
                if (recover_log < 10) {
                    printf("[RECOVER] PC=0x%08X escaped → redirect to game loop\n", pc);
                    recover_log++;
                }
                /* For BOOT.BIN games, redirect to idle (game loop is in BOOT.BIN area).
                 * For relocated games, redirect to 0x109D1CE0 (game loop entry). */
                int bootbin_game = (vf->saved_game_entry >= 0x10C00000 && vf->saved_game_entry < 0x10E00000);
                if (bootbin_game) {
                    vf->cpu.r[15] = 0x10FFF000; /* idle — game will resume via timer */
                } else {
                    vf->cpu.r[15] = 0x109D1CE0;
                }
                vf->cpu.cpsr = 0x000000D3;
            }
        }

        /* BOOT.BIN games manage their own state — skip forced game flags,
         * render context, and button handler calls that would corrupt them.
         * Only apply per-frame state manipulation for relocated games. */
        int is_bootbin = (vf->saved_game_entry >= 0x10C00000 &&
                          vf->saved_game_entry < 0x10E00000);

        if (!is_bootbin) {
            /* Ensure game loop code is intact (BSS clear may zero it).
             * Restore from backup (post-init BOOT.BIN with correct BL targets). */
            if (vf->boot_phase >= 900 &&
                *(uint32_t*)(vf->ram + 0x9D1CE0) == 0 && vf->rtos_backup) {
                memcpy(vf->ram + 0x9D0000, vf->rtos_backup + 0xC0B014,
                       0xA80000 - 0x9D0000);
            }

            /* After any restore, re-set game flags (backup has them at 0).
             * Also reset render context so render_processing doesn't skip. */
            *(uint32_t*)(vf->ram + 0xBE3C40) = 0;
            *(uint32_t*)(vf->ram + 0xBE3C44) = 1;
            *(uint32_t*)(vf->ram + 0xBE3C4C) = (uint32_t)vf->frame_count;
            *(uint32_t*)(vf->ram + 0xBE3C50) = (uint32_t)vf->frame_count - 1;
            vf->ram[0xBE3C60] = 1;
            if (*(uint32_t*)(vf->ram + 0xBBEAE0) == 0)
                *(uint32_t*)(vf->ram + 0xBBEAE0) = 0x00010001;
            *(uint32_t*)(vf->ram + 0xAFDE50) = vf->input;
            /* Call ALL 6 button handler callbacks per-frame. */
            if (vf->boot_phase >= 900) {
                static const uint32_t btn_handlers[] = {
                    0x109D17A4, 0x109D18A8, 0x109D19D0,
                    0x109D1A20, 0x109D1A9C, 0x109D1B34, 0
                };
                uint32_t save_regs[16];
                memcpy(save_regs, vf->cpu.r, sizeof(save_regs));
                uint32_t save_cpsr = vf->cpu.cpsr;
                for (int hi = 0; btn_handlers[hi]; hi++) {
                    vf->cpu.r[15] = btn_handlers[hi];
                    vf->cpu.r[14] = 0x10FFF000;
                    vf->cpu.cpsr = 0x000000D3;
                    vf->cpu.r[0] = vf->input;
                    int budget = 5000;
                    while (budget > 0 && vf->cpu.r[15] != 0x10FFF000) {
                        uint64_t cb = vf->cpu.cycles;
                        arm9_step(&vf->cpu);
                        budget -= (int)(vf->cpu.cycles - cb);
                        if (budget < 0) break;
                        uint32_t vpc = vf->cpu.r[15];
                        if (vpc >= 0xB8000000u || (vpc < 0x10000000u && vpc > 0x1000u))
                            break;
                    }
                }
                memcpy(vf->cpu.r, save_regs, sizeof(save_regs));
                vf->cpu.cpsr = save_cpsr;
            }
            vf->lcd.upbase = 0x10BBEAE0;
            if (!vf->dc_vblank_cb)
                vf->dc_vblank_cb = 0x109D1CE0;
            *(uint32_t*)(vf->ram + 0xBE3EC0) = 1;
            *(uint16_t*)(vf->ram + 0xBE49E0) = 1;
            *(uint32_t*)(vf->ram + 0xB009C4) = 1;
            *(uint32_t*)(vf->ram + 0xB902C0) = 3;
            *(uint32_t*)(vf->ram + 0xBE3EC0) = 1;
        } else {
            /* Force IRQ delivery per-frame for BOOT.BIN games.
             * ROM vector ROM[0xD44] is patched at bp900 to point to 0x10FFF040
             * (timer handler in high SDRAM, safe from calibration test). */
            if (vf->boot_phase >= 900 && ztimer_irq_pending(&vf->timer)) {
                uint32_t save_cpsr = vf->cpu.cpsr;
                vf->cpu.cpsr &= ~0x80; /* unmask IRQ */
                if (vf->cpu.r13_irq < 0x10000000u || vf->cpu.r13_irq > 0x10FFFFFFu)
                    vf->cpu.r13_irq = 0x10800000;
                arm9_irq(&vf->cpu);
                /* Don't restore CPSR — IRQ handler will do SUBS PC,LR,#4
                 * which restores SPSR (which has the original CPSR saved). */
            }
        }
    }

    /* Force game pre-loop after init gets stuck.
     * Jump to 0x109D1CA8 (buffer clear + display setup) instead of
     * directly to game loop. This lets the game set up its own
     * framebuffer and render state before entering the main loop. */
    if (vf->has_rom && vf->boot_phase == 800 &&
        vf->frame_count >= 100 && vf->frame_count <= 200) {
        /* Set asset loading counters to > 7 (game waits for this) */
        *(uint32_t*)(vf->ram + 0xB05A18) = 8;  /* counter1 */
        *(uint32_t*)(vf->ram + 0xB05A1C) = 8;  /* counter2 */
        /* Set game state for active gameplay */
        *(uint32_t*)(vf->ram + 0xB009C4) = 1;  /* game_state = active */
        *(uint32_t*)(vf->ram + 0xB902C0) = 3;  /* game_mode = gameplay */
        /* Load VFF scene data to target RAM addresses.
         * VFF header: +10=entry, +14=param, +18/1C=callbacks, +20=init_cb
         * sec[0]=ARM code, sec[1]=tile descriptors+gfx, sec[2]=compressed tilemap */
        uint32_t vff_entry = 0, vff_init_cb = 0;
        if (vf->cd && vf->cd->is_open) {
            CDEntry ve;
            if (cdrom_find_file_any(vf->cd, "MAIN.VFF", &ve)) {
                uint8_t vhdr[0x60];
                cdrom_read_file(vf->cd, &ve, vhdr, 0, 0x60);
                if (vhdr[0]=='v' && vhdr[1]=='f') {
                    uint32_t nsec = *(uint32_t*)(vhdr + 0x2C);
                    vff_entry = *(uint32_t*)(vhdr + 0x10);
                    vff_init_cb = *(uint32_t*)(vhdr + 0x20);
                    printf("[VFF] entry=0x%08X init_cb=0x%08X nsec=%d\n",
                           vff_entry, vff_init_cb, nsec);
                    uint32_t foff = 0x400; /* header padded to 0x400 */
                    for (uint32_t si = 0; si < nsec && si < 3; si++) {
                        uint32_t ssz = *(uint32_t*)(vhdr + 0x34 + si*0x10);
                        uint32_t dst = *(uint32_t*)(vhdr + 0x30 + si*0x10);
                        if (dst >= 0x10000000 && dst + ssz <= 0x10000000 + VFLASH_RAM_SIZE) {
                            int rd = cdrom_read_file(vf->cd, &ve,
                                        vf->ram + (dst - 0x10000000), foff, ssz);
                            printf("[VFF] sec[%d] %dKB → %08X (load_addr)\n", si, rd/1024, dst);
                        }
                        foff += ssz;
                    }
                }
            }
        }

        /* Execute VFF sec[0] ARM code to initialize scene descriptors.
         * Find entry by scanning for first ARM function prologue (PUSH/STMDB).
         * Called with R0=1 (init), R1=0xFFFF (all scenes). */
        {
            /* Scan sec[0] for first STMDB SP!, {regs} (E92Dxxxx) prologue */
            uint32_t s0_addr = 0x104E2000; /* default sec[0] load address */
            uint32_t s0_size = 0x20000;    /* default scan range */
            if (vff_entry) {
                /* entry_table from VFF header → sec[0] end */
                s0_addr = vff_entry - (vff_entry - 0x104E2000);
                /* Rough: use first 16KB for prologue scan */
            }
            /* Scan for scene entry: LDR Rx, =0xFFFF followed by PUSH.
             * The scene init function starts with LDR R12, =0xFFFF; PUSH {regs}
             * and is called with R0=1 (init), R1=scene_id. */
            uint32_t entry = 0;
            for (uint32_t scan = 0; scan < 0x8000; scan += 4) {
                uint32_t off = s0_addr - 0x10000000 + scan;
                if (off + 8 > VFLASH_RAM_SIZE) break;
                uint32_t i0 = *(uint32_t*)(vf->ram + off);
                uint32_t i1 = *(uint32_t*)(vf->ram + off + 4);
                /* LDR Rx, [PC, #imm] where literal = 0x0000FFFF */
                if ((i0 & 0x0F7F0000) == 0x059F0000 &&
                    (i1 & 0xFFFF0000) == 0xE92D0000) {
                    /* Verify literal value is 0xFFFF */
                    int Rd = (i0 >> 12) & 0xF;
                    int imm = i0 & 0xFFF;
                    int U = (i0 >> 23) & 1;
                    uint32_t lit_addr = off + 8 + (U ? imm : -imm);
                    if (lit_addr + 4 <= VFLASH_RAM_SIZE) {
                        uint32_t lit = *(uint32_t*)(vf->ram + lit_addr);
                        if (lit == 0x0000FFFF || lit == 0xFFFFFFFF) {
                            entry = s0_addr + scan;
                            printf("[VFF-EXEC] Entry: LDR R%d,=0x%X + PUSH at 0x%08X\n",
                                   Rd, lit, entry);
                            break;
                        }
                    }
                }
            }
            if (!entry) entry = s0_addr + 0x400; /* fallback */
            uint32_t first_i = *(uint32_t*)(vf->ram + (entry - 0x10000000));
            if (first_i != 0) {
                uint32_t spc=vf->cpu.r[15], ssp=vf->cpu.r[13];
                uint32_t slr=vf->cpu.r[14], scp=vf->cpu.cpsr;
                vf->cpu.r[0] = 1; vf->cpu.r[1] = 0xFFFF;
                vf->cpu.r[15] = entry;
                vf->cpu.r[14] = 0x10FFF000;
                vf->cpu.r[13] = 0x10B8D000;
                vf->cpu.cpsr = 0x000000D3;
                int si;
                for (si = 0; si < 5000000; si++) {
                    arm9_step(&vf->cpu);
                    if (vf->cpu.r[15] == 0x10FFF000) break;
                }
                printf("[VFF-EXEC] Scene init: %d steps R0=%08X\n", si, vf->cpu.r[0]);
                /* Check what was written to scene table */
                printf("[VFF-EXEC] [10500808]=%08X [1050080C]=%08X [10500810]=%08X\n",
                       *(uint32_t*)(vf->ram+0x500808),
                       *(uint32_t*)(vf->ram+0x50080C),
                       *(uint32_t*)(vf->ram+0x500810));
                /* Dump RAM for analysis */
                {
                    FILE *df = fopen("/tmp/vflash_ram2.bin", "wb");
                    if (df) {
                        fwrite(vf->ram, 1, VFLASH_RAM_SIZE, df);
                        fclose(df);
                        printf("[DUMP] RAM dumped to /tmp/vflash_ram2.bin\n");
                    }
                }
                /* Dump dispatch table entries */
                {
                    uint32_t db = (vff_entry ? vff_entry : 0x10500800) - 0x10000000;
                    for (uint32_t di = 0; di < 0x400; di += 0x10) {
                        uint32_t a = *(uint32_t*)(vf->ram + db + di);
                        uint32_t b = *(uint32_t*)(vf->ram + db + di + 4);
                        uint32_t c = *(uint32_t*)(vf->ram + db + di + 8);
                        uint32_t d = *(uint32_t*)(vf->ram + db + di + 12);
                        if (a == 0 && b == 0 && c == 0 && d == 0) continue;
                        printf("[VFF-DT] +%03X: %08X %08X %08X %08X\n",
                               di, a, b, c, d);
                    }
                }
                /* Call ALL scene callbacks from dispatch table.
                 * Each entry is 0x30 bytes: +0x00=id, +0x04=param, ..., +0x14=callback.
                 * The dispatch table was populated by the scene init call above. */
                uint32_t dt_base = vff_entry ? vff_entry : 0x10500800;
                uint32_t dt_off = dt_base - 0x10000000;
                uint32_t bl_targets[256];
                int bl_count = 0;
                int total_cb_steps = 0;
                int cb_called = 0;
                /* Scan dispatch table for callback pointers and call each one */
                if (dt_off + 0x1000 < VFLASH_RAM_SIZE) {
                    for (uint32_t di = 0; di < 0xDA0; di += 4) {
                        uint32_t v = *(uint32_t*)(vf->ram + dt_off + di);
                        if (v >= s0_addr && v < s0_addr + 0x20000) {
                            /* This is a callback pointer — call it */
                            vf->cpu.r[0] = 0x10300000;
                            vf->cpu.r[1] = di;
                            vf->cpu.r[15] = v;
                            vf->cpu.r[14] = 0x10FFF000;
                            vf->cpu.r[13] = 0x10B8D000;
                            vf->cpu.cpsr = 0x000000D3;
                            for (si = 0; si < 500000; si++) {
                                uint32_t pc_before = vf->cpu.r[15];
                                arm9_step(&vf->cpu);
                                uint32_t pc_after = vf->cpu.r[15];
                                if (pc_before >= 0x10100000 && pc_before < 0x10700000 &&
                                    pc_after >= 0x10A00000 && pc_after < 0x10D00000) {
                                    int found = 0;
                                    for (int bi = 0; bi < bl_count; bi++)
                                        if (bl_targets[bi] == pc_after) { found = 1; break; }
                                    if (!found && bl_count < 256) {
                                        bl_targets[bl_count++] = pc_after;
                                        printf("[VFF-BL] → %08X R0=%08X R1=%08X R2=%08X\n",
                                               pc_after, vf->cpu.r[0], vf->cpu.r[1], vf->cpu.r[2]);
                                    }
                                }
                                if (pc_after == 0x10FFF000 || pc_after == 0 ||
                                    (pc_after > 0x11000000 && pc_after < 0x80000000)) break;
                            }
                            total_cb_steps += si;
                            cb_called++;
                        }
                    }
                }
                printf("[VFF-EXEC] Called %d scene callbacks (%d steps, %d BL targets)\n",
                       cb_called, total_cb_steps, bl_count);
                printf("[VFF-BL] Total unique BOOT.BIN calls: %d\n", bl_count);
                printf("[VFF-SCENE] Entity callback: %d steps\n", si);
                /* Log entity vtable values after scene callback */
                printf("[VFF-SCENE] Entity data after callback:\n");
                int real_vt = 0, dummy_vt = 0, data_vt = 0;
                for (uint32_t ea = 0x320000; ea < 0x400000; ea += 64) {
                    uint32_t v = *(uint32_t*)(vf->ram + ea);
                    if (v == 0) continue;
                    if (v == 0x10310800) { dummy_vt++; continue; }
                    if (v >= 0x10A00000 && v < 0x10E00000) {
                        if (real_vt < 10)
                            printf("  [%08X]+0 = %08X ← REAL VTABLE!\n", 0x10000000+ea, v);
                        real_vt++;
                    } else if (v >= 0x104E0000 && v < 0x10600000) {
                        if (real_vt < 10)
                            printf("  [%08X]+0 = %08X ← VFF VTABLE!\n", 0x10000000+ea, v);
                        real_vt++;
                    } else {
                        data_vt++;
                    }
                }
                printf("[VFF-SCENE] vtables: %d real, %d dummy, %d data\n",
                       real_vt, dummy_vt, data_vt);
                /* Only restore dummy vtable on HLE alloc area (0x320000-0x390000).
                 * Leave native alloc area (0x390000+) alone — may have real vtables. */
                for (uint32_t ea = 0x320000; ea < 0x390000; ea += 0x10) {
                    uint32_t v = *(uint32_t*)(vf->ram + ea);
                    if (v != 0 && v != 0x10310800) {
                        *(uint32_t*)(vf->ram + ea) = 0x10310800;
                    }
                }
                printf("[VFF-SCENE] Restored dummy vtable on HLE alloc area only\n");
                vf->cpu.r[15]=spc; vf->cpu.r[13]=ssp;
                vf->cpu.r[14]=slr; vf->cpu.cpsr=scp;
            }
        }

        /* Load VFF to backup buffer */
        if (0 && vf->cd && vf->cd->is_open) {
            CDEntry vff_e;
            const char *vff_names[] = {"MAIN.VFF","CWMENU.VFF","CW1A.VFF","OPENING.VFF",NULL};
            for (int vi = 0; vff_names[vi]; vi++) {
                if (cdrom_find_file_any(vf->cd, vff_names[vi], &vff_e)) {
                    /* Load VFF to a temp buffer in game RAM (0x10200000) */
                    uint32_t load_sz = vff_e.size;
                    if (load_sz > 0x100000) load_sz = 0x100000; /* max 1MB */
                    int rd = cdrom_read_file(vf->cd, &vff_e, vf->ram + 0x200000, 0, load_sz);
                    if (rd > 0) {
                        printf("[CD-LOAD] Loaded %s (%d bytes) to RAM+0x200000\n",
                               vff_e.name, rd);
                    }
                    break;
                }
            }
        }
        /* Load PTX game artwork to BOTH the old render buffer (0x10048000)
         * AND the render pipeline's framebuffer (0x10BBEAE0).
         * Decode XBGR1555 → RGB565 for LCD blit. */
        memset(vf->ram + 0xBBEAE0, 0, 320*240*2); /* clear render FB */
        if (vf->cd && vf->cd->is_open && vf->ptx_count > 0) {
            CDEntry *pe = &vf->ptx_list[0];
            uint8_t *ptx_buf = malloc(pe->size);
            if (ptx_buf) {
                int rd = cdrom_read_file(vf->cd, pe, ptx_buf, 0, pe->size);
                if (rd > 44) {
                    uint32_t hs = *(uint32_t*)ptx_buf;
                    if (hs >= 12 && hs <= 256 && hs < (uint32_t)rd) {
                        const uint16_t *src = (const uint16_t*)(ptx_buf + hs);
                        uint16_t *fb = (uint16_t*)(vf->ram + 0x48000);
                        uint32_t pw = 512;
                        uint32_t total_rows = ((uint32_t)rd - hs) / (pw * 2);
                        uint32_t src_h = total_rows / 2;
                        if (src_h > 240) src_h = 240;
                        for (uint32_t dy = 0; dy < 240; dy++) {
                            uint32_t sy = dy * src_h / 240;
                            for (uint32_t dx = 0; dx < 320; dx++) {
                                uint16_t p = src[sy * 2 * pw + dx];
                                uint16_t r = (p & 0x1F);
                                uint16_t g = ((p >> 5) & 0x1F);
                                uint16_t b = ((p >> 10) & 0x1F);
                                fb[dy * 320 + dx] = (r << 11) | (g << 6) | b;
                            }
                        }
                        /* Also copy to render pipeline's framebuffer */
                        uint16_t *fb2 = (uint16_t*)(vf->ram + 0xBBEAE0);
                        memcpy(fb2, fb, 320*240*2);
                        printf("[PTX-FB] Loaded %s to render buffer + 0x10BBEAE0\n", pe->name);
                    }
                }
                free(ptx_buf);
            }
        }
        /* Jump to game_setup (109D5B1C) instead of game loop.
         * With counters set, game entry would naturally reach:
         *   game_setup → first_tick → buffer_clear → display → walker → game loop
         * But game_setup + walker are slow, so jump to buffer clear (109D1C9C). */
        /* Clear render buffer ourselves (HLE memset) */
        memset(vf->ram + 0x48000, 0, 0xF0000);
        /* Create dummy entities so render subsystem doesn't loop on NULL.
         * Entity struct: [+0]=vtable_ptr, [+4]=id, [+8]=status_byte(1=ready).
         * Place at 0x10300000 (unused RAM area). */
        {
            uint32_t ent_base = 0x300000; /* RAM offset */
            /* Create 8 dummy entities at 0x200 bytes apart */
            for (int ei = 0; ei < 8; ei++) {
                uint32_t eo = ent_base + ei * 0x200;
                memset(vf->ram + eo, 0, 0x200);
                /* Status byte at +8 = 1 (ready/done) */
                vf->ram[eo + 8] = 1;
                /* Also set bytes at common offsets to non-zero */
                vf->ram[eo + 5] = 1;
                *(uint32_t*)(vf->ram + eo + 0x104) = 0x84; /* state=done */
            }
            /* Set render_ctx to have entity pointer */
            uint32_t ent_addr = 0x10000000 + ent_base;
            *(uint32_t*)(vf->ram + 0xBE3C40) = 0;       /* ctx[0] still 0 */
            *(uint32_t*)(vf->ram + 0xBE3C44) = 1;       /* ctx[1] = frame */
            *(uint32_t*)(vf->ram + 0xBE3C4C) = 0;       /* ctx[3] = no flags yet */
            /* Set render state bytes that render loops poll.
             * 10BE3CA0 (render_ctx+0x60): polled by 10A73924.
             * 10BE49E0: another render struct polled in loops.
             * Set all bytes in render_ctx to non-zero "ready" state. */
            memset(vf->ram + 0xBE3C40, 1, 0x100);
            /* Also set the second render struct area */
            memset(vf->ram + 0xBE49E0, 1, 0x100);
            /* Restore specific ctx values (memset fills all with 0x01010101).
             * Pointer fields MUST be set to valid addresses or 0 (NULL). */
            *(uint32_t*)(vf->ram + 0xBE3C40) = 0;  /* ctx[0] = no crash */
            *(uint32_t*)(vf->ram + 0xBE3C44) = 1;  /* ctx[1] = frame */
            *(uint32_t*)(vf->ram + 0xBE3C4C) = 0;  /* ctx[3] = current frame */
            *(uint32_t*)(vf->ram + 0xBE3C50) = 0;  /* ctx[4] = last processed */
            /* Display list pointers — NULL until entities exist */
            *(uint32_t*)(vf->ram + 0xBE3D20) = 0;  /* display list ptr */
            *(uint32_t*)(vf->ram + 0xBE3D24) = 0;  /* display list array */
            *(uint32_t*)(vf->ram + 0xBE3D28) = 0;  /* count */
            *(uint32_t*)(vf->ram + 0xBE3D2C) = 0;
            *(uint32_t*)(vf->ram + 0xBE3D30) = 0;
            *(uint32_t*)(vf->ram + 0xBE3D34) = 0;
            *(uint32_t*)(vf->ram + 0xBE3D38) = 0;
            /* Framebuffer address for render pipeline */
            *(uint32_t*)(vf->ram + 0xBBEAE0) = 0;  /* clear sentinel */
            printf("[ENTITY] Created 8 dummy entities at %08X\n", ent_addr);
        }
        /* Relocate game code — ONLY if game runs from relocated area (0x109Dxxxx).
         * Some games (Incredibles) run directly from BOOT.BIN (0x10C0xxxx).
         * Skip relocation for those — it would overwrite their running code! */
        int game_in_bootbin = (vf->cpu.r[15] >= 0x10C00000 && vf->cpu.r[15] < 0x10E00000);
        /* RELOC-GAMELOOP: restore utility code for ALL games (including BOOT.BIN).
         * BOOT.BIN games (Incredibles) still need utility functions at 0x109Dxxxx
         * because their switch case handlers BL into this area. */
        if (vf->rtos_backup) {
            uint32_t dst = 0x9D0000;
            uint32_t src = 0xC0B014; /* BOOT.BIN in RAM: 0x10C00000+0xB0014 */
            uint32_t sz = 0xA80000 - 0x9D0000; /* 704KB */
            if (src + sz <= VFLASH_RAM_SIZE) {
                memcpy(vf->ram + dst, vf->rtos_backup + src, sz);
                uint32_t gl_first = *(uint32_t*)(vf->ram + 0x9D1CE0);
                printf("[RELOC-GAMELOOP] Copied %d bytes from backup: %08X→%08X"
                       " (game_loop[0]=%08X)\n",
                       sz, 0x10000000+src, 0x10000000+dst, gl_first);
            }
        }
        if (!game_in_bootbin) {
            vf->cpu.r[15] = 0x109D1CE0;
            vf->cpu.cpsr = 0x000000D3;
            vf->timer.timer[0].ctrl = 0;
            vf->timer.irq.enable = 0;
            vf->timer.irq.status = 0;
        } else {
            /* BOOT.BIN games: keep PC as-is, preserve timer for scheduler */
            printf("[GAME] Running from BOOT.BIN at PC=0x%08X — PC+timer preserved\n",
                   vf->cpu.r[15]);
        }
        /* RELOC-RENDER: restore render/engine code for ALL games.
         * BOOT.BIN games also call render functions in this area. */
        {
            uint32_t src = 0xCBB014;
            uint32_t dst = 0xA80000;
            uint32_t sz  = 0x8DB0C;
            uint8_t *copy_src = vf->rtos_backup;
            const char *src_name = "backup";
            /* Check if backup has code at this offset */
            if (copy_src && *(uint32_t*)(copy_src + src) == 0) {
                /* Backup area zeroed by BSS clear — fall back to CD */
                copy_src = NULL;
            }
            if (copy_src) {
                memcpy(vf->ram + dst, copy_src + src, sz);
            } else if (vf->cd && vf->cd->is_open) {
                CDEntry boot_e;
                if (cdrom_find_file_any(vf->cd, "BOOT.BIN", &boot_e)) {
                    cdrom_read_file(vf->cd, &boot_e,
                        vf->ram + dst, src - 0xC00000, sz);
                }
                src_name = "CD";
            }
            uint32_t first = *(uint32_t*)(vf->ram + dst + 0x9100);
            printf("[RELOC-RENDER] Copied %d bytes from %s: %08X→%08X"
                   " (render_processing[0]=%08X)\n",
                   sz, src_name, 0x10000000+src, 0x10000000+dst, first);
        }
        /* RELOC-FIX: copy 73 unrelocated functions for ALL games. */
        {
            uint32_t src = 0xD48B20;  /* 10D48B20: first unrelocated function */
            uint32_t dst = 0xB0DB0C;  /* 10B0DB0C: relocation target */
            uint32_t sz  = 0x18DE0;   /* 101,856 bytes covering all 73 functions */
            if (src + sz <= VFLASH_RAM_SIZE && dst + sz <= VFLASH_RAM_SIZE) {
                /* Verify source has ARM code (first function should start with PUSH) */
                uint32_t first = *(uint32_t*)(vf->ram + src);
                if ((first & 0xFFFF0000) == 0xE92D0000 || /* PUSH */
                    (first & 0xFFFFF000) == 0xE24DD000 || /* SUB SP */
                    (first & 0xFFFF0000) == 0xE52D0000) { /* STR ..,[SP] */
                    memcpy(vf->ram + dst, vf->ram + src, sz);
                    printf("[RELOC-FIX] Copied %d bytes: %08X→%08X (73 functions)\n",
                           sz, 0x10000000+src, 0x10000000+dst);
                } else {
                    printf("[RELOC-FIX] Source %08X has %08X (not a function prologue)\n",
                           0x10000000+src, first);
                }
            }
        }
        /* Set up LCD controller. */
        vf->lcd.control = 0x082D; /* 16bpp565, TFT, power, enabled */
        /* Fix IRQ vector for BOOT.BIN games: SDRAM vectors get corrupted by
         * calibration test patterns (0x55555555). Patch ROM directly:
         * ROM[0xD44] is the pool value loaded by ROM[0x18] (LDR PC,[PC,#0xD24]).
         * Point it to our timer handler in high SDRAM (never overwritten). */
        {
            uint32_t h = 0xFFF040;
            /* Install timer IRQ handler at 0x10FFF040 */
            if (*(uint32_t*)(vf->ram + h) == 0) {
                uint32_t p = h;
                *(uint32_t*)(vf->ram + p) = 0xE92D500F; p += 4; /* PUSH {R0-R3,R12,LR} */
                *(uint32_t*)(vf->ram + p) = 0xE59F0008; p += 4; /* LDR R0,[PC,#8] */
                *(uint32_t*)(vf->ram + p) = 0xE3A01001; p += 4; /* MOV R1,#1 */
                *(uint32_t*)(vf->ram + p) = 0xE5801000; p += 4; /* STR R1,[R0] */
                *(uint32_t*)(vf->ram + p) = 0xE8BD500F; p += 4; /* POP {R0-R3,R12,LR} */
                *(uint32_t*)(vf->ram + p) = 0xE25EF004; p += 4; /* SUBS PC,LR,#4 */
                *(uint32_t*)(vf->ram + p) = 0x9001000C; p += 4; /* pool: timer IntClr */
            }
            /* Patch ROM vector pool to point directly to handler */
            if (vf->has_rom && vf->rom_size > 0xD48) {
                *(uint32_t*)(vf->rom + 0xD44) = 0x10000000 + h;
                printf("[IRQ-ROM] Patched ROM[0xD44] → 0x%08X (timer handler)\n",
                       0x10000000 + h);
            }
        }
        vf->boot_phase = 900;
    }

    /* Game display: show PTX artwork from disc while game loop runs.
     * The native render pipeline needs entity system (TODO).
     * For now, display PTX sprite sheets as game scene visuals. */
    if (vf->boot_phase >= 900) {
        static uint16_t *ptx_data = NULL;
        static int ptx_w = 0, ptx_h = 0;
        static int ptx_idx = 0;
        if (!ptx_data && vf->ptx_count > 0 && vf->cd) {
            /* Load first large PTX image (scene artwork) */
            for (int pi = 0; pi < vf->ptx_count; pi++) {
                CDEntry *pe = &vf->ptx_list[pi];
                if (pe->size < 50000) continue; /* skip small UI elements */
                uint8_t *buf = malloc(pe->size);
                if (!buf) continue;
                int rd = cdrom_read_file(vf->cd, pe, buf, 0, pe->size);
                if (rd > 44) {
                    uint32_t hs = *(uint32_t*)buf;
                    if (hs >= 12 && hs <= 256 && hs < (uint32_t)rd) {
                        uint32_t data_bytes = (uint32_t)rd - hs;
                        uint32_t rows = data_bytes / 1024;
                        ptx_w = 512;
                        ptx_h = rows;
                        ptx_data = malloc(data_bytes);
                        if (ptx_data) {
                            memcpy(ptx_data, buf + hs, data_bytes);
                            ptx_idx = pi;
                            printf("[GAME-DISPLAY] Loaded PTX #%d (%dx%d)\n",
                                   pi, ptx_w, ptx_h);
                        }
                    }
                }
                free(buf);
                if (ptx_data) break;
            }
        }
        if (ptx_data) {
            /* Clear to desert brown (Cars themed) */
            for (int i = 0; i < VFLASH_SCREEN_W * VFLASH_SCREEN_H; i++)
                vf->framebuf[i] = 0xFF8B6914;
            /* Display PTX scaled to fit 320x240 screen.
             * PTX uses interlaced rows (even rows = image data).
             * Stride is 512 pixels per row. */
            int src_h = ptx_h / 2; /* actual image height (even rows only) */
            for (int y = 0; y < 240; y++) {
                int sy = y * src_h / 240;
                if (sy >= src_h) sy = src_h - 1;
                for (int x = 0; x < 320; x++) {
                    int sx = x;
                    if (sx >= ptx_w) sx = ptx_w - 1;
                    uint16_t p = ptx_data[sy * 2 * 512 + sx]; /* even rows */
                    if (p == 0) continue; /* transparent */
                    /* XBGR1555: R=bits0-4, G=bits5-9, B=bits10-14
                     * Expand 5-bit to 8-bit by replicating top bits into bottom */
                    uint8_t r5 = (p & 0x1F), g5 = ((p >> 5) & 0x1F), b5 = ((p >> 10) & 0x1F);
                    uint8_t r = (r5 << 3) | (r5 >> 2);
                    uint8_t g = (g5 << 3) | (g5 >> 2);
                    uint8_t b = (b5 << 3) | (b5 >> 2);
                    vf->framebuf[y * VFLASH_SCREEN_W + x] =
                        0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                }
            }
            /* Render multi-layer VFF scene from decompressed tile data.
             * sec[1] tiles at 0x10240000 (sky/ground layers)
             * sec[2] tiles at 0x101B8000 (horizon/detail layers)
             * Each layer uses intensity modulation with a base color. */
            int ent_count = 0;
            uint32_t tile_off = 0x240000;
            uint32_t sec2_off = 0x1B8000;
            int nz = 0;
            for (uint32_t ti = 0; ti < 512*256 && ti + tile_off < VFLASH_RAM_SIZE; ti++)
                if (vf->ram[tile_off + ti]) nz++;
            if (nz > 1000) {
                /* HLE multi-layer scene renderer.
                 * Reads dispatch table at 0x10500800 for 16 render layers.
                 * Each layer has: position (X,Y), tile data pointer, base color.
                 * Tiles are 8bpp intensity maps composited with per-layer colors.
                 * Layer order: back-to-front (Y=0=sky → Y=192=ground). */
                static const struct { uint8_t r,g,b; } layer_colors[16] = {
                    {140,100, 60}, /* 0: ground/terrain (Y=192) */
                    {180,150,100}, /* 1: mid terrain (Y=64) */
                    {200,170,120}, /* 2: horizon (Y=128) */
                    { 80,140, 60}, /* 3: vegetation (Y=192) */
                    {160,120, 70}, /* 4: road surface (Y=192) */
                    {100,160,220}, /* 5: sky (Y=0) */
                    {220,180,100}, /* 6: sand/detail (Y=64) */
                    {150, 80, 40}, /* 7: dark ground (Y=192) */
                    {120,180,220}, /* 8: light sky (Y=192) */
                    {200, 60, 40}, /* 9: red accent (Y=192) */
                    {100,100,100}, /* 10: road gray (Y=64) */
                    {220,200,160}, /* 11: light sand (Y=192) */
                    { 80,120, 40}, /* 12: dark green (Y=192) */
                    {180,140,100}, /* 13: tan (Y=192) */
                    { 80,130,180}, /* 14: blue detail (Y=64) */
                    {240,200, 60}, /* 15: yellow highlight (Y=336) */
                };
                /* Layer Y positions from dispatch table position data */
                static const int layer_y[16] = {
                    192, 64, 128, 192, 192, 0, 64, 192,
                    192, 192, 64, 192, 192, 192, 64, 336
                };
                /* Clear to sky gradient */
                for (int y = 0; y < 240; y++) {
                    int sky_r = 60 + (100-60)*y/240;
                    int sky_g = 120 + (170-120)*y/240;
                    int sky_b = 200 + (240-200)*y/240;
                    for (int x = 0; x < 320; x++)
                        vf->framebuf[y*320+x] = 0xFF000000 |
                            ((uint32_t)sky_r<<16)|((uint32_t)sky_g<<8)|sky_b;
                }
                /* Composite each layer from back (sky) to front (ground).
                 * Sort by Y position: Y=0 first, Y=336 last. */
                int order[16];
                for (int i = 0; i < 16; i++) order[i] = i;
                for (int i = 0; i < 15; i++)
                    for (int j = i+1; j < 16; j++)
                        if (layer_y[order[i]] > layer_y[order[j]]) {
                            int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
                        }
                /* Use DECOMPRESSED tile data at 0x10240000 as layer sources.
                 * 29 tiles total (512x256, 256x128, etc.) — assign groups of
                 * tiles to each render layer based on Y position. */
                static const struct { int tile_idx; int tw; int th; } layer_tiles[16] = {
                    { 0, 512,256}, /* 0: Y=192 ground — tile 0 (512x256) */
                    { 5, 512,128}, /* 1: Y=64  terrain — tile 5 (512x128) */
                    { 9, 512,128}, /* 2: Y=128 horizon — tile 9 (512x128) */
                    { 7, 256,256}, /* 3: Y=192 vegetation — tile 7 (256x256) */
                    { 8, 512, 64}, /* 4: Y=192 road — tile 8 (512x64) */
                    {22, 256,256}, /* 5: Y=0   sky — tile 22 (256x256) */
                    { 1, 256,128}, /* 6: Y=64  sand — tile 1 (256x128) */
                    { 3, 256,128}, /* 7: Y=192 dark ground — tile 3 (256x128) */
                    {24, 512,128}, /* 8: Y=192 light sky — tile 24 (512x128) */
                    { 6, 128, 64}, /* 9: Y=192 red accent — tile 6 (128x64) */
                    { 2, 512, 32}, /*10: Y=64  road gray — tile 2 (512x32) */
                    { 4, 512, 32}, /*11: Y=192 light sand — tile 4 (512x32) */
                    {21, 256, 64}, /*12: Y=192 dark green — tile 21 (256x64) */
                    {26, 256, 64}, /*13: Y=192 tan — tile 26 (256x64) */
                    {27, 128,128}, /*14: Y=64  blue detail — tile 27 (128x128) */
                    {10, 256, 32}, /*15: Y=336 yellow — tile 10 (256x32) */
                };
                /* Tile offset table (cumulative sizes) */
                static const uint32_t tile_offsets[] = {
                    0x000000,0x020000,0x028000,0x02C000,0x034000,
                    0x038000,0x048000,0x04A000,0x05A000,0x062000,
                    0x072000,0x074000,0x0F4000,0x0F6000,0x0FA000,
                    0x0FE000,0x0FE800,0x0FF800,0x100800,0x102800,
                    0x104800,0x105800,0x109800,0x119800,0x11B800,
                    0x12B800,0x12D800,0x131800,0x135800,
                };
                for (int li = 0; li < 16; li++) {
                    int idx = order[li];
                    int ly = layer_y[idx];
                    if (ly >= 240) continue;
                    int ti = layer_tiles[idx].tile_idx;
                    int tw = layer_tiles[idx].tw;
                    int th = layer_tiles[idx].th;
                    uint32_t dp = tile_off + tile_offsets[ti];
                    if (dp + (uint32_t)(tw*th) > VFLASH_RAM_SIZE) continue;
                    uint8_t cr = layer_colors[idx].r;
                    uint8_t cg = layer_colors[idx].g;
                    uint8_t cb = layer_colors[idx].b;
                    int lh = (ly == 0) ? 80 : 60;
                    int y_start = ly * 240 / 320;
                    if (y_start + lh > 240) lh = 240 - y_start;
                    for (int y = y_start; y < y_start + lh && y < 240; y++) {
                        for (int x = 0; x < 320; x++) {
                            int sy = (y - y_start) * th / lh;
                            int sx = x * tw / 320;
                            if (sy >= th) sy = th - 1;
                            if (sx >= tw) sx = tw - 1;
                            uint8_t v = vf->ram[dp + sy * tw + sx];
                            if (v == 0) continue;
                            /* Alpha-blend: pixel = old*(1-a) + color*a */
                            uint32_t old = vf->framebuf[y*320+x];
                            uint8_t or2=(old>>16)&0xFF, og=(old>>8)&0xFF, ob=old&0xFF;
                            int a = v;
                            uint8_t nr = (uint8_t)((or2*(255-a) + cr*a) / 255);
                            uint8_t ng = (uint8_t)((og*(255-a) + cg*a) / 255);
                            uint8_t nb = (uint8_t)((ob*(255-a) + cb*a) / 255);
                            vf->framebuf[y*320+x] =
                                0xFF000000|((uint32_t)nr<<16)|((uint32_t)ng<<8)|nb;
                        }
                    }
                }
                /* Overlay sec[1] tile 11 (1024x512) as detail layer */
                {
                    uint32_t t11 = tile_off + 0x074000;
                    for (int y = 60; y < 200; y++) {
                        int sy = (y - 60) * 512 / 140;
                        for (int x = 0; x < 320; x++) {
                            int sx = x * 1024 / 320;
                            if (t11 + sy*1024+sx >= VFLASH_RAM_SIZE) break;
                            uint8_t v = vf->ram[t11 + sy*1024 + sx];
                            if (v < 30) continue; /* skip low intensity */
                            uint32_t old = vf->framebuf[y*320+x];
                            uint8_t or2=(old>>16)&0xFF, og=(old>>8)&0xFF, ob=old&0xFF;
                            int a = v / 3; /* subtle overlay */
                            uint8_t nr = (uint8_t)((or2*(255-a) + 200*a)/255);
                            uint8_t ng = (uint8_t)((og*(255-a) + 160*a)/255);
                            uint8_t nb = (uint8_t)((ob*(255-a) + 100*a)/255);
                            vf->framebuf[y*320+x] =
                                0xFF000000|((uint32_t)nr<<16)|((uint32_t)ng<<8)|nb;
                        }
                    }
                }
                /* No HLE sprites — let native engine render when RTOS works */
                ent_count = nz;
                static int tile_log = 0;
                if (!tile_log) {
                    tile_log = 1;
                    printf("[HLE-RENDER] Scene + interactive car (arrows to move)\n");
                }
            } else {
                ent_count = 0;
            }
            vf->vid.fb_dirty = 1;
            static int ptx_log = 0;
            if (!ptx_log) {
                printf("[GAME-DISPLAY] PTX + %d entity tiles on screen\n", ent_count);
                /* Save screenshot */
                FILE *sf = fopen("/tmp/vflash_game.bmp", "wb");
                if (sf) {
                    int w=320,h=240,rs=w*3,pad=(4-rs%4)%4,isz=(rs+pad)*h;
                    uint8_t hd[54]={0}; hd[0]='B';hd[1]='M';
                    *(uint32_t*)(hd+2)=54+isz; *(uint32_t*)(hd+10)=54;
                    *(uint32_t*)(hd+14)=40; *(int32_t*)(hd+18)=w;
                    *(int32_t*)(hd+22)=-h; *(uint16_t*)(hd+26)=1;
                    *(uint16_t*)(hd+28)=24; *(uint32_t*)(hd+34)=isz;
                    fwrite(hd,1,54,sf);
                    for(int y2=0;y2<h;y2++){for(int x2=0;x2<w;x2++){
                        uint32_t px=vf->framebuf[y2*w+x2];
                        uint8_t rgb[3]={px&0xFF,(px>>8)&0xFF,(px>>16)&0xFF};
                        fwrite(rgb,1,3,sf);}
                        uint8_t z[4]={0};if(pad)fwrite(z,1,pad,sf);}
                    fclose(sf);
                }
                ptx_log = 1;
            }
        }
    }

    /* Fire VBlank callback per-frame: the render pipeline sets a callback
     * via DC register 0xB80007C0. On real HW, the display controller fires
     * this at VBlank. We invoke it here to advance the render pipeline. */
    if (vf->boot_phase >= 900 && vf->dc_vblank_cb &&
        vf->dc_vblank_cb >= 0x10000000 && vf->dc_vblank_cb < 0x11000000) {
        /* Save CPU state */
        uint32_t save_pc = vf->cpu.r[15];
        uint32_t save_lr = vf->cpu.r[14];
        uint32_t save_cpsr = vf->cpu.cpsr;
        /* Call VBlank callback with a small execution budget */
        vf->cpu.r[15] = vf->dc_vblank_cb;
        vf->cpu.r[14] = 0x10FFF000; /* return to idle */
        vf->cpu.cpsr = 0x00000013;  /* SVC, IRQ enabled */
        int vbl_budget = 50000;
        while (vbl_budget > 0 && vf->cpu.r[15] != 0x10FFF000 &&
               vf->cpu.r[15] != 0x10FFF00C) {
            uint64_t cb = vf->cpu.cycles;
            arm9_step(&vf->cpu);
            int cost = (int)(vf->cpu.cycles - cb);
            vbl_budget -= cost > 0 ? cost : 1;
            /* Escape check */
            uint32_t vpc = vf->cpu.r[15];
            if (vpc >= 0xB8000000u || (vpc < 0x10000000u && vpc > 0x1000u)) {
                vf->cpu.r[15] = 0x10FFF000;
                break;
            }
        }
        /* Restore CPU state (callback may have changed registers) */
        vf->cpu.r[15] = save_pc;
        vf->cpu.r[14] = save_lr;
        vf->cpu.cpsr = save_cpsr;
        static int vbl_log = 0;
        if (vbl_log < 5) {
            printf("[VBLANK] Fired callback 0x%08X (budget left: %d)\n",
                   vf->dc_vblank_cb, vbl_budget);
            vbl_log++;
        }
    }

    /* PL111 LCD framebuffer blit: only when game actually writes to LCD
     * (not during asset browser mode where MJP/PTX write directly to framebuf).
     * Skip if video is playing or if the LCD framebuffer hasn't been written by game code. */
    /* Only LCD blit when render pipeline has set a valid framebuffer via DC.
     * dc_vblank_cb != 0 means render code configured the display controller. */
    if ((vf->lcd.control & 1) && vf->lcd.upbase >= VFLASH_RAM_BASE &&
        vf->lcd.upbase < VFLASH_RAM_BASE + VFLASH_RAM_SIZE && !vf->vid.fb_dirty &&
        !vf->mjp_player.playing && vf->dc_vblank_cb &&
        (!vf->ptx_loaded || vf->boot_phase >= 900)) {
        uint32_t fb_off = vf->lcd.upbase - VFLASH_RAM_BASE;
        uint32_t bpp_code = (vf->lcd.control >> 1) & 7;
        /* BPP: 1=2bpp, 2=4bpp, 3=8bpp, 4=16bpp, 5=24bpp, 6=16bpp565 */
        /* Probe: check if ANY pixels are non-zero in the framebuffer */
        {
            static int fb_probe = 0;
            if (fb_probe < 5) {
                int nz = 0;
                for (uint32_t pi = 0; pi < 320*240*4 && fb_off+pi < VFLASH_RAM_SIZE; pi += 4) {
                    if (*(uint32_t*)(vf->ram+fb_off+pi)) { nz++; if (nz > 100) break; }
                }
                if (nz > 0) {
                    printf("[LCD-PROBE] fb@0x%08X bpp=%d: %d non-zero dwords!\n",
                           vf->lcd.upbase, 1<<bpp_code, nz);
                    fb_probe = 5; /* stop probing */
                } else if (vf->frame_count % 50 == 0) {
                    printf("[LCD-PROBE] fb@0x%08X bpp=%d: empty (frame %d)\n",
                           vf->lcd.upbase, 1<<bpp_code, vf->frame_count);
                }
                fb_probe++;
            }
        }
        if (bpp_code == 5) {
            /* 24-bit RGB framebuffer (3 bytes per pixel, packed) */
            const uint8_t *src = vf->ram + fb_off;
            int nz = 0;
            for (int y = 0; y < VFLASH_SCREEN_H; y++) {
                for (int x = 0; x < VFLASH_SCREEN_W; x++) {
                    int off2 = (y * VFLASH_SCREEN_W + x) * 3;
                    uint8_t b = src[off2], g = src[off2+1], r = src[off2+2];
                    if (r || g || b) nz++;
                    vf->framebuf[y * VFLASH_SCREEN_W + x] =
                        0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                }
            }
            if (nz > 0) {
                vf->vid.fb_dirty = 1;
                static int lcd24_log = 0;
                if (lcd24_log++ < 3)
                    printf("[LCD] 24bpp blit: %d non-zero pixels\n", nz);
            }
        } else if (bpp_code == 4 || bpp_code == 6) {
            /* 16-bit RGB565 framebuffer */
            const uint16_t *src = (const uint16_t*)(vf->ram + fb_off);
            for (int y = 0; y < VFLASH_SCREEN_H; y++) {
                for (int x = 0; x < VFLASH_SCREEN_W; x++) {
                    uint16_t px = src[y * VFLASH_SCREEN_W + x];
                    uint8_t r = ((px >> 11) & 0x1F) << 3;
                    uint8_t g = ((px >> 5) & 0x3F) << 2;
                    uint8_t b = (px & 0x1F) << 3;
                    vf->framebuf[y * VFLASH_SCREEN_W + x] =
                        0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
                }
            }
            vf->vid.fb_dirty = 1;
            static int lcd_blit_log = 0;
            if (lcd_blit_log < 3) {
                /* Save screenshot every 100 blits */
                if (lcd_blit_log == 2) {
                    FILE *sf = fopen("/tmp/vflash_screen.bmp","wb");
                    if (sf) {
                        int w=320,h=240,rs=w*3,pad=(4-rs%4)%4,isz=(rs+pad)*h;
                        uint8_t hd[54]={0}; hd[0]='B';hd[1]='M';
                        *(uint32_t*)(hd+2)=54+isz; *(uint32_t*)(hd+10)=54;
                        *(uint32_t*)(hd+14)=40; *(int32_t*)(hd+18)=w;
                        *(int32_t*)(hd+22)=-h; *(uint16_t*)(hd+26)=1;
                        *(uint16_t*)(hd+28)=24; *(uint32_t*)(hd+34)=isz;
                        fwrite(hd,1,54,sf);
                        for(int y2=0;y2<h;y2++){for(int x2=0;x2<w;x2++){
                            uint32_t px=vf->framebuf[y2*w+x2];
                            uint8_t rgb[3]={px&0xFF,(px>>8)&0xFF,(px>>16)&0xFF};
                            fwrite(rgb,1,3,sf);}
                            uint8_t z[4]={0};if(pad)fwrite(z,1,pad,sf);}
                        fclose(sf);
                    }
                }
                lcd_blit_log++;
            }
        } else if (bpp_code == 5) {
            /* 24-bit or 32-bit ARGB */
            const uint32_t *src = (const uint32_t*)(vf->ram + fb_off);
            for (int i = 0; i < VFLASH_SCREEN_W * VFLASH_SCREEN_H; i++)
                vf->framebuf[i] = src[i] | 0xFF000000;
            vf->vid.fb_dirty = 1;
        }
    }

    /* C-side event pump: call game dispatcher directly each frame.
     * Bypasses main CPU loop which gets stuck in IRQ mode. */
    if (vf->has_rom && vf->frame_count > 30 && vf->cd && vf->cd->is_open) {
        static int pump_active = 0;
        if (!pump_active && *(uint32_t*)(vf->ram + 0xB909C0) != 0) {
            pump_active = 1;
            printf("[EVENT-PUMP] Dispatcher pump active\n");
        }
        if (pump_active) {
            /* Save CPU state */
            uint32_t save_pc = vf->cpu.r[15];
            uint32_t save_sp = vf->cpu.r[13];
            uint32_t save_lr = vf->cpu.r[14];
            uint32_t save_cpsr = vf->cpu.cpsr;
            uint32_t save_r0 = vf->cpu.r[0];
            uint32_t save_r1 = vf->cpu.r[1];

            /* Update event data each frame */
            {
                uint32_t evt = 0xF01000;
                *(uint32_t*)(vf->ram + evt + 0x0C) = (uint32_t)vf->frame_count; /* frame counter */
                *(uint32_t*)(vf->ram + evt + 0x18) = vf->input; /* button state */
            }
            /* Call dispatcher: cycle through event types 0-7 */
            vf->cpu.r[0] = (uint32_t)(vf->frame_count % 8);
            vf->cpu.r[1] = *(uint32_t*)(vf->ram + 0xB909C0);
            vf->cpu.r[14] = 0x10FFF000; /* return to idle */
            vf->cpu.r[15] = 0x10C1377C; /* dispatcher entry */
            vf->cpu.r[13] = 0x10FFD000; /* separate stack */
            vf->cpu.cpsr = 0x000000D3;  /* SVC, IRQ off */

            /* Run up to 50K instructions */
            arm9_run(&vf->cpu, 50000);

            /* Check if dispatcher returned to idle or got stuck */
            uint32_t end_pc = vf->cpu.r[15];
            if (end_pc == 0x10FFF000 || end_pc == 0x10FFF00C ||
                (end_pc >= 0x10C13780 && end_pc <= 0x10C13800)) {
                /* Dispatcher returned normally — keep any side effects */
                static int pump_log = 0;
                if (pump_log < 5) {
                    printf("[EVENT-PUMP] Dispatch returned at 0x%08X\n", end_pc);
                    pump_log++;
                }
            } else {
                /* Got stuck — restore state */
                vf->cpu.r[15] = save_pc;
                vf->cpu.r[13] = save_sp;
                vf->cpu.r[14] = save_lr;
                vf->cpu.cpsr = save_cpsr;
                vf->cpu.r[0] = save_r0;
                vf->cpu.r[1] = save_r1;
            }
            /* Always re-enable timer (game code might have disabled it) */
            vf->timer.timer[0].load = 37500;
            vf->timer.timer[0].count = 37500;
            vf->timer.timer[0].ctrl = 0xE2;
            vf->timer.irq.enable |= 0x01;
            vf->timer.timer[0].irq_pending = 0;
            vf->timer.irq.status = 0;
            /* Restore CPU to idle with IRQ enabled */
            vf->cpu.r[15] = 0x10FFF000;
            vf->cpu.r[13] = 0x10FFE000;
            vf->cpu.cpsr  = 0x00000053; /* SVC, IRQ enabled, C flag */
        }
    }

    /* HLE input: write button state to game input buffers.
     * The RTOS input driver normally reads GPIO at 0x80004000 and dispatches
     * button events to callbacks that write to 0x10BBE8xx buffers.
     * Since RTOS dispatch is stubbed, we write directly.
     * Each buffer is 12 bytes: [+0]=button_state, [+4]=pressed, [+8]=timestamp.
     * 5 buttons: DPAD(0), Red(1), Yellow(2), Green(3), Blue(4) */
    if (vf->has_rom && vf->boot_phase >= 900) {
        static const uint32_t btn_bufs[] = {
            0xBBE8BC, 0xBBE8E8, 0xBBE914, 0xBBE940, 0xBBE96C
        };
        /* Map emulator buttons to game button buffers.
         * Button 0 (DPAD): encode direction in R0, pressed in R1 */
        uint32_t pressed = vf->input & ~vf->input_prev;
        uint32_t released = vf->input_prev & ~vf->input;
        /* DPAD → buffer 0: R0=direction (1=up,2=down,3=left,4=right), R1=state */
        {
            uint32_t dir = 0;
            if (vf->input & VFLASH_BTN_UP) dir = 1;
            else if (vf->input & VFLASH_BTN_DOWN) dir = 2;
            else if (vf->input & VFLASH_BTN_LEFT) dir = 3;
            else if (vf->input & VFLASH_BTN_RIGHT) dir = 4;
            *(uint32_t*)(vf->ram + btn_bufs[0]) = dir;
            *(uint32_t*)(vf->ram + btn_bufs[0] + 4) = (dir != 0) ? 1 : 0;
            *(uint32_t*)(vf->ram + btn_bufs[0] + 8) = vf->frame_count;
        }
        /* Color buttons → buffers 1-4: R0=1(pressed)/0(released), R1=event */
        for (int bi = 0; bi < 4; bi++) {
            uint32_t btn_mask = VFLASH_BTN_RED << bi;
            uint32_t is_held = (vf->input & btn_mask) ? 1 : 0;
            uint32_t is_pressed = (pressed & btn_mask) ? 1 : 0;
            *(uint32_t*)(vf->ram + btn_bufs[1+bi]) = is_held;
            *(uint32_t*)(vf->ram + btn_bufs[1+bi] + 4) = is_pressed;
            *(uint32_t*)(vf->ram + btn_bufs[1+bi] + 8) = vf->frame_count;
        }
        /* Also write to input register that game code may poll directly */
        *(uint32_t*)(vf->ram + 0xBBE880) = vf->input;
        if (pressed) {
            static int inp_log = 0;
            if (inp_log < 20) {
                printf("[HLE-INPUT] buttons=0x%03X pressed=0x%03X\n",
                       vf->input, pressed);
                inp_log++;
            }
        }
    }

    vf->input_prev = vf->input;
    vf->frame_count++;
}

void     vflash_set_input(VFlash *vf, uint32_t buttons) { vf->input = buttons; }
void    *vflash_get_cpu(VFlash *vf) { return &vf->cpu; }
void    *vflash_get_timer(VFlash *vf) { return &vf->timer; }
uint8_t *vflash_get_ram(VFlash *vf) { return vf->ram; }
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
    if (ztimer_fiq_pending(&vf->timer)) {
        memcpy(vf->fiq_saved_r8_12, vf->cpu.r + 8, 5 * sizeof(uint32_t));
        arm9_fiq(&vf->cpu);
    } else if (ztimer_irq_pending(&vf->timer))
        arm9_irq(&vf->cpu);

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

