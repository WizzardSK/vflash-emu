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
    uint32_t  dma_param_a, dma_param_b, dma_param_c;
    uint32_t  misc_regs[64];   /* Misc system control at 0x900A0000 (read/write) */
    uint32_t  rtc_regs[16];    /* RTC scratch at 0x900900F0-0x900900FF */
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
    a->sense_key = 0x06; /* UNIT ATTENTION (power on) */
    a->asc = 0x29;       /* power on reset */
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

    /* Re-insert byte-stuffing in entropy data */
    uint8_t *fixed = malloc(total + total / 4 + 16);
    if (!fixed) { free(swapped); return 0; }
    memcpy(fixed, swapped, sos_data);
    uint32_t fi = sos_data;
    for (uint32_t ei = sos_data; ei < total; ei++) {
        fixed[fi++] = swapped[ei];
        if (swapped[ei] == 0xFF && ei + 1 < total) {
            uint8_t nxt = swapped[ei + 1];
            if (nxt == 0x00 || (nxt >= 0xD0 && nxt <= 0xD7) || nxt == 0xD9)
                { fixed[fi++] = nxt; ei++; }
            else
                { fixed[fi++] = 0x00; }
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

    /* Fast path: RAM (most common — ~90% of all accesses) */
    if (__builtin_expect(addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE, 1))
        return *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE));

    /* Physical address 0:
     * With boot ROM: 512KB read-only NOR flash (always present)
     * Without ROM (HLE): addr 0 = RAM (writable, for vector table + stubs) */
    if (addr < 0x00200000u) {
        if (vf->has_rom && addr < vf->rom_size)
            return *(uint32_t*)(vf->rom + addr);
        /* HLE mode: addr 0 = RAM (same buffer as 0x10000000) */
        if (!vf->has_rom && addr < VFLASH_RAM_SIZE)
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
                default: return 0;
            }
        }

        /* PL190 VIC at 0xDC000000 (off = 0x5C000000) */
        if (off >= 0x5C000000u && off < 0x5C001000u) {
            uint32_t vreg = off - 0x5C000000u;
            switch (vreg) {
                case 0x000: return vf->timer.irq.status & vf->timer.irq.enable; /* IRQStatus */
                case 0x004: return vf->timer.irq.status & vf->timer.irq.fiq_sel; /* FIQStatus */
                case 0x008: return vf->timer.irq.status;  /* RawIntr */
                case 0x010: return vf->timer.irq.enable;   /* IntEnable */
                default: return 0;
            }
        }

        /* NOR Flash controller at 0xB8000000 (2MB). */
        if (off >= 0x38000000u && off < 0x38200000u && vf->has_rom) {
            uint32_t foff = off - 0x38000000u;
            if (foff < 0x800) {
                /* Controller status registers */
                switch (foff) {
                    case 0x00: return 0;
                    case 0x08: return 0;      /* Operation complete */
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
                case 0x0C: return vf->misc_regs[0x0C >> 2]; /* Boot status: 0=cold, bit1=warm */
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
                case 0x08: return 0x2000;     /* PMU status */
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

        /* PL011 UART at 0x90020000 (off = 0x10020000) */
        if (off >= 0x10020000u && off < 0x10021000u) {
            uint32_t ureg = off - 0x10020000u;
            switch (ureg) {
                case 0x00: return 0;         /* UARTDR: no data */
                case 0x18: return 0x90;      /* UARTFR: TX empty, RX empty */
                case 0x24: return 0;         /* UARTIBRD */
                case 0x28: return 0;         /* UARTFBRD */
                case 0x2C: return 0;         /* UARTLCR_H */
                case 0x30: return 0;         /* UARTCR */
                case 0xFE0: return 0x11;     /* PL011 PeriphID0 */
                case 0xFE4: return 0x10;     /* PL011 PeriphID1 */
                default:    return 0;
            }
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
            /* Scratch registers at 0xF0-0xFF used by ROM SDRAM calibration */
            if (rreg >= 0xF0 && rreg < 0x100)
                return vf->rtc_regs[(rreg - 0xF0) >> 2];
            switch (rreg) {
                case 0x00:  return 0x67000000; /* RTCDR: time */
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
        if (vf->has_rom && addr + 1 < vf->rom_size)
            return *(uint16_t*)(vf->rom + addr);
        if (!vf->has_rom && addr + 1 < VFLASH_RAM_SIZE)
            return *(uint16_t*)(vf->ram + addr);
        return 0;
    }
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;
        if (vf->has_rom && off + 1 < vf->rom_size)
            return *(uint16_t*)(vf->rom + off);
        if (!vf->has_rom && off + 1 < VFLASH_RAM_SIZE)
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
    if (addr < 0x00200000u) {
        if (vf->has_rom && addr < vf->rom_size)
            return vf->rom[addr];
        if (!vf->has_rom && addr < VFLASH_RAM_SIZE)
            return vf->ram[addr];
        return 0;
    }
    if (addr >= 0xFFFF0000u) {
        uint32_t off = addr - 0xFFFF0000u;
        if (vf->has_rom && off < vf->rom_size)
            return vf->rom[off];
        if (!vf->has_rom && off < VFLASH_RAM_SIZE)
            return vf->ram[off];
        return 0;
    }
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

    /* Fast path: RAM write (most common) */
    if (__builtin_expect(addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE, 1)) {
        *(uint32_t*)(vf->ram + (addr - VFLASH_RAM_BASE)) = val;
        return;
    }

    /* Physical addr 0-2MB:
     * With ROM: read-only, writes ignored.
     * HLE mode (no ROM): writable RAM for vector table and ROM stubs. */
    if (addr < 0x00200000u) {
        if (vf->has_rom) return;  /* ROM is read-only */
        if (addr < VFLASH_RAM_SIZE) {
            *(uint32_t*)(vf->ram + addr) = val;
            return;
        }
        return;
    }

    if (addr >= VFLASH_RAM_BASE && addr < VFLASH_RAM_BASE + VFLASH_RAM_SIZE) {
        uint32_t roff = addr - VFLASH_RAM_BASE;
        /* (mem trace removed) */
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

        /* Primary IRQ + timers */
        if (off < 0x200) { ztimer_write(&vf->timer, off, val); return; }

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
            if (foff >= 0x800 && foff < 0x800 + sizeof(vf->flash_buf)) {
                uint32_t boff = foff - 0x800;
                /* Buffer the write */
                *(uint32_t*)(vf->flash_buf + boff) = val;
                vf->flash_buf_dirty = 1;

                /* Write to reg 0x800 = remap trigger */
                if (foff == 0x800 && val != 0) {
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
                    uint32_t tbl_off = val + 0x2D0;  /* SDRAM config table in RAM */
                    if (tbl_off + 8 < VFLASH_RAM_SIZE) {
                        printf("[FLASH] Table before: [0x%X]=0x%08X [0x%X]=0x%08X [0x%X]=0x%08X\n",
                               tbl_off, *(uint32_t*)(vf->ram + tbl_off),
                               tbl_off+4, *(uint32_t*)(vf->ram + tbl_off+4),
                               tbl_off+8, *(uint32_t*)(vf->ram + tbl_off+8));
                        /* Put terminator right at first entry */
                        *(uint32_t*)(vf->ram + tbl_off) = 0x000000FF;
                        printf("[FLASH] Patched SDRAM table: 0xFF at RAM[0x%X]\n", tbl_off);
                        printf("[FLASH] Verify: RAM[0x%X]=0x%08X\n", tbl_off,
                               *(uint32_t*)(vf->ram + tbl_off));
                    }

                    /* Pre-load BOOT.BIN into RAM so the ROM can jump to it
                     * after calibration. On real HW, the ROM uses the ATAPI
                     * controller to read BOOT.BIN from disc. We bypass ATAPI
                     * by loading it now, so when ROM jumps to 0x10C00000+
                     * the code is already in place. */
                    if (vf->cd && vf->cd->is_open) {
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
                        /* Pre-load everything: kernel + modules + ROM code */
                        {
                            memcpy(vf->ram + 0x9FFD4, vf->rom + 0xC02C, 0xA1FE4);
                            memcpy(vf->ram + 0xAC000, vf->rom + 0xAE010, 0xF95B0);
                            for (uint32_t ci = 0x80000; ci < vf->rom_size; ci += 4) {
                                uint32_t rv = *(uint32_t*)(vf->rom + ci);
                                if (rv != 0) *(uint32_t*)(vf->ram + ci) = rv;
                            }
                            /* Set sched_state=3 BEFORE ROM init runs.
                             * µMORE task_start (0x85E88) checks [0x103585E0]==3
                             * and refuses to register tasks if state != 3.
                             * On real HW this is set by earlier init stages. */
                            *(uint32_t*)(vf->ram + 0x3585E0) = 3;
                            printf("[ROM-PRELOAD] Kernel+modules+code+sched_state=3\n");
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

        /* PL011 UART write at 0x90020000 */
        if (off >= 0x10020000u && off < 0x10021000u) {
            uint32_t ureg = off - 0x10020000u;
            if (ureg == 0x00) {
                /* UARTDR: TX data — print to stderr */
                char c = (char)(val & 0xFF);
                if (c >= 0x20 || c == '\n')
                    fprintf(stderr, "%c", c);
            }
            return;
        }

        /* Watchdog write at 0x90060000 — silently accept */
        if (off >= 0x10060000u && off < 0x10061000u) return;

        /* RTC write at 0x90090000 — store scratch registers */
        if (off >= 0x10090000u && off < 0x10091000u) {
            uint32_t rreg = off - 0x10090000u;
            if (rreg >= 0xF0 && rreg < 0x100)
                vf->rtc_regs[(rreg - 0xF0) >> 2] = val;
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
            if (sreg == 0x08 && val == 1 && vf->has_rom) {
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

                if (vf->atapi.reboot_count > 3) {
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

                    /* After warm boot, redirect to BOOT.BIN entry so it runs
                     * its init functions again. This second run should register
                     * tasks since µMORE kernel is now in RAM.
                     * Patch callback to idle stub (prevent infinite reboot). */
                    *(uint32_t*)(vf->ram + 0xC00020) = 0x10FFF000;

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

                /* Set warm boot flag and restart from ROM */
                vf->misc_regs[0x0C >> 2] |= 0x02;
                arm9_reset(&vf->cpu);
                vf->cpu.r[15] = 0x00000000;
                vf->cpu.r13_irq = 0x10800000;
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
        if (vf->has_rom) return;
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
        if (vf->has_rom) return;
        if (addr < VFLASH_RAM_SIZE) vf->ram[addr] = val;
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

    /* Re-enable timer at frame start (may have been disabled for IRQ delivery) */
    if (vf->has_rom && vf->timer.timer[0].load > 0 && vf->timer.timer[0].ctrl == 0) {
        vf->timer.timer[0].ctrl = 0x77;
        vf->timer.timer[0].count = vf->timer.timer[0].load;
    }

    int done = 0;
    while (done < TOTAL) {
        int slice = (TOTAL - done < SLICE) ? (TOTAL - done) : SLICE;

        uint64_t cyc_before = vf->cpu.cycles;
        arm9_run(&vf->cpu, slice);
        uint32_t actual = (uint32_t)(vf->cpu.cycles - cyc_before);

        /* Trace boot flow: log when PC enters key regions */
        {
            static uint32_t last_region = 0;
        }

        ztimer_tick(&vf->timer, actual);
        done += (int)actual;

        /* Detect scheduler loop or idle stub and set up IRQ handling.
         * Phase 1: ROM init → scheduler loop (0x7FFC0-0x80060) → redirect to BOOT.BIN
         * Phase 2: BOOT.BIN init → idle stub (0xFFF000) → install IRQ, enable IRQs */
        if (vf->has_rom && vf->frame_count > 2) {
            uint32_t pc = vf->cpu.r[15];
            static int phase = 0;

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

                /* Fix page table: map BOOT.BIN at VA 0x10C/0x10D (identity) */
                {
                    uint32_t ttb_off = vf->cpu.cp15.ttb - 0x10000000;
                    for (uint32_t mb = 0x10C; mb <= 0x10D; mb++)
                        *(uint32_t*)(vf->ram + ttb_off + mb * 4) = (mb << 20) | 0xC02;
                    tlb_flush();
                    printf("[SCHED] Mapped VA 0x10C/0x10D → identity\n");
                }
                /* Create LCD handle for game init (prevents NULL crash) */
                {
                    uint32_t h = 0xBF0000;
                    memset(vf->ram + h, 0, 0x100);
                    *(uint32_t*)(vf->ram + h + 0x5C) = 0xC0000000; /* PL111 base */
                    *(uint32_t*)(vf->ram + 0xBC0A40) = 0x10000000 + h;
                    printf("[SCHED] LCD handle at 0x10%06X\n", h);
                }
                /* Jump to game init with IRQ disabled */
                vf->cpu.cpsr = 0x00000093;
                vf->cpu.r[15] = 0x10C16CB8;
                vf->cpu.r[13] = 0x10FFE000;
                vf->cpu.r[14] = 0x10FFF000; /* return → idle */
                /* Clear pending IRQ so it doesn't fire immediately */
                vf->timer.timer[0].irq_pending = 0;
                vf->timer.irq.status = 0;
                printf("[SCHED] Phase 2: → GAME INIT at 0x10C16CB8 (IRQ disabled)\n");
            }
        }

        /* Deliver IRQ if pending and CPSR allows */
        if (vf->frame_count > 1 && ztimer_irq_pending(&vf->timer)) {
            if (vf->cpu.r13_irq < 0x10000000u || vf->cpu.r13_irq > 0x10FFFFFFu)
                vf->cpu.r13_irq = 0x10800000;
            /* Log first IRQ delivery */
            static int irq_logged = 0;
            if (!irq_logged) {
                uint32_t vec_addr = vf->cpu.cp15.hivec ? 0xFFFF0018 : 0x18;
                uint32_t pa = mmu_translate(vf, vec_addr);
                uint32_t vec_insn = mem_read32(vf, vec_addr);
                printf("[IRQ] First IRQ! PC was 0x%08X, vector VA=0x%08X PA=0x%08X insn=0x%08X\n",
                       vf->cpu.r[15], vec_addr, pa, vec_insn);
                printf("[IRQ] SP_irq=0x%08X CPSR=0x%08X MMU=%d\n",
                       vf->cpu.r13_irq, vf->cpu.cpsr, vf->cpu.cp15.mmu_enabled);
                irq_logged = 1;
            }
            arm9_irq(&vf->cpu);
        }
    }

    ztimer_raise_irq(&vf->timer, IRQ_TIMER0);   /* vsync */

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
                                uint8_t b = ((p >> 10) & 0x1F) << 3;
                                uint8_t g = ((p >> 5)  & 0x1F) << 3;
                                uint8_t r = ( p        & 0x1F) << 3;
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

    /* MJP video player — load on first frame, then play frame-by-frame */
    if (vf->frame_count == 1 && vf->has_rom && vf->cd && vf->cd->is_open
        && !vf->mjp_player.playing) {
        /* Find and load MJP file from disc */
        CDEntry mjp_entry;
        static const char *mjp_names[] = {
            "EN.MJP", "MAIN01.MJP", "MAIN.MJP", "CUTSCENE01.MJP",
            "INTRO.MJP", "MOVIE.MJP", "OPENING.MJP",
            "101KW_MOVIE.MJP", "106KW_MOVIE.MJP", NULL
        };
        int mjp_found = 0;
        for (int mi = 0; mjp_names[mi] && !mjp_found; mi++)
            mjp_found = cdrom_find_file_any(vf->cd, mjp_names[mi], &mjp_entry);
        if (!mjp_found) {
            CDEntry entries[64];
            uint8_t pvd[2048];
            if (cdrom_read_sector(vf->cd, 16, pvd)) {
                uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
                uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
                int n = cdrom_list_dir(vf->cd, root_lba, root_size, entries, 64);
                for (int ei = 0; ei < n && !mjp_found; ei++) {
                    char *dot = strrchr(entries[ei].name, '.');
                    if (dot && (strcmp(dot, ".MJP") == 0 || strcmp(dot, ".mjp") == 0))
                        { mjp_entry = entries[ei]; mjp_found = 1; }
                }
            }
        }
        if (mjp_found && mjp_entry.size > 100) {
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
    }

    /* Decode next MJP frame if player is active */
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
                        /* Scale decoded frame to fill 320×240 display */
                        uint32_t *src = mjp_get_framebuf(vf->video);
                        int sw = vf->video->width;
                        int src_w = vf->mjp_player.vid_w ? vf->mjp_player.vid_w : sw;
                        int src_h = vf->mjp_player.vid_h ? vf->mjp_player.vid_h : vf->video->height;
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
            /* Loop video if we reached the end */
            if (co + 12 >= dsz)
                vf->mjp_player.chunk_off = 0x40;
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

    /* PL111 LCD framebuffer blit: if LCD is enabled and has a valid
     * framebuffer address, copy it to our display framebuffer. */
    if ((vf->lcd.control & 1) && vf->lcd.upbase >= VFLASH_RAM_BASE &&
        vf->lcd.upbase < VFLASH_RAM_BASE + VFLASH_RAM_SIZE && !vf->vid.fb_dirty) {
        uint32_t fb_off = vf->lcd.upbase - VFLASH_RAM_BASE;
        uint32_t bpp_code = (vf->lcd.control >> 1) & 7;
        /* BPP: 1=2bpp, 2=4bpp, 3=8bpp, 4=16bpp, 5=24bpp, 6=16bpp565 */
        if (bpp_code == 4 || bpp_code == 6) {
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
        } else if (bpp_code == 5) {
            /* 24-bit or 32-bit ARGB */
            const uint32_t *src = (const uint32_t*)(vf->ram + fb_off);
            for (int i = 0; i < VFLASH_SCREEN_W * VFLASH_SCREEN_H; i++)
                vf->framebuf[i] = src[i] | 0xFF000000;
            vf->vid.fb_dirty = 1;
        }
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

