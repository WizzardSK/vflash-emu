/* vff_extract.c — Extract VFF sections from V.Flash disc image
 * Usage: vff_extract <disc.bin> [filename.VFF]
 * Extracts sec[0], sec[1], sec[2] as separate files for Ghidra analysis.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "cdrom.h"

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <disc.bin> [VFF_name]\n", argv[0]);
        return 1;
    }
    const char *vff_name = argc > 2 ? argv[2] : "MAIN.VFF";

    CDROM *cd = cdrom_create();
    if (!cdrom_open(cd, argv[1])) {
        fprintf(stderr, "Failed to open disc: %s\n", argv[1]);
        return 1;
    }

    CDEntry ve;
    if (!cdrom_find_file_any(cd, vff_name, &ve)) {
        fprintf(stderr, "VFF file '%s' not found on disc\n", vff_name);
        cdrom_destroy(cd);
        return 1;
    }
    printf("Found %s: LBA=%u size=%u bytes\n", ve.name, ve.lba, ve.size);

    /* Read VFF header (0x400 bytes) */
    uint8_t vhdr[0x400];
    cdrom_read_file(cd, &ve, vhdr, 0, 0x400);

    if (vhdr[0] != 'v' || vhdr[1] != 'f') {
        fprintf(stderr, "Not a VFF file (magic: %02X %02X)\n", vhdr[0], vhdr[1]);
        cdrom_destroy(cd);
        return 1;
    }

    /* Dump full header */
    printf("VFF Header:\n");
    for (int i = 0; i < 0x60; i += 4) {
        uint32_t w = *(uint32_t*)(vhdr + i);
        printf("  +%02X: %08X", i, w);
        if (i == 0x10) printf("  ← entry");
        if (i == 0x14) printf("  ← param");
        if (i == 0x18) printf("  ← callback1");
        if (i == 0x1C) printf("  ← callback2");
        if (i == 0x20) printf("  ← init_cb");
        if (i == 0x2C) printf("  ← nsec");
        if (i >= 0x30 && i < 0x60) {
            int si = (i - 0x30) / 0x10;
            int fi = (i - 0x30) % 0x10;
            const char *fn[] = {"load_addr","size","unk1","unk2"};
            printf("  ← sec[%d].%s", si, fn[fi/4]);
        }
        printf("\n");
    }

    uint32_t nsec = *(uint32_t*)(vhdr + 0x2C);
    uint32_t entry = *(uint32_t*)(vhdr + 0x10);
    printf("\nEntry=0x%08X nsec=%u\n\n", entry, nsec);

    /* Extract each section */
    uint32_t foff = 0x400;
    for (uint32_t si = 0; si < nsec && si < 8; si++) {
        uint32_t dst = *(uint32_t*)(vhdr + 0x30 + si * 0x10);
        uint32_t ssz = *(uint32_t*)(vhdr + 0x34 + si * 0x10);
        printf("sec[%u]: load_addr=0x%08X size=%u (0x%X) file_offset=0x%X\n",
               si, dst, ssz, ssz, foff);

        if (ssz == 0 || ssz > 64*1024*1024) {
            printf("  Skipping (invalid size)\n");
            foff += ssz;
            continue;
        }

        uint8_t *buf = malloc(ssz);
        if (!buf) { perror("malloc"); break; }

        int rd = cdrom_read_file(cd, &ve, buf, foff, ssz);
        printf("  Read %d bytes\n", rd);

        char outname[256];
        snprintf(outname, sizeof(outname), "/tmp/vff_sec%u.bin", si);
        FILE *fp = fopen(outname, "wb");
        if (fp) {
            fwrite(buf, 1, rd, fp);
            fclose(fp);
            printf("  → %s\n", outname);
        }

        /* Show first 64 bytes */
        printf("  First 64 bytes:\n  ");
        for (int j = 0; j < 64 && j < rd; j++) {
            printf("%02X ", buf[j]);
            if ((j & 15) == 15) printf("\n  ");
        }
        printf("\n");

        /* For sec[0]: find ARM code start */
        if (si == 0) {
            int arm_start = -1;
            for (int j = 0; j < rd - 4; j += 4) {
                uint32_t w = *(uint32_t*)(buf + j);
                if (w != 0) {
                    arm_start = j;
                    break;
                }
            }
            if (arm_start >= 0) {
                printf("  ARM code starts at offset 0x%X (addr 0x%08X)\n",
                       arm_start, dst + arm_start);
                printf("  First instructions:\n");
                for (int j = arm_start; j < arm_start + 40 && j < rd; j += 4) {
                    uint32_t w = *(uint32_t*)(buf + j);
                    printf("    %08X: %08X\n", dst + j, w);
                }
            }
        }

        free(buf);
        foff += ssz;
    }

    cdrom_destroy(cd);
    printf("\nDone. Load sec[0] in Ghidra at base 0x%08X (ARM LE)\n",
           *(uint32_t*)(vhdr + 0x30));
    return 0;
}
