#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* V.Flash Disc Analyzer
 * Lists all files in ISO 9660, identifies ARM executables,
 * dumps headers of .mjp/.ptx/.snd files
 */

#define SECTOR_SIZE 2048

static FILE *iso;

static int read_sector(uint32_t lba, uint8_t *buf) {
    fseek(iso, (long)lba * SECTOR_SIZE, SEEK_SET);
    return fread(buf, 1, SECTOR_SIZE, iso) == SECTOR_SIZE;
}

static int read_file_data(uint32_t lba, uint32_t size, uint8_t *buf) {
    uint32_t sectors = (size + SECTOR_SIZE - 1) / SECTOR_SIZE;
    uint32_t read = 0;
    uint8_t sector[SECTOR_SIZE];
    for (uint32_t i = 0; i < sectors && read < size; i++) {
        if (!read_sector(lba + i, sector)) break;
        uint32_t chunk = SECTOR_SIZE;
        if (read + chunk > size) chunk = size - read;
        memcpy(buf + read, sector, chunk);
        read += chunk;
    }
    return read;
}

/* Check if data looks like an ARM ELF */
static int is_elf(const uint8_t *d, uint32_t sz) {
    return sz > 52 && d[0]==0x7F && d[1]=='E' && d[2]=='L' && d[3]=='F';
}

/* Check if data looks like raw ARM code */
static int is_arm_raw(const uint8_t *d, uint32_t sz) {
    if (sz < 8) return 0;
    uint32_t w0 = *(uint32_t*)d;
    uint32_t w1 = *(uint32_t*)(d+4);
    if ((w0 >> 24) == 0xEA || (w0 >> 24) == 0xEB) return 1;
    if ((w1 >> 24) == 0xEA || (w1 >> 24) == 0xEB) return 1;
    if ((w0 & 0xFFFF0000) == 0xE59F0000) return 1;
    return 0;
}

/* Probe .mjp file */
static void probe_mjp(const char *name, uint32_t lba, uint32_t size) {
    uint8_t hdr[16];
    uint8_t sector[SECTOR_SIZE];
    if (!read_sector(lba, sector)) return;
    memcpy(hdr, sector, 16);
    printf("    [MJP] %s %u bytes — first bytes: %02X %02X %02X %02X %02X %02X %02X %02X\n",
           name, size, hdr[0],hdr[1],hdr[2],hdr[3],hdr[4],hdr[5],hdr[6],hdr[7]);
    /* JPEG magic: FF D8 FF */
    if (hdr[0]==0xFF && hdr[1]==0xD8 && hdr[2]==0xFF)
        printf("    [MJP] -> Valid JPEG header detected\n");
}

/* Probe .ptx file */
static void probe_ptx(const char *name, uint32_t lba, uint32_t size) {
    uint8_t sector[SECTOR_SIZE];
    if (!read_sector(lba, sector)) return;
    uint16_t w = *(uint16_t*)sector;
    uint16_t h = *(uint16_t*)(sector+2);
    printf("    [PTX] %s %u bytes — first u16s: %u %u", name, size, w, h);
    if (w > 0 && w <= 640 && h > 0 && h <= 480)
        printf(" (possible %ux%u image)", w, h);
    printf("\n");
    /* Check if RGB565 size matches */
    if (size == (uint32_t)w * h * 2 + 4)
        printf("    [PTX] -> Size matches %ux%u RGB565 with 4-byte header\n", w, h);
    else if (size == (uint32_t)w * h * 2)
        printf("    [PTX] -> Size matches %ux%u RGB565 headerless\n", w, h);
}

/* Probe .snd file */
static void probe_snd(const char *name, uint32_t lba, uint32_t size) {
    uint8_t sector[SECTOR_SIZE];
    if (!read_sector(lba, sector)) return;
    printf("    [SND] %s %u bytes", name, size);
    if (sector[0]=='R' && sector[1]=='I' && sector[2]=='F' && sector[3]=='F')
        printf(" — WAV/RIFF header");
    else
        printf(" — raw PCM (%u samples @ 16-bit)", size/2);
    printf("\n");
}

/* Probe ARM binary */
static void probe_bin(const char *name, uint32_t lba, uint32_t size) {
    uint8_t buf[256];
    uint32_t read_sz = size < 256 ? size : 256;
    if (!read_file_data(lba, read_sz, buf)) return;

    printf("    [BIN] %s %u bytes\n", name, size);
    if (is_elf(buf, read_sz)) {
        uint32_t entry = *(uint32_t*)(buf + 24);
        uint16_t machine = *(uint16_t*)(buf + 18);
        printf("    [BIN] -> ELF: machine=0x%04X entry=0x%08X\n", machine, entry);
        if (machine == 0x28) printf("    [BIN] -> ARM ELF confirmed!\n");
    } else if (is_arm_raw(buf, read_sz)) {
        printf("    [BIN] -> Raw ARM binary (first word: 0x%08X)\n", *(uint32_t*)buf);
    } else {
        printf("    [BIN] -> Unknown format (first bytes: %02X %02X %02X %02X)\n",
               buf[0], buf[1], buf[2], buf[3]);
    }
}

/* Recursive directory listing */
static void list_dir(uint32_t dir_lba, uint32_t dir_size, int depth, const char *path) {
    uint8_t sector[SECTOR_SIZE];
    uint32_t remaining = dir_size;
    uint32_t lba = dir_lba;

    while (remaining > 0) {
        if (!read_sector(lba, sector)) break;
        uint32_t off = 0;

        while (off < SECTOR_SIZE) {
            uint8_t rec_len = sector[off];
            if (rec_len == 0) break;

            uint8_t name_len = sector[off + 32];
            if (name_len == 0 || off + rec_len > SECTOR_SIZE) break;

            char name[256] = {0};
            memcpy(name, sector + off + 33, name_len < 255 ? name_len : 255);

            /* Strip version ;1 */
            char *semi = strchr(name, ';');
            if (semi) *semi = 0;

            /* Skip . and .. */
            if (strcmp(name, "\x00") != 0 && strcmp(name, "\x01") != 0 &&
                name[0] != '\x00' && name[0] != '\x01') {

                uint32_t entry_lba  = *(uint32_t*)(sector + off + 2);
                uint32_t entry_size = *(uint32_t*)(sector + off + 10);
                int is_dir = (sector[off + 25] & 0x02) != 0;

                for (int i = 0; i < depth; i++) printf("  ");
                if (is_dir) {
                    printf("[DIR] %s/\n", name);
                    char newpath[512];
                    snprintf(newpath, sizeof(newpath), "%s%s/", path, name);
                    list_dir(entry_lba, entry_size, depth + 1, newpath);
                } else {
                    printf("[FILE] %s/%s (%u bytes, LBA %u)\n", path, name, entry_size, entry_lba);

                    /* Probe known extensions */
                    char *ext = strrchr(name, '.');
                    if (ext) {
                        if (strcasecmp(ext, ".mjp") == 0) probe_mjp(name, entry_lba, entry_size);
                        else if (strcasecmp(ext, ".ptx") == 0) probe_ptx(name, entry_lba, entry_size);
                        else if (strcasecmp(ext, ".snd") == 0) probe_snd(name, entry_lba, entry_size);
                        else if (strcasecmp(ext, ".bin") == 0 ||
                                 strcasecmp(ext, ".exe") == 0 ||
                                 strcasecmp(ext, ".elf") == 0) probe_bin(name, entry_lba, entry_size);
                    }
                }
            }

            off += rec_len;
        }

        lba++;
        remaining -= (remaining > SECTOR_SIZE ? SECTOR_SIZE : remaining);
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <disc.iso>\n", argv[0]);
        return 1;
    }

    iso = fopen(argv[1], "rb");
    if (!iso) { perror("fopen"); return 1; }

    fseek(iso, 0, SEEK_END);
    long iso_size = ftell(iso);
    printf("=== V.Flash Disc Analyzer ===\n");
    printf("File: %s\n", argv[1]);
    printf("Size: %ld bytes (%ld sectors)\n\n", iso_size, iso_size / SECTOR_SIZE);

    /* Read PVD */
    uint8_t pvd[SECTOR_SIZE];
    if (!read_sector(16, pvd)) { fprintf(stderr, "Cannot read PVD\n"); fclose(iso); return 1; }

    if (pvd[0] != 1) { fprintf(stderr, "Not a Primary Volume Descriptor\n"); fclose(iso); return 1; }

    char vol_id[33] = {0};
    memcpy(vol_id, pvd + 40, 32);
    /* trim trailing spaces */
    for (int i = 31; i >= 0 && vol_id[i] == ' '; i--) vol_id[i] = 0;

    char sys_id[33] = {0};
    memcpy(sys_id, pvd + 8, 32);
    for (int i = 31; i >= 0 && sys_id[i] == ' '; i--) sys_id[i] = 0;

    printf("Volume ID:  %s\n", vol_id);
    printf("System ID:  %s\n", sys_id);

    uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
    uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
    printf("Root LBA:   %u\n", root_lba);
    printf("Root size:  %u bytes\n\n", root_size);

    printf("=== File listing ===\n");
    list_dir(root_lba, root_size, 0, "/");

    fclose(iso);
    return 0;
}
