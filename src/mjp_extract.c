/* mjp_extract — V.Flash Motion JPEG extractor
 * Extracts all JPEG frames from a .mjp file or directly from ISO 9660 disc
 *
 * Usage:
 *   mjp_extract file.mjp [outdir]        — extract from .mjp file
 *   mjp_extract disc.iso [outdir]        — extract all .mjp from disc
 *   mjp_extract disc.iso game/video.mjp  — extract specific file from disc
 *
 * Output: frame_0000.jpg, frame_0001.jpg, ...
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <errno.h>

/* ---- Minimal ISO 9660 reader ---- */

typedef struct {
    char     name[256];
    uint32_t lba;
    uint32_t size;
    int      is_dir;
} ISOEntry;

static FILE *g_iso = NULL;

static int iso_read_sector(uint32_t lba, uint8_t *out) {
    if (!g_iso) return 0;
    if (fseek(g_iso, lba * 2048, SEEK_SET) != 0) return 0;
    return fread(out, 1, 2048, g_iso) == 2048;
}

static void iso_parse_name(const uint8_t *de, char *name_out, int maxlen) {
    uint8_t nlen = de[32];
    int n = nlen < maxlen - 1 ? nlen : maxlen - 1;
    memcpy(name_out, de + 33, n);
    name_out[n] = 0;
    /* Strip version suffix ;1 */
    char *semi = strchr(name_out, ';');
    if (semi) *semi = 0;
}

/* List all files in a directory (non-recursive for simplicity) */
static int iso_list_dir(uint32_t dir_lba, uint32_t dir_size,
                         ISOEntry *entries, int maxentries) {
    uint8_t sector[2048];
    int count = 0;
    uint32_t read = 0;

    while (read < dir_size && count < maxentries) {
        uint32_t lba = dir_lba + read / 2048;
        if (!iso_read_sector(lba, sector)) break;

        uint32_t off = read % 2048;
        while (off < 2048 && off < (dir_size - (lba - dir_lba) * 2048)) {
            uint8_t reclen = sector[off];
            if (reclen == 0) { off = 2048; break; }  /* end of sector */

            uint32_t e_lba  = *(uint32_t*)(sector + off + 2);
            uint32_t e_size = *(uint32_t*)(sector + off + 10);
            uint8_t  flags  = sector[off + 25];

            ISOEntry *e = &entries[count];
            iso_parse_name(sector + off, e->name, sizeof(e->name));
            e->lba    = e_lba;
            e->size   = e_size;
            e->is_dir = (flags & 0x02) ? 1 : 0;

            if (e->name[0] != '\0' && strcmp(e->name, ".") && strcmp(e->name, ".."))
                count++;

            off += reclen;
        }
        read += 2048;
    }
    return count;
}

/* Recursively find all .mjp files on disc */
static int find_mjp_files(uint32_t dir_lba, uint32_t dir_size,
                           const char *path,
                           ISOEntry *results, int maxresults, int *found) {
    ISOEntry entries[256];
    int n = iso_list_dir(dir_lba, dir_size, entries, 256);

    for (int i = 0; i < n && *found < maxresults; i++) {
        char fullpath[512];
        snprintf(fullpath, sizeof(fullpath), "%s/%s", path, entries[i].name);

        if (entries[i].is_dir) {
            find_mjp_files(entries[i].lba, entries[i].size,
                           fullpath, results, maxresults, found);
        } else {
            /* Check for .MJP or .mjp extension */
            int nlen = strlen(entries[i].name);
            if (nlen >= 4) {
                const char *ext = entries[i].name + nlen - 4;
                if (strcasecmp(ext, ".mjp") == 0 || strcasecmp(ext, ".mjv") == 0) {
                    results[*found] = entries[i];
                    strncpy(results[*found].name, fullpath,
                            sizeof(results[*found].name) - 1);
                    (*found)++;
                    printf("[Scan] Found: %s (%u bytes)\n", fullpath, entries[i].size);
                }
            }
        }
    }
    return *found;
}

/* ---- MJP frame splitter ---- */
/* MJP is a concatenation of standard JPEG files.
 * Each JPEG starts with 0xFFD8 and ends with 0xFFD9. */

static int extract_frames(const uint8_t *data, uint32_t size,
                           const char *outdir, const char *prefix,
                           int *frame_base) {
    int frames = 0;
    uint32_t i = 0;

    while (i + 4 < size) {
        /* Find JPEG SOI marker */
        if (data[i] != 0xFF || data[i + 1] != 0xD8) {
            i++;
            continue;
        }

        /* Find matching EOI marker */
        uint32_t start = i;
        uint32_t j = i + 2;
        int found_eoi = 0;

        while (j + 1 < size) {
            if (data[j] == 0xFF && data[j + 1] == 0xD9) {
                j += 2;
                found_eoi = 1;
                break;
            }
            /* Skip over segment data to avoid false EOI */
            if (data[j] == 0xFF && data[j + 1] != 0x00 &&
                data[j + 1] != 0xFF && data[j + 1] != 0xD9) {
                if (j + 3 < size) {
                    uint16_t seg_len = (data[j + 2] << 8) | data[j + 3];
                    j += 2 + seg_len;
                    continue;
                }
            }
            j++;
        }

        if (!found_eoi) break;

        /* Write JPEG frame */
        char fname[512];
        snprintf(fname, sizeof(fname), "%s/%s%04d.jpg",
                 outdir, prefix, *frame_base + frames);

        FILE *f = fopen(fname, "wb");
        if (f) {
            fwrite(data + start, 1, j - start, f);
            fclose(f);
            frames++;
            if (frames <= 5 || frames % 50 == 0)
                printf("[MJP] Frame %04d: offset=%u size=%u → %s\n",
                       *frame_base + frames - 1, start, j - start, fname);
        } else {
            fprintf(stderr, "[MJP] Cannot write: %s (%s)\n", fname, strerror(errno));
        }

        i = j;
    }

    *frame_base += frames;
    return frames;
}

/* ---- Main ---- */

static void mkdir_p(const char *path) {
    char tmp[512];
    strncpy(tmp, path, sizeof(tmp) - 1);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') {
            *p = 0;
            mkdir(tmp, 0755);
            *p = '/';
        }
    }
    mkdir(tmp, 0755);
}

static int is_iso(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    uint8_t buf[8];
    if (fseek(f, 16 * 2048 + 1, SEEK_SET) != 0) { fclose(f); return 0; }
    int ok = fread(buf, 1, 5, f) == 5 &&
             memcmp(buf, "CD001", 5) == 0;
    fclose(f);
    return ok;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,
            "mjp_extract — V.Flash Motion JPEG extractor\n\n"
            "Usage:\n"
            "  %s file.mjp [outdir]        extract .mjp file\n"
            "  %s disc.iso [outdir]        extract all .mjp from disc\n\n"
            "Output: frame_0000.jpg, frame_0001.jpg, ...\n",
            argv[0], argv[0]);
        return 1;
    }

    const char *inpath  = argv[1];
    const char *outdir  = argc >= 3 ? argv[2] : "frames";

    mkdir_p(outdir);

    if (is_iso(inpath)) {
        /* ---- ISO disc mode ---- */
        printf("[ISO] Scanning disc: %s\n", inpath);
        g_iso = fopen(inpath, "rb");
        if (!g_iso) { perror(inpath); return 1; }

        /* Read PVD */
        uint8_t pvd[2048];
        if (!iso_read_sector(16, pvd)) {
            fprintf(stderr, "Cannot read PVD\n");
            fclose(g_iso);
            return 1;
        }

        uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
        uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);
        printf("[ISO] Root dir: LBA=%u size=%u\n", root_lba, root_size);

        ISOEntry mjp_files[1024];
        int found = 0;
        find_mjp_files(root_lba, root_size, "", mjp_files, 1024, &found);
        printf("[ISO] Found %d .mjp file(s)\n\n", found);

        int total_frames = 0;
        for (int i = 0; i < found; i++) {
            ISOEntry *e = &mjp_files[i];
            uint8_t *buf = malloc(e->size);
            if (!buf) continue;

            /* Read file from disc */
            uint32_t bytes = 0;
            while (bytes < e->size) {
                uint32_t lba = e->lba + bytes / 2048;
                uint8_t sector[2048];
                if (!iso_read_sector(lba, sector)) break;
                uint32_t chunk = e->size - bytes;
                if (chunk > 2048) chunk = 2048;
                memcpy(buf + bytes, sector, chunk);
                bytes += chunk;
            }

            if (bytes < e->size) {
                fprintf(stderr, "[ISO] Read error: %s\n", e->name);
                free(buf);
                continue;
            }

            /* Build output prefix from filename */
            const char *fname = strrchr(e->name, '/');
            fname = fname ? fname + 1 : e->name;
            char prefix[64];
            snprintf(prefix, sizeof(prefix), "%.63s", fname);
            char *dot = strrchr(prefix, '.');
            if (dot) { *dot = '_'; }
            strncat(prefix, "_", sizeof(prefix) - strlen(prefix) - 1);

            printf("\n[MJP] Extracting: %s (%u bytes)\n", e->name, e->size);
            int base = total_frames;
            int n = extract_frames(buf, bytes, outdir, prefix, &base);
            printf("[MJP] %d frame(s) extracted\n", n);
            total_frames = base;
            free(buf);
        }

        printf("\n[Done] Total frames: %d → %s/\n", total_frames, outdir);
        fclose(g_iso);

    } else {
        /* ---- Raw .mjp file mode ---- */
        FILE *f = fopen(inpath, "rb");
        if (!f) { perror(inpath); return 1; }

        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);

        uint8_t *data = malloc(sz);
        if (!data) { fprintf(stderr, "Out of memory\n"); fclose(f); return 1; }
        if (fread(data, 1, (size_t)sz, f) != (size_t)sz) { fprintf(stderr, "Read error\n"); free(data); fclose(f); return 1; }
        fclose(f);

        printf("[MJP] File: %s (%ld bytes)\n", inpath, sz);

        int base = 0;
        int n = extract_frames(data, (uint32_t)sz, outdir, "frame_", &base);
        printf("\n[Done] %d frame(s) extracted → %s/\n", n, outdir);
        free(data);
    }

    return 0;
}
