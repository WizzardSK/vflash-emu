/* ptx_extract — V.Flash .ptx image extractor
 * Extracts .ptx raw bitmap files from disc or file and saves as PNG
 *
 * Usage:
 *   ptx_extract file.ptx [out.png]
 *   ptx_extract disc.iso [outdir]
 *
 * Output: PNG files decoded from ptx_decode() in ptx.c
 *
 * PNG encoding: uses libpng if available, else writes PPM fallback.
 */

#include "ptx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <errno.h>

#ifdef HAVE_PNG
#include <png.h>
#endif

/* ---- Minimal ISO 9660 reader (duplicated from mjp_extract for standalone build) ---- */

static FILE *g_iso = NULL;

static int iso_read_sector(uint32_t lba, uint8_t *out) {
    if (!g_iso) return 0;
    if (fseek(g_iso, lba * 2048, SEEK_SET) != 0) return 0;
    return fread(out, 1, 2048, g_iso) == 2048;
}

typedef struct {
    char     name[512];
    uint32_t lba;
    uint32_t size;
    int      is_dir;
} Ent;

static int iso_list_dir(uint32_t lba, uint32_t size, Ent *out, int max) {
    uint8_t sec[2048];
    int n = 0;
    uint32_t read = 0;
    while (read < size && n < max) {
        if (!iso_read_sector(lba + read / 2048, sec)) break;
        uint32_t off = 0;
        while (off < 2048) {
            uint8_t rl = sec[off];
            if (!rl) { off = 2048; break; }
            uint8_t nlen = sec[off + 32];
            int nl = nlen < 255 ? nlen : 255;
            memcpy(out[n].name, sec + off + 33, nl);
            out[n].name[nl] = 0;
            char *semi = strchr(out[n].name, ';');
            if (semi) *semi = 0;
            out[n].lba    = *(uint32_t*)(sec + off + 2);
            out[n].size   = *(uint32_t*)(sec + off + 10);
            out[n].is_dir = (sec[off + 25] & 2) ? 1 : 0;
            if (out[n].name[0] && strcmp(out[n].name, ".") && strcmp(out[n].name, ".."))
                n++;
            off += rl;
        }
        read += 2048;
    }
    return n;
}

static int find_ptx(uint32_t dlba, uint32_t dsz, const char *path,
                    Ent *res, int maxr, int *found) {
    Ent ents[512];
    int n = iso_list_dir(dlba, dsz, ents, 512);
    for (int i = 0; i < n && *found < maxr; i++) {
        char fp[512];
        snprintf(fp, sizeof(fp), "%s/%s", path, ents[i].name);
        if (ents[i].is_dir) {
            find_ptx(ents[i].lba, ents[i].size, fp, res, maxr, found);
        } else {
            int nl = strlen(ents[i].name);
            if (nl >= 4) {
                const char *ext = ents[i].name + nl - 4;
                if (strcasecmp(ext, ".ptx") == 0 ||
                    strcasecmp(ext, ".pix") == 0 ||
                    strcasecmp(ext, ".raw") == 0) {
                    res[*found] = ents[i];
                    snprintf(res[*found].name, 512, "%s", fp);
                    (*found)++;
                    printf("[Scan] Found: %s (%u bytes)\n", fp, ents[i].size);
                }
            }
        }
    }
    return *found;
}

/* ---- Image writers ---- */

static int write_ppm(const char *path, const uint32_t *pixels,
                     uint32_t w, uint32_t h) {
    FILE *f = fopen(path, "wb");
    if (!f) { fprintf(stderr, "Cannot write %s: %s\n", path, strerror(errno)); return 0; }
    fprintf(f, "P6\n%u %u\n255\n", w, h);
    for (uint32_t i = 0; i < w * h; i++) {
        uint32_t p = pixels[i];
        uint8_t rgb[3] = {
            (p >> 16) & 0xFF,   /* R */
            (p >>  8) & 0xFF,   /* G */
            (p >>  0) & 0xFF    /* B */
        };
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
    return 1;
}

#ifdef HAVE_PNG
static int write_png(const char *path, const uint32_t *pixels,
                     uint32_t w, uint32_t h) {
    FILE *f = fopen(path, "wb");
    if (!f) return 0;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop   info = png_create_info_struct(png);
    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(f); return 0;
    }

    png_init_io(png, f);
    png_set_IHDR(png, info, w, h, 8,
                 PNG_COLOR_TYPE_RGB_ALPHA,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    uint8_t *row = malloc(w * 4);
    for (uint32_t y = 0; y < h; y++) {
        const uint32_t *src = pixels + y * w;
        for (uint32_t x = 0; x < w; x++) {
            uint32_t p = src[x];
            row[x * 4 + 0] = (p >> 16) & 0xFF;  /* R */
            row[x * 4 + 1] = (p >>  8) & 0xFF;  /* G */
            row[x * 4 + 2] = (p >>  0) & 0xFF;  /* B */
            row[x * 4 + 3] = (p >> 24) & 0xFF;  /* A */
        }
        png_write_row(png, row);
    }
    free(row);

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(f);
    return 1;
}
#endif

static int save_image(const char *path, const uint32_t *pixels,
                      uint32_t w, uint32_t h) {
#ifdef HAVE_PNG
    return write_png(path, pixels, w, h);
#else
    /* fallback: write PPM (change .png extension to .ppm) */
    char ppmpath[512];
    strncpy(ppmpath, path, sizeof(ppmpath) - 1);
    char *dot = strrchr(ppmpath, '.');
    if (dot) strcpy(dot, ".ppm");
    return write_ppm(ppmpath, pixels, w, h);
#endif
}

static uint8_t *iso_read_file(Ent *e) {
    uint8_t *buf = malloc(e->size);
    if (!buf) return NULL;
    uint32_t done = 0;
    while (done < e->size) {
        uint8_t sec[2048];
        if (!iso_read_sector(e->lba + done / 2048, sec)) { free(buf); return NULL; }
        uint32_t chunk = e->size - done < 2048 ? e->size - done : 2048;
        memcpy(buf + done, sec, chunk);
        done += chunk;
    }
    return buf;
}

static void mkdir_p(const char *path) {
    char tmp[512];
    strncpy(tmp, path, sizeof(tmp) - 1);
    for (char *p = tmp + 1; *p; p++) {
        if (*p == '/') { *p = 0; mkdir(tmp, 0755); *p = '/'; }
    }
    mkdir(tmp, 0755);
}

static int is_iso_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    uint8_t buf[5];
    int ok = fseek(f, 16 * 2048 + 1, SEEK_SET) == 0 &&
             fread(buf, 1, 5, f) == 5 &&
             memcmp(buf, "CD001", 5) == 0;
    fclose(f);
    return ok;
}

static int process_ptx(const uint8_t *data, uint32_t size,
                        const char *outpath, const char *label) {
    PTXImage *img = ptx_decode(data, size);
    if (!img) {
        fprintf(stderr, "[PTX] Decode failed: %s\n", label);
        return 0;
    }

    if (save_image(outpath, img->pixels, img->width, img->height)) {
        printf("[PTX] %s → %ux%u fmt=%d → %s\n",
               label, img->width, img->height, img->format, outpath);
        ptx_destroy(img);
        return 1;
    }

    ptx_destroy(img);
    return 0;
}

/* ---- Main ---- */

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr,
            "ptx_extract — V.Flash .ptx image extractor\n\n"
            "Usage:\n"
            "  %s file.ptx [out.png]      extract single file\n"
            "  %s disc.iso [outdir]       extract all .ptx from disc\n\n"
#ifdef HAVE_PNG
            "Output: PNG\n",
#else
            "Output: PPM (build with -DHAVE_PNG -lpng for PNG support)\n",
#endif
            argv[0], argv[0]);
        return 1;
    }

    const char *inpath = argv[1];

    if (is_iso_file(inpath)) {
        /* ISO disc mode */
        const char *outdir = argc >= 3 ? argv[2] : "ptx_out";
        mkdir_p(outdir);

        g_iso = fopen(inpath, "rb");
        if (!g_iso) { perror(inpath); return 1; }

        uint8_t pvd[2048];
        if (!iso_read_sector(16, pvd)) {
            fprintf(stderr, "Cannot read PVD\n");
            fclose(g_iso); return 1;
        }

        uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
        uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);

        Ent ptx_files[2048];
        int found = 0;
        find_ptx(root_lba, root_size, "", ptx_files, 2048, &found);
        printf("[ISO] Found %d .ptx file(s)\n\n", found);

        int ok = 0;
        for (int i = 0; i < found; i++) {
            uint8_t *buf = iso_read_file(&ptx_files[i]);
            if (!buf) { fprintf(stderr, "[ISO] Read error: %s\n", ptx_files[i].name); continue; }

            /* Build output path: outdir/<disc_path>.png */
            const char *base = strrchr(ptx_files[i].name, '/');
            base = base ? base + 1 : ptx_files[i].name;
            char outpath[512];
            snprintf(outpath, sizeof(outpath), "%s/%04d_%s.png", outdir, i, base);

            ok += process_ptx(buf, ptx_files[i].size, outpath, ptx_files[i].name);
            free(buf);
        }

        printf("\n[Done] %d/%d image(s) decoded → %s/\n", ok, found, outdir);
        fclose(g_iso);

    } else {
        /* Single file mode */
        FILE *f = fopen(inpath, "rb");
        if (!f) { perror(inpath); return 1; }
        fseek(f, 0, SEEK_END);
        long sz = ftell(f);
        fseek(f, 0, SEEK_SET);
        uint8_t *data = malloc(sz);
        if (fread(data, 1, (size_t)sz, f) != (size_t)sz) { fprintf(stderr, "Read error\n"); free(data); fclose(f); return 1; }
        fclose(f);

        char outpath[512];
        if (argc >= 3) {
            strncpy(outpath, argv[2], sizeof(outpath) - 1);
        } else {
            strncpy(outpath, inpath, sizeof(outpath) - 5);
            char *dot = strrchr(outpath, '.');
            if (dot) strcpy(dot, ".png");
            else strncat(outpath, ".png", 4);
        }

        int ok = process_ptx(data, (uint32_t)sz, outpath, inpath);
        free(data);
        return ok ? 0 : 1;
    }

    return 0;
}
