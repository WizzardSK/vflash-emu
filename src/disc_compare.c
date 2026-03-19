/* disc_compare — V.Flash disc structure comparator
 * Compares file layouts across multiple ISO 9660 discs to find
 * common patterns: fixed-address resources, engine files, per-game data.
 *
 * Usage:
 *   disc_compare disc1.iso disc2.iso [disc3.iso ...]
 *
 * Output:
 *   - Files present in ALL discs (engine/shared code)
 *   - Files present in SOME discs
 *   - File size differences for same-named files
 *   - LBA layout summary (fixed vs variable addresses)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MAX_DISCS    16
#define MAX_FILES    4096

typedef struct {
    char     path[512];   /* Full ISO path e.g. /GAME/BOOT.BIN */
    uint32_t lba;
    uint32_t size;
} FileEntry;

typedef struct {
    char       disc_path[512];
    FileEntry  files[MAX_FILES];
    int        nfiles;
    char       label[33];    /* ISO volume label */
} DiscInfo;

/* ---- ISO 9660 reader ---- */

static FILE *g_f = NULL;

static int sec_read(uint32_t lba, uint8_t *out) {
    if (!g_f) return 0;
    if (fseek(g_f, (long)lba * 2048, SEEK_SET)) return 0;
    return fread(out, 1, 2048, g_f) == 2048;
}

static void parse_name(const uint8_t *de, char *out, int max) {
    uint8_t nl = de[32];
    int n = nl < max - 1 ? nl : max - 1;
    memcpy(out, de + 33, n);
    out[n] = 0;
    char *s = strchr(out, ';');
    if (s) *s = 0;
}

static void scan_dir(uint32_t dlba, uint32_t dsz, const char *path,
                     FileEntry *files, int *n, int max) {
    uint8_t sec[2048];
    uint32_t done = 0;

    while (done < dsz && *n < max) {
        if (!sec_read(dlba + done / 2048, sec)) break;
        uint32_t off = 0;
        while (off < 2048 && *n < max) {
            uint8_t rl = sec[off];
            if (!rl) { off = 2048; break; }

            uint32_t lba  = *(uint32_t*)(sec + off + 2);
            uint32_t size = *(uint32_t*)(sec + off + 10);
            uint8_t  fl   = sec[off + 25];
            char name[256];
            parse_name(sec + off, name, sizeof(name));

            if (name[0] && strcmp(name, ".") && strcmp(name, "..")) {
                char fullpath[512];
                snprintf(fullpath, sizeof(fullpath), "%s/%s", path, name);

                if (fl & 2) {
                    /* Directory — recurse */
                    scan_dir(lba, size, fullpath, files, n, max);
                } else {
                    snprintf(files[*n].path, 512, "%s", fullpath);
                    files[*n].lba  = lba;
                    files[*n].size = size;
                    (*n)++;
                }
            }
            off += rl;
        }
        done += 2048;
    }
}

static int load_disc(const char *isopath, DiscInfo *di) {
    g_f = fopen(isopath, "rb");
    if (!g_f) { perror(isopath); return 0; }

    strncpy(di->disc_path, isopath, 511);
    di->nfiles = 0;

    uint8_t pvd[2048];
    if (!sec_read(16, pvd)) {
        fprintf(stderr, "[%s] Cannot read PVD\n", isopath);
        fclose(g_f); g_f = NULL; return 0;
    }

    /* Volume label at bytes 40..71 */
    memcpy(di->label, pvd + 40, 32);
    di->label[32] = 0;
    /* Trim trailing spaces */
    for (int i = 31; i >= 0 && di->label[i] == ' '; i--)
        di->label[i] = 0;

    uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
    uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);

    scan_dir(root_lba, root_size, "", di->files, &di->nfiles, MAX_FILES);

    fclose(g_f);
    g_f = NULL;

    printf("[Load] %s — \"%s\" — %d files\n", isopath, di->label, di->nfiles);
    return 1;
}

/* ---- Comparison logic ---- */

typedef struct {
    char     path[512];
    uint32_t size[MAX_DISCS];    /* 0 = not present */
    uint32_t lba[MAX_DISCS];
    int      present[MAX_DISCS];
    int      ndiscs;
} FileSummary;

static FileSummary summary[MAX_FILES * MAX_DISCS];
static int nsummary = 0;

static FileSummary *find_or_add(const char *path) {
    for (int i = 0; i < nsummary; i++)
        if (strcmp(summary[i].path, path) == 0)
            return &summary[i];
    if (nsummary >= MAX_FILES * MAX_DISCS) return NULL;
    strncpy(summary[nsummary].path, path, 511);
    return &summary[nsummary++];
}

static int cmp_path(const void *a, const void *b) {
    return strcmp(((FileSummary*)a)->path, ((FileSummary*)b)->path);
}

static const char *human_size(uint32_t sz) {
    static char buf[32];
    if (sz >= 1024 * 1024)
        snprintf(buf, sizeof(buf), "%u MB", sz / (1024 * 1024));
    else if (sz >= 1024)
        snprintf(buf, sizeof(buf), "%u KB", sz / 1024);
    else
        snprintf(buf, sizeof(buf), "%u B", sz);
    return buf;
}

static const char *ext_of(const char *path) {
    const char *dot = strrchr(path, '.');
    return dot ? dot + 1 : "";
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr,
            "disc_compare — V.Flash disc structure comparator\n\n"
            "Usage: %s disc1.iso disc2.iso [disc3.iso ...]\n\n"
            "Finds common files (engine), per-game files, and size differences.\n",
            argv[0]);
        return 1;
    }

    int ndiscs = argc - 1;
    if (ndiscs > MAX_DISCS) ndiscs = MAX_DISCS;

    DiscInfo *discs = calloc(ndiscs, sizeof(DiscInfo));
    if (!discs) { fprintf(stderr, "Out of memory\n"); return 1; }

    /* Load all discs */
    printf("=== Loading discs ===\n");
    int loaded = 0;
    for (int i = 0; i < ndiscs; i++) {
        if (load_disc(argv[i + 1], &discs[loaded]))
            loaded++;
    }
    ndiscs = loaded;
    printf("\n");

    if (ndiscs < 2) {
        fprintf(stderr, "Need at least 2 readable discs\n");
        free(discs);
        return 1;
    }

    /* Build summary table */
    for (int d = 0; d < ndiscs; d++) {
        for (int f = 0; f < discs[d].nfiles; f++) {
            FileSummary *s = find_or_add(discs[d].files[f].path);
            if (!s) continue;
            s->present[d] = 1;
            s->size[d]    = discs[d].files[f].size;
            s->lba[d]     = discs[d].files[f].lba;
            s->ndiscs     = ndiscs;
        }
    }

    qsort(summary, nsummary, sizeof(FileSummary), cmp_path);

    /* Categorize */
    int n_all = 0, n_unique = 0;
    int n_size_diff = 0, n_lba_fixed = 0;

    printf("=== Files present in ALL %d discs (shared engine) ===\n", ndiscs);
    for (int i = 0; i < nsummary; i++) {
        FileSummary *s = &summary[i];
        int cnt = 0;
        for (int d = 0; d < ndiscs; d++) cnt += s->present[d];
        if (cnt < ndiscs) continue;

        n_all++;

        /* Check size consistency */
        int same_size = 1;
        for (int d = 1; d < ndiscs; d++)
            if (s->size[d] != s->size[0]) { same_size = 0; break; }

        /* Check LBA consistency (fixed address = likely hardcoded by engine) */
        int same_lba = 1;
        for (int d = 1; d < ndiscs; d++)
            if (s->lba[d] != s->lba[0]) { same_lba = 0; break; }

        if (same_lba) n_lba_fixed++;
        if (!same_size) n_size_diff++;

        printf("  %-50s  %8s%s%s\n",
               s->path,
               human_size(s->size[0]),
               same_size ? "" : " [SIZE DIFF]",
               same_lba  ? " [FIXED LBA]" : "");
    }
    printf("Total: %d files\n\n", n_all);

    printf("=== Files with SIZE DIFFERENCES across discs ===\n");
    for (int i = 0; i < nsummary; i++) {
        FileSummary *s = &summary[i];
        int cnt = 0;
        for (int d = 0; d < ndiscs; d++) cnt += s->present[d];
        if (cnt < ndiscs) continue;

        int same = 1;
        for (int d = 1; d < ndiscs; d++)
            if (s->size[d] != s->size[0]) { same = 0; break; }
        if (same) continue;

        printf("  %s\n", s->path);
        for (int d = 0; d < ndiscs; d++) {
            const char *disc = strrchr(discs[d].disc_path, '/');
            disc = disc ? disc + 1 : discs[d].disc_path;
            printf("    [%d] %-30s  %8s  LBA=%u\n",
                   d, disc, human_size(s->size[d]), s->lba[d]);
        }
    }
    printf("\n");

    printf("=== Files present in only 1 disc (per-game exclusive) ===\n");
    for (int i = 0; i < nsummary; i++) {
        FileSummary *s = &summary[i];
        int cnt = 0, which = -1;
        for (int d = 0; d < ndiscs; d++)
            if (s->present[d]) { cnt++; which = d; }
        if (cnt != 1) continue;
        n_unique++;

        const char *disc = strrchr(discs[which].disc_path, '/');
        disc = disc ? disc + 1 : discs[which].disc_path;
        printf("  %-50s  %8s  [disc %d: %s]\n",
               s->path, human_size(s->size[which]), which, disc);
    }
    printf("Total: %d files\n\n", n_unique);

    /* Extension frequency */
    printf("=== File type frequency (all discs combined) ===\n");
    struct { char ext[16]; int count; uint64_t total_size; } exts[64];
    int nexts = 0;

    for (int d = 0; d < ndiscs; d++) {
        for (int f = 0; f < discs[d].nfiles; f++) {
            const char *e = ext_of(discs[d].files[f].path);
            char eupper[16];
            int el = strlen(e) < 15 ? strlen(e) : 15;
            for (int k = 0; k < el; k++)
                eupper[k] = e[k] >= 'a' && e[k] <= 'z' ? e[k] - 32 : e[k];
            eupper[el] = 0;

            int found = 0;
            for (int x = 0; x < nexts; x++) {
                if (strcmp(exts[x].ext, eupper) == 0) {
                    exts[x].count++;
                    exts[x].total_size += discs[d].files[f].size;
                    found = 1;
                    break;
                }
            }
            if (!found && nexts < 64) {
                snprintf(exts[nexts].ext, 16, "%s", eupper);
                exts[nexts].count = 1;
                exts[nexts].total_size = discs[d].files[f].size;
                nexts++;
            }
        }
    }

    /* Sort by count descending */
    for (int a = 0; a < nexts - 1; a++)
        for (int b = a + 1; b < nexts; b++)
            if (exts[b].count > exts[a].count) {
                typeof(exts[0]) tmp = exts[a]; exts[a] = exts[b]; exts[b] = tmp;
            }

    for (int x = 0; x < nexts; x++) {
        uint32_t avg = exts[x].count > 0
                       ? (uint32_t)(exts[x].total_size / exts[x].count)
                       : 0;
        printf("  .%-6s  %4d files  avg %8s\n",
               exts[x].ext, exts[x].count, human_size(avg));
    }
    printf("\n");

    /* Summary stats */
    printf("=== Summary ===\n");
    printf("  Discs compared    : %d\n", ndiscs);
    printf("  Unique paths      : %d\n", nsummary);
    printf("  Shared (all discs): %d\n", n_all);
    printf("  Fixed LBA files   : %d  (likely hardcoded by engine)\n", n_lba_fixed);
    printf("  Size differs      : %d\n", n_size_diff);
    printf("  Unique to 1 disc  : %d\n", n_unique);

    free(discs);
    return 0;
}
