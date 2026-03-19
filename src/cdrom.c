#include "cdrom.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

CDROM* cdrom_create(void)  { return calloc(1, sizeof(CDROM)); }

void cdrom_destroy(CDROM *cd) {
    if (cd) { cdrom_close(cd); free(cd); }
}

/* If path is a .cue file, extract the .bin filename from it */
static int resolve_cue(const char *path, char *binpath, size_t binpath_size) {
    size_t len = strlen(path);
    if (len < 4) return 0;
    const char *ext = path + len - 4;
    if (strcasecmp(ext, ".cue") != 0) return 0;

    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char line[1024];
    while (fgets(line, sizeof(line), f)) {
        /* Look for: FILE "something.bin" BINARY */
        char *q1 = strchr(line, '"');
        if (!q1) continue;
        char *q2 = strchr(q1 + 1, '"');
        if (!q2) continue;
        *q2 = 0;
        /* Build path relative to cue file's directory */
        const char *slash = strrchr(path, '/');
        if (slash) {
            size_t dirlen = (size_t)(slash - path + 1);
            if (dirlen + strlen(q1 + 1) >= binpath_size) { fclose(f); return 0; }
            memcpy(binpath, path, dirlen);
            strcpy(binpath + dirlen, q1 + 1);
        } else {
            strncpy(binpath, q1 + 1, binpath_size - 1);
            binpath[binpath_size - 1] = 0;
        }
        fclose(f);
        return 1;
    }
    fclose(f);
    return 0;
}

int cdrom_open(CDROM *cd, const char *path) {
    /* Handle .cue files by extracting the .bin path */
    char resolved[1024];
    if (resolve_cue(path, resolved, sizeof(resolved))) {
        printf("[CDROM] CUE -> BIN: %s\n", resolved);
        path = resolved;
    }

    cd->fp = fopen(path, "rb");
    if (!cd->fp) { fprintf(stderr, "[CDROM] Cannot open: %s\n", path); return 0; }
    strncpy(cd->path, path, sizeof(cd->path) - 1);
    fseek(cd->fp, 0, SEEK_END);
    long file_size = ftell(cd->fp);

    /* Auto-detect sector size: if file size is divisible by 2352 but not 2048,
       or if we find a CD sync pattern at offset 0, it's raw BIN */
    uint8_t sync[12];
    fseek(cd->fp, 0, SEEK_SET);
    fread(sync, 1, 12, cd->fp);
    static const uint8_t cd_sync[12] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00};

    if (memcmp(sync, cd_sync, 12) == 0 || (file_size % CDROM_RAW_SIZE == 0 && file_size % CDROM_SECTOR_SIZE != 0)) {
        cd->raw_sector_size = CDROM_RAW_SIZE;
        cd->data_offset = 16; /* skip 12 sync + 4 header */
        cd->sector_count = (uint32_t)(file_size / CDROM_RAW_SIZE);
        printf("[CDROM] Opened BIN (raw 2352): %s (%u sectors)\n", path, cd->sector_count);
    } else {
        cd->raw_sector_size = CDROM_SECTOR_SIZE;
        cd->data_offset = 0;
        cd->sector_count = (uint32_t)(file_size / CDROM_SECTOR_SIZE);
        printf("[CDROM] Opened ISO (2048): %s (%u sectors)\n", path, cd->sector_count);
    }
    cd->is_open = 1;
    return 1;
}

void cdrom_close(CDROM *cd) {
    if (cd->fp) { fclose(cd->fp); cd->fp = NULL; }
    cd->is_open = 0;
}

int cdrom_read_sector(CDROM *cd, uint32_t lba, uint8_t *buf) {
    if (!cd->is_open || lba >= cd->sector_count) return 0;
    fseek(cd->fp, (long)lba * cd->raw_sector_size + cd->data_offset, SEEK_SET);
    return fread(buf, 1, CDROM_SECTOR_SIZE, cd->fp) == CDROM_SECTOR_SIZE;
}

/* ---- ISO 9660 path parsing ---- */

/* Parse a single directory entry at sector+off into CDEntry.
 * Returns record length (0 = end of sector). */
static int parse_de(const uint8_t *sector, uint32_t off, CDEntry *e) {
    uint8_t rl = sector[off];
    if (rl == 0) return 0;

    uint8_t nl = sector[off + 32];
    int n = nl < (int)sizeof(e->name) - 1 ? nl : (int)sizeof(e->name) - 1;
    memcpy(e->name, sector + off + 33, n);
    e->name[n] = 0;
    char *semi = strchr(e->name, ';');
    if (semi) *semi = 0;

    e->lba    = *(uint32_t*)(sector + off + 2);
    e->size   = *(uint32_t*)(sector + off + 10);
    e->is_dir = (sector[off + 25] & 0x02) != 0;
    return rl;
}

/* Search a directory for a NAME component (single path element, no slashes).
 * Returns 1 and fills *out if found. */
static int dir_find_name(CDROM *cd, uint32_t dir_lba, uint32_t dir_size,
                          const char *name, CDEntry *out) {
    uint8_t sector[CDROM_SECTOR_SIZE];
    uint32_t remaining = dir_size;
    uint32_t cur_lba   = dir_lba;

    while (remaining > 0) {
        if (!cdrom_read_sector(cd, cur_lba, sector)) break;
        uint32_t off = 0;
        while (off < CDROM_SECTOR_SIZE) {
            CDEntry e;
            int rl = parse_de(sector, off, &e);
            if (!rl) break;
            if (e.name[0] && strcasecmp(e.name, name) == 0) {
                *out = e;
                return 1;
            }
            off += (uint32_t)rl;
        }
        cur_lba++;
        remaining -= remaining > CDROM_SECTOR_SIZE ? CDROM_SECTOR_SIZE : remaining;
    }
    return 0;
}

/* Recursively walk path components split by '/'.
 * path = "GAME/DATA/BOOT.BIN" or just "BOOT.BIN" */
int cdrom_find_file(CDROM *cd, const char *path, CDEntry *entry) {
    uint8_t pvd[CDROM_SECTOR_SIZE];
    if (!cdrom_read_sector(cd, 16, pvd) || pvd[0] != 1) return 0;

    uint32_t dir_lba  = *(uint32_t*)(pvd + 156 + 2);
    uint32_t dir_size = *(uint32_t*)(pvd + 156 + 10);

    /* Split path on '/' and walk component by component */
    char buf[512];
    strncpy(buf, path, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char *saveptr = NULL;
    char *tok = strtok_r(buf, "/\\", &saveptr);
    CDEntry cur = {0};

    while (tok) {
        if (!dir_find_name(cd, dir_lba, dir_size, tok, &cur)) {
            /* Not found in this directory — also try scanning all subdirs
             * one level deep as a fallback for flat path searches */
            return 0;
        }
        char *next = strtok_r(NULL, "/\\", &saveptr);
        if (!next) {
            /* Found the target */
            *entry = cur;
            printf("[CDROM] Found: %s  LBA=%u size=%u\n",
                   path, entry->lba, entry->size);
            return 1;
        }
        if (!cur.is_dir) return 0;  /* Intermediate component is not a directory */
        dir_lba  = cur.lba;
        dir_size = cur.size;
        tok      = next;
    }
    return 0;
}

/* Recursive disc-wide search for a bare filename (no path prefix).
 * Searches all directories depth-first. */
static int find_recursive(CDROM *cd, uint32_t dir_lba, uint32_t dir_size,
                           const char *name, CDEntry *out, int depth) {
    if (depth > 8) return 0;  /* Safety limit */

    uint8_t sector[CDROM_SECTOR_SIZE];
    uint32_t remaining = dir_size;
    uint32_t cur_lba   = dir_lba;

    /* Collect subdirs to recurse into after checking files */
    CDEntry subdirs[64];
    int nsubs = 0;

    while (remaining > 0) {
        if (!cdrom_read_sector(cd, cur_lba, sector)) break;
        uint32_t off = 0;
        while (off < CDROM_SECTOR_SIZE) {
            CDEntry e;
            int rl = parse_de(sector, off, &e);
            if (!rl) break;
            if (!e.name[0] || strcmp(e.name, ".") == 0 || strcmp(e.name, "..") == 0) {
                off += (uint32_t)rl; continue;
            }
            if (!e.is_dir && strcasecmp(e.name, name) == 0) {
                *out = e;
                return 1;
            }
            if (e.is_dir && nsubs < 64)
                subdirs[nsubs++] = e;
            off += (uint32_t)rl;
        }
        cur_lba++;
        remaining -= remaining > CDROM_SECTOR_SIZE ? CDROM_SECTOR_SIZE : remaining;
    }

    /* Recurse into subdirectories */
    for (int i = 0; i < nsubs; i++) {
        if (find_recursive(cd, subdirs[i].lba, subdirs[i].size, name, out, depth + 1))
            return 1;
    }
    return 0;
}

/* Find a bare filename anywhere on the disc (depth-first search) */
int cdrom_find_file_any(CDROM *cd, const char *name, CDEntry *entry) {
    uint8_t pvd[CDROM_SECTOR_SIZE];
    if (!cdrom_read_sector(cd, 16, pvd) || pvd[0] != 1) return 0;

    uint32_t root_lba  = *(uint32_t*)(pvd + 156 + 2);
    uint32_t root_size = *(uint32_t*)(pvd + 156 + 10);

    if (find_recursive(cd, root_lba, root_size, name, entry, 0)) {
        printf("[CDROM] Found (any): %s  LBA=%u size=%u\n",
               name, entry->lba, entry->size);
        return 1;
    }
    return 0;
}

/* List all files in a directory (non-recursive) */
int cdrom_list_dir(CDROM *cd, uint32_t dir_lba, uint32_t dir_size,
                   CDEntry *entries, int maxentries) {
    uint8_t sector[CDROM_SECTOR_SIZE];
    uint32_t remaining = dir_size;
    uint32_t cur_lba   = dir_lba;
    int count = 0;

    while (remaining > 0 && count < maxentries) {
        if (!cdrom_read_sector(cd, cur_lba, sector)) break;
        uint32_t off = 0;
        while (off < CDROM_SECTOR_SIZE && count < maxentries) {
            CDEntry e;
            int rl = parse_de(sector, off, &e);
            if (!rl) break;
            if (e.name[0] && strcmp(e.name, ".") && strcmp(e.name, ".."))
                entries[count++] = e;
            off += (uint32_t)rl;
        }
        cur_lba++;
        remaining -= remaining > CDROM_SECTOR_SIZE ? CDROM_SECTOR_SIZE : remaining;
    }
    return count;
}

int cdrom_read_file(CDROM *cd, const CDEntry *entry,
                    uint8_t *buf, uint32_t offset, uint32_t size) {
    uint32_t lba  = entry->lba + offset / CDROM_SECTOR_SIZE;
    uint32_t skip = offset % CDROM_SECTOR_SIZE;
    uint32_t read = 0;
    uint8_t  sector[CDROM_SECTOR_SIZE];

    while (read < size) {
        if (!cdrom_read_sector(cd, lba, sector)) break;
        uint32_t chunk = CDROM_SECTOR_SIZE - skip;
        if (chunk > size - read) chunk = size - read;
        memcpy(buf + read, sector + skip, chunk);
        read += chunk;
        skip  = 0;
        lba++;
    }
    return (int)read;
}
