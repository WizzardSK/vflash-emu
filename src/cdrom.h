#pragma once
#include <stdint.h>
#include <stdio.h>

/* V.Flash CD-ROM subsystem
 * ISO 9660 filesystem, no copy protection
 */

#define CDROM_SECTOR_SIZE  2048
#define CDROM_RAW_SIZE     2352

struct CDROM {
    FILE    *fp;
    char     path[512];
    uint32_t sector_count;
    uint32_t raw_sector_size; /* 2048 for ISO, 2352 for BIN */
    uint32_t data_offset;     /* 0 for ISO, 16 for BIN (skip sync+header) */
    int      is_open;
};
typedef struct CDROM CDROM;

typedef struct {
    char     name[256];   /* Full ISO 9660 filename (no version suffix) */
    uint32_t lba;
    uint32_t size;
    int      is_dir;
} CDEntry;

CDROM*   cdrom_create(void);
void     cdrom_destroy(CDROM *cd);
int      cdrom_open(CDROM *cd, const char *path);
void     cdrom_close(CDROM *cd);
int      cdrom_read_sector(CDROM *cd, uint32_t lba, uint8_t *buf);

/* Find file by path (supports subdirs: "GAME/DATA/BOOT.BIN") */
int      cdrom_find_file(CDROM *cd, const char *path, CDEntry *entry);

/* Find bare filename anywhere on disc (depth-first recursive search) */
int      cdrom_find_file_any(CDROM *cd, const char *name, CDEntry *entry);

/* List directory entries (non-recursive) */
int      cdrom_list_dir(CDROM *cd, uint32_t dir_lba, uint32_t dir_size,
                        CDEntry *entries, int maxentries);

/* Read file data into buffer */
int      cdrom_read_file(CDROM *cd, const CDEntry *entry,
                         uint8_t *buf, uint32_t offset, uint32_t size);
