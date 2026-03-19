#pragma once
#include <stdint.h>

/* V.Flash video decoder
 * Format: Motion JPEG (.mjp) — each frame is a standard JPEG
 * Resolution: 320x240 estimated
 * Uses libjpeg for decoding
 */

struct MJPDecoder {
    int      width;
    int      height;
    uint32_t *framebuf;  /* RGBA output */
    int      frames_decoded;
};
typedef struct MJPDecoder MJPDecoder;

MJPDecoder* mjp_create(int width, int height);
void        mjp_destroy(MJPDecoder *dec);
int         mjp_decode_frame(MJPDecoder *dec, const uint8_t *data, uint32_t size);
uint32_t*   mjp_get_framebuf(MJPDecoder *dec);
