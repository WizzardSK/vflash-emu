#pragma once
#include <stdint.h>

/* V.Flash .ptx raw picture format
 * Based on analysis of V.Flash discs
 * Likely: raw RGB565 or indexed bitmap
 * Resolution: typically 320x240 or smaller sprites
 */

typedef struct {
    uint32_t  width;
    uint32_t  height;
    uint32_t  format;   /* Detected pixel format */
    uint32_t *pixels;   /* RGBA output */
} PTXImage;

#define PTX_FMT_UNKNOWN  0
#define PTX_FMT_RGB565   1
#define PTX_FMT_RGB555   2
#define PTX_FMT_RGBA8888 3
#define PTX_FMT_INDEXED  4

PTXImage* ptx_decode(const uint8_t *data, uint32_t size);
void      ptx_destroy(PTXImage *img);

/* Try to auto-detect format from data patterns */
int ptx_probe(const uint8_t *data, uint32_t size,
              uint32_t *out_w, uint32_t *out_h, uint32_t *out_fmt);
