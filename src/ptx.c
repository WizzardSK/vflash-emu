#include "ptx.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ---- Format probe heuristics ---- */

/* Check if data looks like RGB565:
 * Typical images have smooth color gradients,
 * so adjacent pixels should be similar */
static float rgb565_score(const uint8_t *data, uint32_t npix) {
    if (npix < 16) return 0.0f;
    const uint16_t *px = (const uint16_t*)data;
    uint32_t smooth = 0;
    for (uint32_t i = 1; i < npix && i < 1000; i++) {
        int dr = ((px[i]>>11)&0x1F) - ((px[i-1]>>11)&0x1F);
        int dg = ((px[i]>>5)&0x3F)  - ((px[i-1]>>5)&0x3F);
        int db = (px[i]&0x1F)        - (px[i-1]&0x1F);
        if (abs(dr)<4 && abs(dg)<6 && abs(db)<4) smooth++;
    }
    return (float)smooth / (npix < 1000 ? npix-1 : 999);
}

/* Try common resolutions for a given pixel count */
static const uint32_t common_widths[] = {320, 256, 240, 160, 128, 80, 64, 0};

int ptx_probe(const uint8_t *data, uint32_t size,
              uint32_t *out_w, uint32_t *out_h, uint32_t *out_fmt) {
    if (!data || size < 4) return 0;

    /* Check for a simple header: width(2) height(2) pixels... */
    uint16_t hdr_w = *(uint16_t*)(data);
    uint16_t hdr_h = *(uint16_t*)(data+2);

    if (hdr_w > 0 && hdr_w <= 640 && hdr_h > 0 && hdr_h <= 480) {
        uint32_t npix16 = (size - 4) / 2;
        uint32_t npix32 = (size - 4) / 4;
        if (npix16 == (uint32_t)hdr_w * hdr_h) {
            float score = rgb565_score(data + 4, npix16);
            if (score > 0.5f) {
                *out_w = hdr_w; *out_h = hdr_h; *out_fmt = PTX_FMT_RGB565;
                printf("[PTX] Detected: %ux%u RGB565 (header, score=%.2f)\n", hdr_w, hdr_h, score);
                return 1;
            }
        }
        if (npix32 == (uint32_t)hdr_w * hdr_h) {
            *out_w = hdr_w; *out_h = hdr_h; *out_fmt = PTX_FMT_RGBA8888;
            printf("[PTX] Detected: %ux%u RGBA8888 (header)\n", hdr_w, hdr_h);
            return 1;
        }
    }

    /* No header — try to guess from size */
    for (int wi = 0; common_widths[wi]; wi++) {
        uint32_t w = common_widths[wi];
        /* RGB565: size = w * h * 2 */
        if (size % (w * 2) == 0) {
            uint32_t h = size / (w * 2);
            if (h > 0 && h <= 480) {
                float score = rgb565_score(data, size / 2);
                if (score > 0.6f) {
                    *out_w = w; *out_h = h; *out_fmt = PTX_FMT_RGB565;
                    printf("[PTX] Detected: %ux%u RGB565 (size heuristic, score=%.2f)\n", w, h, score);
                    return 1;
                }
            }
        }
        /* RGB555: same size as RGB565, try if 565 score is low */
        if (size % (w * 2) == 0) {
            uint32_t h = size / (w * 2);
            if (h > 0 && h <= 480) {
                *out_w = w; *out_h = h; *out_fmt = PTX_FMT_RGB555;
                printf("[PTX] Guessing: %ux%u RGB555\n", w, h);
                return 1;
            }
        }
    }

    /* Fallback: treat as 320x240 RGB565 if size is close */
    if (size >= 320*240*2) {
        *out_w = 320; *out_h = 240; *out_fmt = PTX_FMT_RGB565;
        printf("[PTX] Fallback: 320x240 RGB565\n");
        return 1;
    }

    fprintf(stderr, "[PTX] Cannot detect format for %u bytes\n", size);
    return 0;
}

static inline uint32_t rgb565_to_rgba(uint16_t px) {
    uint8_t r = ((px >> 11) & 0x1F) << 3; r |= r >> 5;
    uint8_t g = ((px >>  5) & 0x3F) << 2; g |= g >> 6;
    uint8_t b = ( px        & 0x1F) << 3; b |= b >> 5;
    return 0xFF000000 | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

static inline uint32_t rgb555_to_rgba(uint16_t px) {
    uint8_t r = ((px >> 10) & 0x1F) << 3; r |= r >> 5;
    uint8_t g = ((px >>  5) & 0x1F) << 3; g |= g >> 5;
    uint8_t b = ( px        & 0x1F) << 3; b |= b >> 5;
    uint8_t a = (px >> 15) ? 0xFF : 0x00;  /* bit15 = alpha in RGB1555 */
    return ((uint32_t)a << 24) | ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

PTXImage* ptx_decode(const uint8_t *data, uint32_t size) {
    uint32_t w=0, h=0, fmt=0;
    if (!ptx_probe(data, size, &w, &h, &fmt)) return NULL;

    PTXImage *img = calloc(1, sizeof(PTXImage));
    img->width  = w;
    img->height = h;
    img->format = fmt;
    img->pixels = malloc(w * h * sizeof(uint32_t));

    const uint8_t *src = data;
    /* Skip header if detected with header */
    uint16_t hdr_w = *(uint16_t*)data, hdr_h = *(uint16_t*)(data+2);
    if (hdr_w == w && hdr_h == h) src += 4;

    switch (fmt) {
        case PTX_FMT_RGB565: {
            const uint16_t *px = (const uint16_t*)src;
            for (uint32_t i = 0; i < w * h; i++)
                img->pixels[i] = rgb565_to_rgba(px[i]);
            break;
        }
        case PTX_FMT_RGB555: {
            const uint16_t *px = (const uint16_t*)src;
            for (uint32_t i = 0; i < w * h; i++)
                img->pixels[i] = rgb555_to_rgba(px[i]);
            break;
        }
        case PTX_FMT_RGBA8888:
            memcpy(img->pixels, src, w * h * 4);
            break;
        default:
            memset(img->pixels, 0x80, w * h * 4);
    }

    printf("[PTX] Decoded %ux%u image\n", w, h);
    return img;
}

void ptx_destroy(PTXImage *img) {
    if (img) { free(img->pixels); free(img); }
}
