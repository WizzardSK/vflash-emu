#include "mjp.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <jpeglib.h>
#include <setjmp.h>

typedef struct {
    struct jpeg_error_mgr pub;
    jmp_buf setjmp_buf;
} MJPErrorMgr;

static void mjp_error_exit(j_common_ptr cinfo) {
    MJPErrorMgr *err = (MJPErrorMgr*)cinfo->err;
    longjmp(err->setjmp_buf, 1);
}

MJPDecoder* mjp_create(int width, int height) {
    MJPDecoder *dec = calloc(1, sizeof(MJPDecoder));
    dec->width    = width;
    dec->height   = height;
    dec->framebuf = calloc(width * height, sizeof(uint32_t));
    return dec;
}

void mjp_destroy(MJPDecoder *dec) {
    if (dec) { free(dec->framebuf); free(dec); }
}

static void mjp_output_message(j_common_ptr cinfo) {
    (void)cinfo; /* suppress libjpeg warnings */
}

int mjp_decode_frame(MJPDecoder *dec, const uint8_t *data, uint32_t size) {
    struct jpeg_decompress_struct cinfo;
    MJPErrorMgr jerr;
    int decoded_lines = 0;

    /* Decode into temporary buffer — only copy to framebuf on success.
     * This prevents flickering (black frames) on decode errors. */
    uint32_t *tmpbuf = malloc(dec->width * dec->height * sizeof(uint32_t));
    if (!tmpbuf) return 0;
    memset(tmpbuf, 0, dec->width * dec->height * sizeof(uint32_t));

    cinfo.err = jpeg_std_error(&jerr.pub);
    jerr.pub.error_exit = mjp_error_exit;
    jerr.pub.output_message = mjp_output_message;
    if (setjmp(jerr.setjmp_buf)) {
        jpeg_destroy_decompress(&cinfo);
        if (decoded_lines > dec->height / 2) {
            /* Got most of the frame — use it */
            memcpy(dec->framebuf, tmpbuf, dec->width * dec->height * sizeof(uint32_t));
            free(tmpbuf);
            dec->frames_decoded++;
            return 1;
        }
        /* Too few lines — keep previous frame (no flicker) */
        free(tmpbuf);
        return 0;
    }

    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, (unsigned char*)data, size);
    jpeg_read_header(&cinfo, TRUE);
    cinfo.out_color_space = JCS_RGB;
    jpeg_start_decompress(&cinfo);

    int w = cinfo.output_width;
    int h = cinfo.output_height;
    dec->decoded_w = w;
    dec->decoded_h = h;
    uint8_t *row = malloc(w * 3);

    while ((int)cinfo.output_scanline < h) {
        int y = cinfo.output_scanline;
        jpeg_read_scanlines(&cinfo, &row, 1);
        decoded_lines++;
        if (y < dec->height) {
            for (int x = 0; x < w && x < dec->width; x++) {
                uint8_t r = row[x*3+0];
                uint8_t g = row[x*3+1];
                uint8_t b = row[x*3+2];
                tmpbuf[y * dec->width + x] = 0xFF000000 | (r<<16) | (g<<8) | b;
            }
        }
    }

    free(row);
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);

    /* Full decode success — swap to framebuf */
    memcpy(dec->framebuf, tmpbuf, dec->width * dec->height * sizeof(uint32_t));
    free(tmpbuf);
    dec->frames_decoded++;
    return 1;
}

uint32_t* mjp_get_framebuf(MJPDecoder *dec) {
    return dec->framebuf;
}
