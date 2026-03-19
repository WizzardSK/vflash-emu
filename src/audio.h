#pragma once
#include <stdint.h>

/* V.Flash Audio
 * .snd files are PCM WAV
 * ZEVIO 1020 ZSP400 DSP handles audio mixing
 * Output: 16-bit stereo, estimated 44100Hz
 */

#define AUDIO_SAMPLE_RATE  44100
#define AUDIO_CHANNELS     2
#define AUDIO_BUF_SAMPLES  1024

typedef struct {
    int16_t  *buf;          /* Ring buffer */
    uint32_t  buf_size;     /* In samples */
    uint32_t  write_pos;
    uint32_t  read_pos;
    int       initialized;
    uint32_t  volume;       /* 0-256 */
} Audio;

Audio*  audio_create(void);
void    audio_destroy(Audio *a);
int     audio_init_sdl(Audio *a);
void    audio_push_samples(Audio *a, const int16_t *samples, uint32_t count);
void    audio_set_volume(Audio *a, uint32_t vol);

/* .snd file decoder (PCM WAV) */
int     snd_decode(const uint8_t *data, uint32_t size,
                   int16_t **out_samples, uint32_t *out_count, uint32_t *out_rate);

/* DMA audio queue: raw PCM from RAM (called from I/O write handler)
 * stereo: 1=stereo, 0=mono
 * s16:    1=16-bit signed, 0=8-bit unsigned */
void    audio_queue_pcm(Audio *a, const uint8_t *data, uint32_t size,
                        int stereo, int s16);
