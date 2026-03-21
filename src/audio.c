#include "audio.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

static void audio_callback(void *userdata, uint8_t *stream, int len) {
    Audio *a = userdata;
    int16_t *out = (int16_t*)stream;
    int samples = len / sizeof(int16_t);

    for (int i = 0; i < samples; i++) {
        if (a->read_pos != a->write_pos) {
            int16_t s = a->buf[a->read_pos];
            s = (int16_t)((int32_t)s * a->volume / 256);
            out[i] = s;
            a->read_pos = (a->read_pos + 1) % a->buf_size;
        } else {
            out[i] = 0;
        }
    }
}

Audio* audio_create(void) {
    Audio *a = calloc(1, sizeof(Audio));
    a->buf_size = AUDIO_BUF_SAMPLES * 64; /* large buffer for MJP audio */
    a->buf = calloc(a->buf_size, sizeof(int16_t));
    a->volume = 200;
    return a;
}

void audio_destroy(Audio *a) {
    if (!a) return;
    if (a->initialized) SDL_CloseAudio();
    free(a->buf);
    free(a);
}

int audio_init_sdl(Audio *a) {
    SDL_AudioSpec want = {0}, got;
    want.freq     = AUDIO_SAMPLE_RATE;
    want.format   = AUDIO_S16SYS;
    want.channels = AUDIO_CHANNELS;
    want.samples  = AUDIO_BUF_SAMPLES;
    want.callback = audio_callback;
    want.userdata = a;

    if (SDL_OpenAudio(&want, &got) < 0) {
        fprintf(stderr, "[Audio] SDL_OpenAudio failed: %s\n", SDL_GetError());
        return 0;
    }

    printf("[Audio] Init: %dHz %dch fmt=0x%X buf=%d\n",
           got.freq, got.channels, got.format, got.samples);
    SDL_PauseAudio(0);
    a->initialized = 1;
    return 1;
}

void audio_push_samples(Audio *a, const int16_t *samples, uint32_t count) {
    for (uint32_t i = 0; i < count; i++) {
        uint32_t next = (a->write_pos + 1) % a->buf_size;
        if (next == a->read_pos) break;  /* Buffer full */
        a->buf[a->write_pos] = samples[i];
        a->write_pos = next;
    }
}

void audio_set_volume(Audio *a, uint32_t vol) {
    a->volume = vol > 256 ? 256 : vol;
}

/* ---- .snd / WAV decoder ---- */
int snd_decode(const uint8_t *data, uint32_t size,
               int16_t **out_samples, uint32_t *out_count, uint32_t *out_rate) {
    if (!data || size < 44) return 0;

    /* Check for RIFF WAV header */
    if (data[0]=='R' && data[1]=='I' && data[2]=='F' && data[3]=='F' &&
        data[8]=='W' && data[9]=='A' && data[10]=='V' && data[11]=='E') {

        /* Parse fmt chunk */
        uint32_t off = 12;
        uint32_t data_off = 0, data_size = 0;
        uint16_t channels = 1, bits = 16;
        uint32_t sample_rate = 44100;

        while (off + 8 <= size) {
            uint32_t chunk_id   = *(uint32_t*)(data + off);
            uint32_t chunk_size = *(uint32_t*)(data + off + 4);
            off += 8;
            if (chunk_id == 0x20746D66) { /* "fmt " */
                channels    = *(uint16_t*)(data + off + 2);
                sample_rate = *(uint32_t*)(data + off + 4);
                bits        = *(uint16_t*)(data + off + 14);
            } else if (chunk_id == 0x61746164) { /* "data" */
                data_off  = off;
                data_size = chunk_size;
                break;
            }
            off += chunk_size;
        }

        if (!data_off) return 0;

        uint32_t samples = data_size / (bits / 8);
        *out_samples = malloc(samples * sizeof(int16_t));
        *out_count   = samples;
        *out_rate    = sample_rate;

        if (bits == 16) {
            memcpy(*out_samples, data + data_off, samples * sizeof(int16_t));
        } else if (bits == 8) {
            for (uint32_t i = 0; i < samples; i++)
                (*out_samples)[i] = ((int16_t)data[data_off + i] - 128) * 256;
        }
        (void)channels;
        printf("[Audio] WAV: %uHz %uch %ubit %u samples\n",
               sample_rate, channels, bits, samples);
        return 1;
    }

    /* Raw PCM fallback — assume 16-bit stereo 44100Hz */
    uint32_t samples = size / sizeof(int16_t);
    *out_samples = malloc(size);
    *out_count   = samples;
    *out_rate    = 44100;
    memcpy(*out_samples, data, size);
    printf("[Audio] Raw PCM: %u samples\n", samples);
    return 1;
}

/* ---- IMA ADPCM decoder (V.Flash MJP audio) ---- */
static const int16_t ima_step_table[89] = {
    7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,
    60,66,73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,
    337,371,408,449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,
    1552,1707,1878,2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,
    5358,5894,6484,7132,7845,8630,9493,10442,11487,12635,13899,15289,
    16818,18500,20350,22385,24623,27086,29794,32767
};
static const int8_t ima_index_table[16] = {
    -1,-1,-1,-1, 2,4,6,8, -1,-1,-1,-1, 2,4,6,8
};

void audio_decode_ima_adpcm(Audio *a, const uint8_t *data, uint32_t size) {
    if (!a || !data || size < 4) return;

    /* 4-byte header: predictor(2) + step_index(1) + reserved(1) */
    int16_t predictor = (int16_t)(data[0] | (data[1] << 8));
    int step_idx = data[2];
    if (step_idx > 88) step_idx = 88;

    /* Decode nibbles → 16-bit PCM, push as mono (auto-upmixed to stereo) */
    /* Resample from 22050 to 44100 by duplicating each sample */
    for (uint32_t i = 4; i < size; i++) {
        uint8_t byte = data[i];
        for (int n = 0; n < 2; n++) {
            uint8_t nibble = (n == 0) ? (byte & 0xF) : ((byte >> 4) & 0xF);
            int step = ima_step_table[step_idx];
            int diff = step >> 3;
            if (nibble & 1) diff += step >> 2;
            if (nibble & 2) diff += step >> 1;
            if (nibble & 4) diff += step;
            if (nibble & 8) diff = -diff;
            predictor = (int16_t)(predictor + diff);
            if (predictor > 32767) predictor = 32767;
            if (predictor < -32768) predictor = -32768;
            step_idx += ima_index_table[nibble];
            if (step_idx < 0) step_idx = 0;
            if (step_idx > 88) step_idx = 88;
            /* Push sample twice (22050→44100 upsample) as stereo (L+R) */
            int16_t stereo[4] = {predictor, predictor, predictor, predictor};
            audio_push_samples(a, stereo, 4);
        }
    }
}

/* Queue raw PCM from RAM (called from DMA write handler) */
void audio_queue_pcm(Audio *a, const uint8_t *data, uint32_t size,
                     int stereo, int s16) {
    if (!a || !data || size == 0) return;

    int16_t *tmp  = NULL;
    uint32_t count = 0;

    if (s16) {
        count = size / sizeof(int16_t);
        tmp = malloc(count * sizeof(int16_t));
        if (!tmp) return;
        memcpy(tmp, data, count * sizeof(int16_t));
    } else {
        /* 8-bit unsigned → 16-bit signed */
        count = size;
        tmp = malloc(count * sizeof(int16_t));
        if (!tmp) return;
        for (uint32_t i = 0; i < count; i++)
            tmp[i] = ((int16_t)data[i] - 128) * 256;
    }

    /* Upmix mono → stereo if needed */
    if (!stereo) {
        int16_t *stereo_buf = malloc(count * 2 * sizeof(int16_t));
        if (stereo_buf) {
            for (uint32_t i = 0; i < count; i++) {
                stereo_buf[i * 2]     = tmp[i];
                stereo_buf[i * 2 + 1] = tmp[i];
            }
            free(tmp);
            tmp   = stereo_buf;
            count *= 2;
        }
    }

    audio_push_samples(a, tmp, count);
    free(tmp);
}
