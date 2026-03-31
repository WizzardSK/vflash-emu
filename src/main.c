#include "vflash.h"
#include "debugger.h"
#include <SDL2/SDL.h>
#include <stdio.h>
#include <string.h>

static void print_usage(const char *prog) {
    fprintf(stderr,
        "V.Flash Emulator\n"
        "Usage: %s [options] <disc.iso>\n\n"
        "Options:\n"
        "  --debug      Enable I/O trace and register dumps\n"
        "  --dbg        Start interactive debugger (paused at boot)\n"
        "  --dbg-run    Start interactive debugger (running)\n"
        "  --headless   Run without display\n"
        "  --scale N    Window scale factor (default: 2)\n"
        "  --help       Show this help\n\n"
        "Controls:\n"
        "  Arrow keys   D-Pad\n"
        "  Z / X / C / V  Red/Yellow/Green/Blue\n"
        "  Enter        Enter/OK\n"
        "  F1           Toggle debug trace\n"
        "  F2           Pause/resume debugger\n"
        "  F11          Toggle fullscreen\n"
        "  Esc          Quit\n\n"
        "Debugger commands (on stdin):\n"
        "  s [N]        step N instructions\n"
        "  c            continue\n"
        "  n            step over\n"
        "  b <addr>     breakpoint\n"
        "  bc <addr>    clear breakpoint\n"
        "  bl           list breakpoints\n"
        "  r            registers\n"
        "  m <addr> [N] memory dump\n"
        "  d <addr> [N] disassemble\n"
        "  pc           current instruction\n"
        "  bt           stack dump\n"
        "  setreg r0=1  write register\n"
        "  q            quit\n",
        prog);
}

int main(int argc, char **argv) {
    const char *disc_path = NULL;
    int debug    = 0;
    int headless = 0;
    int scale    = 2;
    int dbg_mode = 0;  /* 0=off, 1=paused at boot, 2=running with debugger */

    /* Parse arguments */
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--debug") == 0)        debug    = 1;
        else if (strcmp(argv[i], "--headless") == 0) headless = 1;
        else if (strcmp(argv[i], "--dbg") == 0)      dbg_mode = 1;
        else if (strcmp(argv[i], "--dbg-run") == 0)  dbg_mode = 2;
        else if (strcmp(argv[i], "--help") == 0)    { print_usage(argv[0]); return 0; }
        else if (strcmp(argv[i], "--scale") == 0 && i+1 < argc) {
            scale = atoi(argv[++i]);
            if (scale < 1 || scale > 4) scale = 2;
        }
        else if (argv[i][0] != '-') disc_path = argv[i];
    }

    if (!disc_path) {
        print_usage(argv[0]);
        return 1;
    }

    /* Create emulator */
    VFlash *vf = vflash_create(disc_path);
    if (!vf) return 1;

    if (debug) vflash_set_debug(vf, 1);

    /* Init debugger if requested */
    if (dbg_mode) dbg_init(vf, dbg_mode == 1);

    /* Init SDL */
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        fprintf(stderr, "[SDL] Init failed: %s\n", SDL_GetError());
        vflash_destroy(vf);
        return 1;
    }

    vflash_init_audio(vf);

    SDL_Window   *win = NULL;
    SDL_Renderer *ren = NULL;
    SDL_Texture  *tex = NULL;
    int fullscreen = 0;

    if (!headless) {
        win = SDL_CreateWindow(
            "V.Flash Emulator",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            VFLASH_SCREEN_W * scale, VFLASH_SCREEN_H * scale,
            SDL_WINDOW_RESIZABLE
        );
        if (!win) {
            fprintf(stderr, "[SDL] Window failed: %s\n", SDL_GetError());
            SDL_Quit(); vflash_destroy(vf); return 1;
        }

        ren = SDL_CreateRenderer(win, -1,
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        if (!ren) ren = SDL_CreateRenderer(win, -1, 0);

        tex = SDL_CreateTexture(ren,
            SDL_PIXELFORMAT_ARGB8888,
            SDL_TEXTUREACCESS_STREAMING,
            VFLASH_SCREEN_W, VFLASH_SCREEN_H);

        SDL_RenderSetLogicalSize(ren, VFLASH_SCREEN_W, VFLASH_SCREEN_H);
        printf("[SDL] Window: %dx%d (scale %d)\n",
               VFLASH_SCREEN_W * scale, VFLASH_SCREEN_H * scale, scale);
    }

    /* Redirect stdout to log file for capture when running in background.
     * Also keeps line-buffered so messages appear in real time. */
    {
        const char *logfile = getenv("VFLASH_LOG");
        if (logfile) {
            FILE *lf = freopen(logfile, "w", stdout);
            if (lf) setlinebuf(lf);
        } else {
            setlinebuf(stdout);
        }
    }

    /* Main loop */
    int       running   = 1;
    SDL_Event ev;
    uint32_t  fps_timer = SDL_GetTicks();
    int       fps_count = 0;
    char      title[64];

    while (running) {
        uint32_t buttons = 0;
        uint32_t frame_start = SDL_GetTicks();

        /* Events */
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT && !headless) {
                printf("[Main] SDL_QUIT received, exiting\n");
                running = 0; break;
            }
            if (ev.type == SDL_KEYDOWN) {
                switch (ev.key.keysym.sym) {
                    case SDLK_ESCAPE: printf("[Main] ESC pressed\n"); running = 0; break;
                    case SDLK_F1:
                        debug = !debug;
                        vflash_set_debug(vf, debug);
                        printf("[Main] Debug %s\n", debug ? "ON" : "OFF");
                        break;
                    case SDLK_F2:
                        if (dbg_mode) {
                            if (dbg_is_running()) dbg_pause(vf);
                            else                  dbg_resume();
                        }
                        break;
                    case SDLK_F5: {
                        /* Save screenshot as BMP */
                        uint32_t *fb = vflash_get_framebuffer(vf);
                        SDL_Surface *surf = SDL_CreateRGBSurfaceFrom(
                            fb, 320, 240, 32, 320*4,
                            0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);
                        if (surf) {
                            SDL_SaveBMP(surf, "screenshot.bmp");
                            SDL_FreeSurface(surf);
                            printf("[Main] Screenshot saved: screenshot.bmp\n");
                        }
                        break;
                    }
                    case SDLK_F11:
                        if (!headless) {
                            fullscreen = !fullscreen;
                            SDL_SetWindowFullscreen(win,
                                fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
                        }
                        break;
                }
            }
        }

        /* Input */
        if (!headless) {
            const uint8_t *keys = SDL_GetKeyboardState(NULL);
            if (keys[SDL_SCANCODE_UP])     buttons |= VFLASH_BTN_UP;
            if (keys[SDL_SCANCODE_DOWN])   buttons |= VFLASH_BTN_DOWN;
            if (keys[SDL_SCANCODE_LEFT])   buttons |= VFLASH_BTN_LEFT;
            if (keys[SDL_SCANCODE_RIGHT])  buttons |= VFLASH_BTN_RIGHT;
            if (keys[SDL_SCANCODE_Z])      buttons |= VFLASH_BTN_RED;
            if (keys[SDL_SCANCODE_X])      buttons |= VFLASH_BTN_YELLOW;
            if (keys[SDL_SCANCODE_C])      buttons |= VFLASH_BTN_GREEN;
            if (keys[SDL_SCANCODE_V])      buttons |= VFLASH_BTN_BLUE;
            if (keys[SDL_SCANCODE_RETURN]) buttons |= VFLASH_BTN_ENTER;
        } else {
            /* Headless auto-input: simulate button presses to progress
             * past title screens and menus.
             * Schedule: Enter at 3s, 5s, 8s; Red at 10s, 15s, 20s;
             * then cycle Enter+Red every 10s. */
            uint32_t sec = SDL_GetTicks() / 1000;
            if (sec >= 3 && sec < 4)  buttons |= VFLASH_BTN_ENTER;
            if (sec >= 5 && sec < 6)  buttons |= VFLASH_BTN_ENTER;
            if (sec >= 8 && sec < 9)  buttons |= VFLASH_BTN_ENTER;
            if (sec >= 10 && sec < 11) buttons |= VFLASH_BTN_RED;
            if (sec >= 15 && sec < 16) buttons |= VFLASH_BTN_RED;
            if (sec >= 20 && sec < 21) buttons |= VFLASH_BTN_ENTER;
            if (sec >= 25 && sec < 26) buttons |= VFLASH_BTN_RED;
            if (sec >= 30 && sec < 31) buttons |= VFLASH_BTN_ENTER;
            if (sec >= 35 && sec < 36) buttons |= VFLASH_BTN_RED;
            if (sec >= 40) {
                /* Cycle Enter/Red every 5 seconds */
                uint32_t phase = (sec - 40) / 5;
                uint32_t in_phase = (sec - 40) % 5;
                if (in_phase == 0)
                    buttons |= (phase % 2) ? VFLASH_BTN_RED : VFLASH_BTN_ENTER;
            }
        }
        vflash_set_input(vf, buttons);

        /* Debugger frame tick (handles commands, stepping, BP check) */
        if (dbg_mode) {
            int dbg_ret = dbg_frame(vf);
            if (dbg_ret == DBG_QUIT) { running = 0; break; }
            /* If paused: don't run emulator, just render last frame */
            if (dbg_ret == DBG_PAUSE) goto render;
        }

        /* Run one emulator frame */
        vflash_run_frame(vf);

        /* Breakpoint check after frame */
        if (dbg_mode && vflash_bp_hit(vf)) {
            fprintf(stderr, "[DBG] *** Breakpoint hit at 0x%08X ***\n",
                    vflash_get_pc(vf));
        }

    render:
        /* Render */
        if (!headless && win) {
            SDL_UpdateTexture(tex, NULL,
                vflash_get_framebuffer(vf),
                VFLASH_SCREEN_W * sizeof(uint32_t));
            SDL_RenderClear(ren);
            SDL_RenderCopy(ren, tex, NULL, NULL);
            SDL_RenderPresent(ren);
        }

        /* FPS */
        fps_count++;
        uint32_t now = SDL_GetTicks();
        if (now - fps_timer >= 1000) {
            const char *dbg_tag = dbg_mode
                ? (dbg_is_running() ? " [DBG:run]" : " [DBG:paused]") : "";
            if (!headless && win) {
                snprintf(title, sizeof(title),
                         "V.Flash Emulator — %d FPS%s%s",
                         fps_count, debug ? " [TRACE]" : "", dbg_tag);
                SDL_SetWindowTitle(win, title);
            } else {
                printf("[Main] %d FPS%s\n", fps_count, dbg_tag);
            }
            fps_count = 0;
            fps_timer = now;
        }

        /* Frame timing — target 60fps (skip delay when debugger paused) */
        if (!dbg_mode || dbg_is_running()) {
            uint32_t elapsed = SDL_GetTicks() - frame_start;
            if (elapsed < 16) SDL_Delay(16 - elapsed);
        } else {
            SDL_Delay(16);  /* Paused: still yield CPU */
        }
    }

    /* Cleanup */
    if (tex) SDL_DestroyTexture(tex);
    if (ren) SDL_DestroyRenderer(ren);
    if (win) SDL_DestroyWindow(win);
    SDL_Quit();
    vflash_destroy(vf);
    printf("[Main] Exited cleanly\n");
    return 0;
}
