CC      = gcc
CFLAGS  = -O2 -Wall -Wextra -Iinclude $(shell sdl2-config --cflags)
LDFLAGS = $(shell sdl2-config --libs) -ljpeg -lm

SRCS = src/main.c src/vflash.c src/arm9.c src/cp15.c src/cdrom.c \
       src/mjp.c src/audio.c src/ptx.c src/ztimer.c src/disasm.c \
       src/debugger.c
OBJS = $(SRCS:.c=.o)
BIN  = vflash

TOOLS = disc_analyze mjp_extract ptx_extract disc_compare testrom_gen

all: $(BIN) $(TOOLS)

$(BIN): $(OBJS)
	$(CC) $(OBJS) -o $(BIN) $(LDFLAGS)

testrom_gen: testrom_gen.c
	$(CC) -O2 -Wall testrom_gen.c -o testrom_gen

testrom.bin: testrom_gen
	./testrom_gen

disc_analyze: src/disc_analyze.c
	$(CC) -O2 -Wall src/disc_analyze.c -o disc_analyze

mjp_extract: src/mjp_extract.c
	$(CC) -O2 -Wall src/mjp_extract.c -o mjp_extract

ptx_extract: src/ptx_extract.c src/ptx.o
	$(CC) -O2 -Wall $(CFLAGS) src/ptx_extract.c src/ptx.o -o ptx_extract

disc_compare: src/disc_compare.c
	$(CC) -O2 -Wall src/disc_compare.c -o disc_compare

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(BIN) $(TOOLS) testrom.bin

.PHONY: all clean

