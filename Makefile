CC = gcc
CFLAGS = -Wall -O2 -std=c99 -pthread
LDFLAGS = -L../libfobos-sdr-agile/build-local -L../local-agile/lib
LIBS = -lfobos_sdr -lusb-1.0 -lm -pthread
INCS = -I../libfobos-sdr-agile -I../libfobos-sdr-agile/fobos -I../local-agile/include

TARGETS = fobos-scanner fobos-stream-test

.PHONY: all clean run stream-test

all: $(TARGETS)

fobos-scanner: main.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBS)

fobos-stream-test: fobos_stream_test.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBS)

run: fobos-scanner
	LD_LIBRARY_PATH=../libfobos-sdr-agile/build-local:../local-agile/lib ./fobos-scanner

stream-test: fobos-stream-test
	LD_LIBRARY_PATH=../libfobos-sdr-agile/build-local:../local-agile/lib ./fobos-stream-test

clean:
	rm -f $(TARGETS)
