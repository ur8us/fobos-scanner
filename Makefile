CC = gcc
CFLAGS = -Wall -O2 -std=c99 -pthread
LDFLAGS = -L../libfobos-sdr-agile/build-local -L../local-agile/lib
LIBS = -lfobos_sdr -lusb-1.0 -lm -pthread
INCS = -I../libfobos-sdr-agile -I../libfobos-sdr-agile/fobos -I../local-agile/include

TARGET = fobos-scanner

.PHONY: all clean run

all: $(TARGET)

$(TARGET): main.c
	$(CC) $(CFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBS)

run: $(TARGET)
	LD_LIBRARY_PATH=../libfobos-sdr-agile/build-local:../local-agile/lib ./$(TARGET)

clean:
	rm -f $(TARGET)
