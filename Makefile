CC = gcc
FOBOS_SRC ?= ../libfobos-sdr-agile
FOBOS_BUILD ?= $(FOBOS_SRC)/build-local
FOBOS_LOCAL ?= ../local-agile

CFLAGS ?= -O2
WARN_CFLAGS ?= -Wall -Wextra -Wformat-security
CPPFLAGS ?=
LDFLAGS ?=
LDLIBS ?= -lfobos_sdr -lusb-1.0 -lm -pthread
INCS = -I$(FOBOS_SRC) -I$(FOBOS_SRC)/fobos -I$(FOBOS_LOCAL)/include
LIBDIRS = -L$(FOBOS_BUILD) -L$(FOBOS_LOCAL)/lib
THREAD_CFLAGS = -pthread
STD_CFLAGS = -std=c99

TARGETS = fobos-scanner tools/fobos-stream-test tools/fobos-fq-response

.PHONY: all clean run stream-test fq-response check

all: $(TARGETS)

fobos-scanner: main.c
	$(CC) $(STD_CFLAGS) $(WARN_CFLAGS) $(CFLAGS) $(THREAD_CFLAGS) $(CPPFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBDIRS) $(LDLIBS)

tools/fobos-stream-test: tools/fobos_stream_test.c
	$(CC) $(STD_CFLAGS) $(WARN_CFLAGS) $(CFLAGS) $(THREAD_CFLAGS) $(CPPFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBDIRS) $(LDLIBS)

tools/fobos-fq-response: tools/fobos_freq_response.c
	$(CC) $(STD_CFLAGS) $(WARN_CFLAGS) $(CFLAGS) $(THREAD_CFLAGS) $(CPPFLAGS) $(INCS) -o $@ $< $(LDFLAGS) $(LIBDIRS) $(LDLIBS)

run: fobos-scanner
	LD_LIBRARY_PATH=$(FOBOS_BUILD):$(FOBOS_LOCAL)/lib$${LD_LIBRARY_PATH:+:$$LD_LIBRARY_PATH} ./fobos-scanner

stream-test: tools/fobos-stream-test
	LD_LIBRARY_PATH=$(FOBOS_BUILD):$(FOBOS_LOCAL)/lib$${LD_LIBRARY_PATH:+:$$LD_LIBRARY_PATH} ./tools/fobos-stream-test

fq-response: tools/fobos-fq-response
	LD_LIBRARY_PATH=$(FOBOS_BUILD):$(FOBOS_LOCAL)/lib$${LD_LIBRARY_PATH:+:$$LD_LIBRARY_PATH} ./tools/fobos-fq-response

check: all
	@echo "Build checks passed. Run tools/http_smoke_test.sh against a running backend for HTTP/API checks."

clean:
	rm -f $(TARGETS)
