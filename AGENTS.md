# Agent Notes

## Project Shape

`fobos-scanner` is a small C backend plus a single-page HTML frontend for RigExpert Fobos SDR.

- `main.c` starts an HTTP server on port `8080`, serves `index.html`, exposes JSON API endpoints, and streams scan lines to the browser over Server-Sent Events.
- `index.html` owns the frontend controls, spectrum canvas, waterfall canvas, hover readout, and scan parameter requests.
- `Makefile` builds one binary, `fobos-scanner`, against the agile Fobos SDR library and `libusb`.
- `run.sh` starts the binary with the local agile library paths in `LD_LIBRARY_PATH`.
- `fobos-scanner.conf` is a local runtime config file and should stay untracked.
- `scanner-deepseek` is reference material and should stay untracked.

## Scanner Architecture

The backend uses the agile Fobos SDR hardware scan API:

1. Build an air-frequency scan band from start/end frequency and `step = samplerate * bw_ratio`.
2. Cap the hardware scan table to `MAX_FREQS` (`256`) because the library scan list is limited.
3. Convert each air-frequency center to a receiver frequency before calling `fobos_sdr_start_scan()`.
4. Start asynchronous reading with `fobos_sdr_read_async()`.
5. Keep `scan_callback()` minimal: it reads the current scan index, copies the sample buffer into a bounded queue, and returns.
6. A worker thread consumes queued buffers, runs FFT magnitude averaging, aggregates one slot per hardware scan frequency, normalizes to fixed dB limits, and publishes complete spectrum/waterfall rows by SSE.

Do not move FFT, JSON generation, SSE broadcasting, or other expensive work back into `scan_callback()`.

## Frequency Rules

Frontend frequencies are air/signal frequencies. The backend converts them to receiver frequencies only immediately before programming the SDR.

- Positive converter: `receiver = radio - converter`
- Negative converter: `receiver = abs(-converter - radio)`
- If the scan needs more than `256` hardware frequency points, clamp the end frequency to the last covered air frequency and return that effective end to the frontend.
- The frequency scale and hover readout should use air frequencies. If converter is nonzero, the hover popup also shows receiver frequency on a second line.

## Bandwidth Ratio

`bw_ratio` controls both scan spacing and displayed FFT width:

- `1.0` means scan in `samplerate` steps and display the full FFT span for each step.
- `0.5` means scan in `samplerate / 2` steps and display only the centered half of each FFT result.
- Other ratios follow the same rule: `step = samplerate * bw_ratio`, and displayed bins are the centered `bw_ratio` part of the FFT.

Signal normalization should not depend on the number of displayed bins. Keep waterfall/spectrum row values on fixed dB limits unless intentionally changing the user-facing level scale.

## Build

Expected workspace layout:

```text
FobosSDR/
  fobos-scanner/
  libfobos-sdr-agile/
  local-agile/
```

Build from this directory:

```sh
make
```

Run with local library paths:

```sh
./run.sh
```

Equivalent direct command:

```sh
LD_LIBRARY_PATH=../libfobos-sdr-agile/build-local:../local-agile/lib ./fobos-scanner
```

Open the UI at:

```text
http://localhost:8080
```

## Dependencies

- GCC with C99 support
- GNU Make
- POSIX threads
- `libm`
- `libusb-1.0`
- Agile Fobos SDR headers in `../libfobos-sdr-agile` and `../libfobos-sdr-agile/fobos`
- Agile Fobos SDR library in `../libfobos-sdr-agile/build-local` or `../local-agile/lib`

On Debian/Ubuntu, system packages usually include:

```sh
sudo apt install build-essential libusb-1.0-0-dev
```

The agile Fobos SDR library must be built separately before this project.

## Checks

Run these before handing changes back:

```sh
make
perl -0777 -ne 'print $1 if /<script>(.*)<\/script>/s' index.html | node --check
```

If a `fobos-scanner` process is running, stop it before rebuilding because the linker may not be able to overwrite the active binary.
