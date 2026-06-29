# Agent Notes

## Project Shape

`fobos-scanner` is a small C backend plus a single-page HTML frontend for RigExpert Fobos SDR.

- `main.c` starts an HTTP server on port `8080`, serves `index.html`, exposes JSON API endpoints, and streams scan lines to the browser over Server-Sent Events.
- `index.html` owns the frontend controls, spectrum canvas, waterfall canvas, hover readout, and scan parameter requests.
- `Makefile` builds one binary, `fobos-scanner`, against the agile Fobos SDR library and `libusb`.
- `run.sh` starts the binary with the local agile library paths in `LD_LIBRARY_PATH`.
- `fobos-scanner.conf` is a local runtime config file and should stay untracked.
- `scanner-deepseek` is reference material and should stay untracked.
- `bands.ini` is a tracked, human-editable band overlay file served to the frontend.
- `tasks.md`, `PLAN.MD`, and `suggestions.md` are local planning/report files and should stay untracked.

## Scanner Architecture

The backend uses the agile Fobos SDR hardware scan API:

1. Build an air-frequency scan band from start/end frequency and `step = samplerate * bw_ratio`.
2. Cap the hardware scan table to `MAX_FREQS` (`256`) because the library scan list is limited.
3. Convert each air-frequency center to a receiver frequency before calling `fobos_sdr_start_scan()`.
4. Start asynchronous reading with `fobos_sdr_read_async()`.
5. Keep `scan_callback()` minimal: it reads the current scan index, copies the sample buffer into a bounded queue, and returns.
6. A worker thread consumes queued buffers, runs FFT magnitude averaging, aggregates one slot per hardware scan frequency, normalizes to fixed dB limits, and publishes complete spectrum/waterfall rows by SSE.

Do not move FFT, JSON generation, SSE broadcasting, or other expensive work back into `scan_callback()`.

When the current visible span needs only one hardware scan point, the backend uses normal single-frequency streaming instead of `fobos_sdr_start_scan()`. At high zoom, single mode may use a 3-stage CIC decimator before a 65536-point FFT. The decimator accumulates continuous raw I/Q buffers in the worker thread; do not rate-drop raw buffers before the CIC path because gaps corrupt the filtered stream.

Single mode distinguishes the visible span from the processed source span. When the processed span is wide enough, the backend tunes the receiver center just outside the visible window so the I/Q zero-frequency dip is not centered on the screen.

Single mode chooses the smallest power-of-two FFT size that can fill the current display, from `1024` through `65536`. If more frequency resolution is needed, increase decimation instead of using FFT sizes above `65536`. The `Min Rate` setting applies only to decimated single-stream mode and uses FFT-window overlap to meet the requested waterfall line cadence.

The backend should not auto-start a scan immediately after the server banner. The web UI starts scanning through `/api/start`, and if a scan is already active the UI should attach to `/api/waterfall` instead of restarting it. This avoids racing Fobos async USB setup during page load or refresh.

`/api/start` is for hardware scan changes: band, sample rate, bandwidth ratio, converter, direct sampling, and gains. It can restart the hardware scan.

`/api/fft` is for FFT-size changes only. It must not call `start_scan()`, must not call `fobos_sdr_start_scan()`, and must not reset frontend zoom, waterfall history, or brightness settings. The scan worker notices `g_fft_generation`, swaps FFT buffers, drops the partial mixed-size row, and continues processing the current hardware scan.

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

FFT magnitudes are normalized by the Hann window sum and compensated to a `1024`-point FFT reference bandwidth with `sqrt(fft_size / 1024)`. This keeps displayed levels comparable as FFT size changes. Preserve the division by the number of averaged FFTs.

The frontend uses peak-per-pixel reduction when more FFT bins exist than canvas pixels. Do not switch it back to nearest-bin sampling unless you also solve narrow-signal disappearance at high FFT sizes.

## Frontend Behavior

- Shift + mouse wheel zooms the spectrum/waterfall horizontally around the cursor.
- Holding Shift changes the spectrum/waterfall cursor to zoom mode.
- Frequency scale, hover frequency, and hover level must use the current zoomed view.
- Frequency scale ticks are rendered as border-left strokes, not 1px background rectangles. This fixed Firefox/browser-zoom cases where subpixel 1px tick divs disappeared.
- The scan label shows `from - to (bandwidth) MHz`.
- Waterfall min/max sliders currently allow values up to `400`.
- `/api/status` is polled as a heartbeat. If the backend is unreachable, the UI must show `disconnected`, not stale `scanning`.
- `bands.ini` is loaded by the frontend from `/bands.ini`; keep its syntax simple key/value sections with `name`, `start_mhz`, and `end_mhz`.

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
