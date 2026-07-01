# Agent Notes

## Project Shape

`fobos-scanner` is a small C backend plus a single-page HTML frontend for RigExpert Fobos SDR.

- `main.c` starts an HTTP server on port `8080`, serves `index.html`, exposes JSON API endpoints, and streams scan lines to the browser over Server-Sent Events.
- `index.html` owns the frontend controls, spectrum canvas, waterfall canvas, hover readout, and scan parameter requests.
- `Makefile` builds `fobos-scanner`, plus the standalone `tools/fobos-stream-test` and `tools/fobos-fq-response` utilities against the agile Fobos SDR library and `libusb`.
- `run-scanner.sh` starts the binary with the local agile library paths in `LD_LIBRARY_PATH`.
- `fobos-scanner.conf` is a local runtime config file and should stay untracked.
- `scanner-deepseek` is reference material and should stay untracked.
- `bands.ini` is a tracked, human-editable band overlay file served to the frontend.
- `markers.ini` is a tracked, human-editable marker file; `/api/markers/save` must validate syntax and use atomic replacement.
- `tasks.md`, `PLAN.MD`, and `suggestions.md` are local planning/report files and should stay untracked.

## Stream Test Utility

`tools/fobos_stream_test.c` builds as `tools/fobos-stream-test`. It must stay independent from the web scanner backend. It opens the receiver in normal single-frequency async streaming mode and reports stream health from callback timing and sample contents.

The public agile callback does not expose a hardware sequence number. Do not claim exact buffer order/loss detection unless the library API changes; infer likely missing buffers from callback cadence, large gaps, and expected callback counts.

## Frequency Response Utility

`tools/fobos_freq_response.c` builds as `tools/fobos-fq-response`. It is independent from the web scanner backend and uses synchronous Fobos reads to measure a noise-only passband response with antennas disconnected.

The utility intentionally captures at many receiver center frequencies over multiple passes, robust-averages per-bin dB spectra, clamps narrow upward peaks against a local low-percentile baseline, smooths the result, and optionally mirror-averages around DC. Its default artifacts are `fq_response.txt` and `fq_response.png`, which are local calibration outputs and should stay untracked. `tools/run-fq-response.sh` must `cd` to the scanner root before running so these artifacts land beside `main.c`, not inside `tools/`.

The text output contains `response_db`, `correction_db`, `correction_linear`, `raw_avg_db`, and `despurred_db`; future scanner compensation should consume the correction values as an inverse frequency-domain window, not as waterfall brightness settings. Do not let narrow spurs become inverse correction notches.

`main.c` loads `fq_response.txt` at startup when present and applies the smoothed `correction_linear` column only in hardware scan mode. The table is FFT-shifted low-to-high: row `0` is `-Fs/2`, the middle row is DC, and the final row is `+Fs/2`; do not index it by unshifted FFT bins. The scanner indexes by physical baseband offset, using calibration sample-rate metadata when present, so lower BW ratios use the centered corresponding slice of a BW=1.0 calibration instead of compressing the full edge correction into the narrower scan slice. Use interpolation for smaller FFT sizes and the centered subset for a shorter final scan slice. Do not apply this table to single-frequency fixed/zoom mode unless that mode gets its own correction design.

## Scanner Architecture

The backend uses the agile Fobos SDR hardware scan API:

1. Build an air-frequency scan band from start/end frequency and `step = samplerate * bw_ratio`.
2. Cap the hardware scan table to `MAX_FREQS` (`256`) because the library scan list is limited.
3. Convert each air-frequency center to a receiver frequency before calling `fobos_sdr_start_scan()`.
4. Start asynchronous reading with `fobos_sdr_read_async()`.
5. Keep `scan_callback()` minimal: it reads the current scan index, copies the sample buffer into a bounded queue, and returns.
6. A worker thread consumes queued buffers, runs FFT magnitude averaging, aggregates one slot per hardware scan frequency, normalizes to fixed dB limits, and publishes complete spectrum/waterfall rows by SSE.

Hardware scan mode intentionally uses `SCAN_BUF_LEN = 98304` complex samples per scan point, not the agile API minimum `65536`. Broad scans can otherwise produce `fobos_sdr_get_scan_index() == -1` buffers while the receiver is still tuning, leaving whole scan slots blank near scan edges or tuner band transitions.

Do not move FFT, JSON generation, SSE broadcasting, or other expensive work back into `scan_callback()`.

When the current visible span needs only one hardware scan point, the backend uses normal single-frequency streaming instead of `fobos_sdr_start_scan()`. At high zoom, single mode may use a 3-stage CIC decimator before a 65536-point FFT. The decimator accumulates continuous raw I/Q buffers in the worker thread; do not rate-drop raw buffers before the CIC path because gaps corrupt the filtered stream.

Single mode distinguishes the visible span from the processed source span. When the processed span is wide enough, the backend tunes the receiver center just outside the visible window so the I/Q zero-frequency dip is not centered on the screen.

Single mode chooses the smallest power-of-two FFT size that can fill the current display, from `1024` through `65536`. If more frequency resolution is needed, increase decimation instead of using FFT sizes above `65536`. The `Min Rate` setting applies only to decimated single-stream mode and uses FFT-window overlap to meet the requested waterfall line cadence.

Fresh configs default to a minimum waterfall rate of `10 lines/s` and a maximum rate limit of `20 lines/s`; saved `fobos-scanner.conf` values override those defaults.

The backend should not auto-start a scan immediately after the server banner. The web UI starts scanning through `/api/start`, and if a scan is already active the UI should attach to `/api/waterfall` instead of restarting it. This avoids racing Fobos async USB setup during page load or refresh.

`/api/start` is for scan changes: band, software bandwidth ratio, converter, direct sampling, and gains. It can restart the hardware scan. The scanner sample rate is fixed at 50 MHz; the frontend sample-rate control is disabled and the backend ignores incoming sample-rate values for scanner starts.

Gain limits must match the Fobos SDR API behavior used by the UI and `fobos-stream-test`: LNA accepts `0..3`, VGA accepts `0..31`.

`/api/fft` is for FFT-size changes only. It must not call `start_scan()`, must not call `fobos_sdr_start_scan()`, and must not reset frontend zoom, waterfall history, or brightness settings. The scan worker notices `g_fft_generation`, swaps FFT buffers, drops the partial mixed-size row, and continues processing the current hardware scan.

The backend HTTP layer intentionally uses a small flat-JSON parser for control endpoints. Keep parsing centralized: do not reintroduce repeated endpoint-local `strstr()` plus `sscanf()` scans. Bad JSON, invalid numeric fields, oversized bodies, unsupported methods, and invalid marker files should return explicit JSON error responses.

`fobos-scanner.conf` persists both configured scan range and visible range. Do not reset the visible range while clamping hardware scan limits unless the current visible range is outside the valid configured bounds.

## Backend/Frontend Traffic

Live display data flows over `GET /api/waterfall` as Server-Sent Events. Each event is named `line` and contains JSON metadata plus a `d` array of `display_bins` unsigned 8-bit magnitudes encoded as decimal JSON numbers. The frontend uses that row for both the latest spectrum trace and one waterfall row.

The backend must reduce processed FFT/source bins to exactly `display_bins` values before sending. Do not send all FFT bins unless the frontend protocol and traffic budget are deliberately redesigned.

`publish_scan_line()` must size its SSE JSON buffer from the actual metadata length plus worst-case row data. Avoid fixed optimistic constants for line payload allocation.

The `traffic_kbytes_s` value reported by `/api/status` is measured from actual bytes successfully written by `sse_broadcast()` over the recent traffic window. It is aggregate SSE payload traffic across connected clients. It does not include most `/api/status` polling, control POSTs, static files, or browser/TCP/IP framing overhead.

Per-line SSE traffic is:

```text
B_line = H + (N - 1) + sum(digits(v_i), i = 0..N-1)
```

Where `N = display_bins`, `v_i` are values in `d`, and `H` is fixed SSE/JSON metadata overhead for the line. For one browser:

```text
T_sse_kbytes_per_sec = R * B_line / 1024
```

For multiple connected browser tabs, multiply by the number of clients. Use this formula for estimates, but use the measured `traffic_kbytes_s` field when describing what the program currently sends.

## Frequency Rules

Frontend frequencies are air/signal frequencies. The backend converts them to receiver frequencies only immediately before programming the SDR.

- Positive converter: `receiver = radio - converter`
- Negative converter: `receiver = abs(-converter - radio)`
- In RF-input mode, configured air-frequency ranges must be clamped so converted receiver frequencies stay within `50 MHz` through `6000 MHz` before the 256-point hardware scan clamp is applied.
- If the scan needs more than `256` hardware frequency points, clamp the end frequency to the last covered air frequency and return that effective end to the frontend.
- The frequency scale and hover readout should use air frequencies. If converter is nonzero, the hover popup also shows receiver frequency on a second line.

## Bandwidth Ratio

`bw_ratio` is a software scanner ratio, labeled `BW USAGE IN AUTO SCAN MODE` in the UI. Its default is `0.9`. It controls scan spacing and displayed FFT width only; do not use it to narrow the Fobos SDR hardware auto bandwidth. Scanner starts should keep the hardware passband full with `fobos_sdr_set_auto_bandwidth(..., 1.0)` so each 50 MHz stream contains the full receiver bandwidth.

- `1.0` means scan in `samplerate` steps and display the full FFT span for each step.
- `0.5` means scan in `samplerate / 2` steps and display only the centered half of each FFT result.
- Other ratios follow the same rule: `step = samplerate * bw_ratio`, and displayed bins are the centered `bw_ratio` part of the FFT.

Signal normalization should not depend on the number of displayed bins. Keep waterfall/spectrum row values on fixed dB limits unless intentionally changing the user-facing level scale.

FFT magnitudes are normalized by the Hann window sum and compensated to a `1024`-point FFT reference bandwidth with `sqrt(fft_size / 1024)`. This keeps displayed levels comparable as FFT size changes. Preserve the division by the number of averaged FFTs.

The frontend uses peak-per-pixel reduction when more FFT bins exist than canvas pixels. Do not switch it back to nearest-bin sampling unless you also solve narrow-signal disappearance at high FFT sizes.

## Frontend Behavior

- Shift + wheel zooms the spectrum/waterfall horizontally around the cursor.
- Holding Shift changes the spectrum/waterfall cursor to zoom mode.
- Plain `+`, `-`, arrow keys, and `0` are app zoom/pan shortcuts, but do not intercept browser zoom shortcuts such as `Ctrl++`, `Ctrl+-`, or `Ctrl+0`.
- Frequency markers open with Shift + left-click, or Ctrl/Alt + right-click. Do not use Shift + right-click for marker editing; Firefox reserves that shortcut for its native context menu and page JavaScript cannot suppress it reliably.
- Frequency scale, hover frequency, and hover level must use the current zoomed view.
- Frequency scale ticks are rendered as border-left strokes, not 1px background rectangles. This fixed Firefox/browser-zoom cases where subpixel 1px tick divs disappeared.
- The scan label shows `from - to (bandwidth) MHz`.
- The first status line starts with `SW: YYYYMMDD`, generated from the backend compile date in `/api/status`.
- Waterfall min/max sliders currently allow values up to `400`.
- Auto waterfall levels are enabled by default on first page load unless the user has a saved preference.
- `/api/status` is polled as a heartbeat. If the backend is unreachable, the UI must show `disconnected`, not stale `scanning`.
- SSE line events must be validated before rendering. Ignore malformed rows and stale-width rows; schedule a debounced `/api/view` update when `display_bins` differs.
- Visible-view persistence in localStorage must be throttled. Do not write once per waterfall row.
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

The Makefile supports overriding `FOBOS_SRC`, `FOBOS_BUILD`, and `FOBOS_LOCAL`. Keep these variables working when changing include or library paths.

Run with local library paths:

```sh
./run-scanner.sh
```

Equivalent direct command:

```sh
LD_LIBRARY_PATH=../libfobos-sdr-agile/build-local:../local-agile/lib ./fobos-scanner
```

Run the stream integrity tester:

```sh
./tools/run-stream-test.sh
```

Run the frequency response calibration utility:

```sh
./tools/run-fq-response.sh
```

Run compile checks:

```sh
make check
```

With the backend running, use `tools/http_smoke_test.sh` for parser/error-path smoke checks.

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
