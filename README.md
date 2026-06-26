# Fobos SDR Scanner

Browser-based spectrum and waterfall scanner for RigExpert Fobos SDR using the agile Fobos SDR API.

![Fobos SDR Scanner screenshot](images/screenshot01.png)

The program runs a small C HTTP server on `localhost:8080`, controls the SDR through `libfobos_sdr`, and streams waterfall lines to the browser with Server-Sent Events. The scanner uses the Fobos hardware scan API to sweep a frequency list, compute FFT magnitudes, and render the current spectrum and waterfall in `index.html`.

## Features

- Hardware-assisted frequency scanning through `fobos_sdr_start_scan()`
- Live spectrum and top-to-bottom waterfall display
- Hover readout for frequency and spectrum level
- Adjustable LNA/VGA gain while scanning
- Converter frequency offset for upconverter use
- Waterfall brightness window controls
- Configurable frequency range, sample rate, bandwidth ratio, and direct sampling mode

## Dependencies

This project expects to be built inside the Fobos SDR workspace layout used by this repository:

```text
FobosSDR/
  fobos-scanner/
  libfobos-sdr-agile/
  local-agile/
```

Required tools and libraries:

- GCC with C99 support
- GNU Make
- POSIX threads
- math library (`libm`)
- `libusb-1.0`
- Fobos SDR agile headers and library
  - headers: `../libfobos-sdr-agile`, `../libfobos-sdr-agile/fobos`
  - library: `../libfobos-sdr-agile/build-local/libfobos_sdr`
- local agile dependency prefix, if used by your build
  - headers: `../local-agile/include`
  - libraries: `../local-agile/lib`

On Debian/Ubuntu, the system build tools and libusb headers are typically installed with:

```sh
sudo apt install build-essential libusb-1.0-0-dev
```

The Fobos SDR agile library must be built separately before building this scanner.

## Build

From the `fobos-scanner` directory:

```sh
make
```

This produces:

```text
./fobos-scanner
```

To remove the built binary:

```sh
make clean
```

## Run

Use the included wrapper so the local Fobos SDR libraries are on `LD_LIBRARY_PATH`:

```sh
./run.sh
```

Or use the Makefile target:

```sh
make run
```

Then open:

```text
http://localhost:8080
```

## Notes

- Default scan start frequency is `50 MHz`.
- The backend listens on port `8080`.
- The scanner needs access to a connected Fobos SDR supported by the agile firmware/API.
- Gain sliders update the device live through `POST /api/gain`.
- Waterfall data is sent as compact `uint8` magnitude rows over Server-Sent Events.

