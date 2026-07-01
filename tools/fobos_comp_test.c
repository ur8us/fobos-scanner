/*
 * Fobos SDR compensation diagnostic.
 *
 * Captures a real single-frequency stream, averages a normal 1024-point
 * shifted FFT, applies fq_response.txt with the same bin-center mapping used
 * by the scanner, and writes one PNG plot per BW usage 0.1 .. 1.0.
 */

#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <errno.h>
#include <fobos_sdr.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FFT_SIZE            1024U
#define SYNC_BUF_LEN        8192U
#define DEFAULT_FREQ_MHZ    100.0
#define DEFAULT_SAMPLERATE  50000000.0
#define DEFAULT_BUFFERS     128U
#define DEFAULT_SETTLE      8U
#define DEFAULT_LNA_GAIN    2U
#define DEFAULT_VGA_GAIN    15U
#define DEFAULT_DIRECT      0U
#define DEFAULT_CLK_SOURCE  0
#define DEFAULT_FQ_RESPONSE "fq_response.txt"
#define DEFAULT_OUT_DIR     "comp_test_out"
#define DEFAULT_OUT_PREFIX  "comp_test"
#define LNA_GAIN_MAX        3U
#define VGA_GAIN_MAX        31U
#define HARDWARE_AUTO_BW    1.0

typedef struct {
    double frequency_hz;
    double samplerate;
    uint32_t buffers;
    uint32_t settle_buffers;
    unsigned int lna_gain;
    unsigned int vga_gain;
    unsigned int direct_sampling;
    int clk_source;
    const char *fq_response_path;
    const char *out_dir;
    const char *out_prefix;
} comp_config_t;

typedef struct {
    float *correction;
    size_t count;
    double samplerate_hz;
} correction_table_t;

static int parse_double_value(const char *text, double *out)
{
    char *end = NULL;
    double value;

    errno = 0;
    value = strtod(text, &end);
    if (errno || end == text)
        return -1;
    while (*end && isspace((unsigned char)*end))
        end++;
    if (*end)
        return -1;
    *out = value;
    return 0;
}

static int parse_hz_value(const char *text, double *out)
{
    char *end = NULL;
    double value;
    double scale = 1.0;

    errno = 0;
    value = strtod(text, &end);
    if (errno || end == text)
        return -1;
    while (*end && isspace((unsigned char)*end))
        end++;
    if (*end) {
        char suffix = (char)tolower((unsigned char)*end++);
        if (suffix == 'k')
            scale = 1.0e3;
        else if (suffix == 'm')
            scale = 1.0e6;
        else if (suffix == 'g')
            scale = 1.0e9;
        else
            return -1;
        if (*end == 'h' || *end == 'H')
            end++;
        if (*end == 'z' || *end == 'Z')
            end++;
        while (*end && isspace((unsigned char)*end))
            end++;
        if (*end)
            return -1;
    }
    *out = value * scale;
    return 0;
}

static int parse_uint_value(const char *text, uint32_t *out)
{
    char *end = NULL;
    unsigned long value;

    errno = 0;
    value = strtoul(text, &end, 0);
    if (errno || end == text || value > UINT32_MAX)
        return -1;
    while (*end && isspace((unsigned char)*end))
        end++;
    if (*end)
        return -1;
    *out = (uint32_t)value;
    return 0;
}

static int parse_clock_value(const char *text, int *out)
{
    if (strcmp(text, "internal") == 0 || strcmp(text, "int") == 0 ||
        strcmp(text, "0") == 0) {
        *out = 0;
        return 0;
    }
    if (strcmp(text, "external") == 0 || strcmp(text, "ext") == 0 ||
        strcmp(text, "1") == 0) {
        *out = 1;
        return 0;
    }
    return -1;
}

static void print_usage(const char *argv0)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "Captures a real stream and writes one PNG per BW usage with raw spectrum, correction curve, and compensated spectrum.\n"
        "\n"
        "Options:\n"
        "  -f, --freq-mhz MHz       RF center frequency in MHz (default %.3f)\n"
        "      --freq-hz HZ         RF center frequency in Hz\n"
        "  -r, --samplerate HZ      sample rate, suffixes k/m/g accepted (default %.0f)\n"
        "      --buffers N          sync buffers averaged, %u samples each (default %u)\n"
        "      --settle-buffers N   sync buffers discarded after tuning (default %u)\n"
        "      --lna N              LNA gain 0..3 (default %u)\n"
        "      --vga N              VGA gain 0..31 (default %u)\n"
        "      --direct N           direct sampling mode 0/1/2 (default %u)\n"
        "      --clock internal|external|0|1\n"
        "      --fq-response PATH   correction text file (default %s)\n"
        "      --out-dir DIR        output directory (default %s)\n"
        "      --out-prefix NAME    output filename prefix (default %s)\n"
        "  -h, --help              show this help\n",
        argv0, DEFAULT_FREQ_MHZ, DEFAULT_SAMPLERATE, SYNC_BUF_LEN,
        DEFAULT_BUFFERS, DEFAULT_SETTLE, DEFAULT_LNA_GAIN, DEFAULT_VGA_GAIN,
        DEFAULT_DIRECT, DEFAULT_FQ_RESPONSE, DEFAULT_OUT_DIR,
        DEFAULT_OUT_PREFIX);
}

static int parse_args(int argc, char **argv, comp_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->frequency_hz = DEFAULT_FREQ_MHZ * 1.0e6;
    cfg->samplerate = DEFAULT_SAMPLERATE;
    cfg->buffers = DEFAULT_BUFFERS;
    cfg->settle_buffers = DEFAULT_SETTLE;
    cfg->lna_gain = DEFAULT_LNA_GAIN;
    cfg->vga_gain = DEFAULT_VGA_GAIN;
    cfg->direct_sampling = DEFAULT_DIRECT;
    cfg->clk_source = DEFAULT_CLK_SOURCE;
    cfg->fq_response_path = DEFAULT_FQ_RESPONSE;
    cfg->out_dir = DEFAULT_OUT_DIR;
    cfg->out_prefix = DEFAULT_OUT_PREFIX;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *value = NULL;
        double dtmp;
        uint32_t utmp;

        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }

        if (strcmp(arg, "-f") == 0 || strcmp(arg, "--freq-mhz") == 0 ||
            strcmp(arg, "--freq-hz") == 0 ||
            strcmp(arg, "-r") == 0 || strcmp(arg, "--samplerate") == 0 ||
            strcmp(arg, "--buffers") == 0 ||
            strcmp(arg, "--settle-buffers") == 0 ||
            strcmp(arg, "--lna") == 0 || strcmp(arg, "--vga") == 0 ||
            strcmp(arg, "--direct") == 0 || strcmp(arg, "--clock") == 0 ||
            strcmp(arg, "--fq-response") == 0 ||
            strcmp(arg, "--out-dir") == 0 ||
            strcmp(arg, "--out-prefix") == 0) {
            if (i + 1 >= argc) {
                fprintf(stderr, "Missing value for %s\n", arg);
                return -1;
            }
            value = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", arg);
            return -1;
        }

        if (strcmp(arg, "-f") == 0 || strcmp(arg, "--freq-mhz") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp < 0.0)
                return -1;
            cfg->frequency_hz = dtmp * 1.0e6;
        } else if (strcmp(arg, "--freq-hz") == 0) {
            if (parse_hz_value(value, &dtmp) != 0 || dtmp < 0.0)
                return -1;
            cfg->frequency_hz = dtmp;
        } else if (strcmp(arg, "-r") == 0 || strcmp(arg, "--samplerate") == 0) {
            if (parse_hz_value(value, &dtmp) != 0 || dtmp <= 0.0)
                return -1;
            cfg->samplerate = dtmp;
        } else if (strcmp(arg, "--buffers") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp == 0)
                return -1;
            cfg->buffers = utmp;
        } else if (strcmp(arg, "--settle-buffers") == 0) {
            if (parse_uint_value(value, &utmp) != 0)
                return -1;
            cfg->settle_buffers = utmp;
        } else if (strcmp(arg, "--lna") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > LNA_GAIN_MAX)
                return -1;
            cfg->lna_gain = utmp;
        } else if (strcmp(arg, "--vga") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > VGA_GAIN_MAX)
                return -1;
            cfg->vga_gain = utmp;
        } else if (strcmp(arg, "--direct") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > 2U)
                return -1;
            cfg->direct_sampling = utmp;
        } else if (strcmp(arg, "--clock") == 0) {
            if (parse_clock_value(value, &cfg->clk_source) != 0)
                return -1;
        } else if (strcmp(arg, "--fq-response") == 0) {
            cfg->fq_response_path = value;
        } else if (strcmp(arg, "--out-dir") == 0) {
            cfg->out_dir = value;
        } else if (strcmp(arg, "--out-prefix") == 0) {
            cfg->out_prefix = value;
        }
    }

    return 0;
}

static int ensure_dir(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0)
        return S_ISDIR(st.st_mode) ? 0 : -1;
    if (mkdir(path, 0775) != 0 && errno != EEXIST)
        return -1;
    return 0;
}

static int append_correction(correction_table_t *table, size_t *capacity,
                             float correction)
{
    if (table->count == *capacity) {
        size_t new_capacity = *capacity ? *capacity * 2U : 4096U;
        float *new_values = realloc(table->correction,
                                    new_capacity * sizeof(float));
        if (!new_values)
            return -1;
        table->correction = new_values;
        *capacity = new_capacity;
    }
    table->correction[table->count++] = correction;
    return 0;
}

static int load_correction_table(const char *path, correction_table_t *table)
{
    FILE *f;
    char line[1024];
    size_t capacity = 0;

    memset(table, 0, sizeof(*table));
    f = fopen(path, "r");
    if (!f) {
        perror(path);
        return -1;
    }

    while (fgets(line, sizeof(line), f)) {
        char *p = line;
        unsigned int bin;
        double offset_hz;
        double offset_mhz;
        double response_db;
        double correction_db;
        double correction_linear;

        while (*p && isspace((unsigned char)*p))
            p++;
        if (*p == '\0')
            continue;
        if (*p == '#') {
            if (sscanf(p, "# samplerate_hz %lf", &table->samplerate_hz) == 1)
                continue;
            continue;
        }

        if (sscanf(p, "%u %lf %lf %lf %lf %lf",
                   &bin, &offset_hz, &offset_mhz, &response_db,
                   &correction_db, &correction_linear) == 6 &&
            isfinite(correction_linear) && correction_linear > 0.0 &&
            correction_linear < 100.0) {
            if (append_correction(table, &capacity,
                                  (float)correction_linear) != 0) {
                fclose(f);
                return -1;
            }
        }
    }
    fclose(f);

    if (table->count < 2 || !table->correction) {
        fprintf(stderr, "%s: no usable correction rows\n", path);
        free(table->correction);
        memset(table, 0, sizeof(*table));
        return -1;
    }
    if (table->samplerate_hz <= 0.0)
        table->samplerate_hz = DEFAULT_SAMPLERATE;

    return 0;
}

static float correction_at_index(const correction_table_t *table, double idxd)
{
    size_t lo;
    size_t hi;
    double frac;

    if (!table->correction || table->count == 0)
        return 1.0f;
    if (table->count == 1)
        return table->correction[0];
    if (idxd < 0.0)
        idxd = 0.0;
    if (idxd > (double)(table->count - 1U))
        idxd = (double)(table->count - 1U);

    lo = (size_t)floor(idxd);
    if (lo >= table->count - 1U)
        return table->correction[table->count - 1U];
    hi = lo + 1U;
    frac = idxd - (double)lo;
    return (float)((1.0 - frac) * (double)table->correction[lo] +
                   frac * (double)table->correction[hi]);
}

static float correction_for_shifted_bin(const correction_table_t *table,
                                        int shifted_bin, double samplerate)
{
    double offset_hz = (((double)shifted_bin / (double)FFT_SIZE) - 0.5) *
        samplerate;
    double idxd = ((offset_hz / table->samplerate_hz) + 0.5) *
        (double)table->count;
    return correction_at_index(table, idxd);
}

static void fft_c2c(float *data, int n)
{
    int j = 0;

    for (int i = 1; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = data[2*i];
            float ti = data[2*i + 1];
            data[2*i] = data[2*j];
            data[2*i + 1] = data[2*j + 1];
            data[2*j] = tr;
            data[2*j + 1] = ti;
        }
    }

    for (int len = 2; len <= n; len <<= 1) {
        double ang = -2.0 * M_PI / (double)len;
        float wlen_r = (float)cos(ang);
        float wlen_i = (float)sin(ang);
        for (int i = 0; i < n; i += len) {
            float wr = 1.0f;
            float wi = 0.0f;
            for (int k = 0; k < len / 2; k++) {
                int u = i + k;
                int v = i + k + len / 2;
                float ur = data[2*u];
                float ui = data[2*u + 1];
                float vr = data[2*v] * wr - data[2*v + 1] * wi;
                float vi = data[2*v] * wi + data[2*v + 1] * wr;
                data[2*u] = ur + vr;
                data[2*u + 1] = ui + vi;
                data[2*v] = ur - vr;
                data[2*v + 1] = ui - vi;
                {
                    float nwr = wr * wlen_r - wi * wlen_i;
                    wi = wr * wlen_i + wi * wlen_r;
                    wr = nwr;
                }
            }
        }
    }
}

static void init_window(float *window)
{
    for (uint32_t i = 0; i < FFT_SIZE; i++)
        window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI *
            (float)i / (float)(FFT_SIZE - 1U)));
}

static int check_ret(const char *name, int ret)
{
    if (ret != FOBOS_ERR_OK) {
        fprintf(stderr, "%s failed: %d\n", name, ret);
        return -1;
    }
    return 0;
}

static int process_fft_block(const float *buf, uint32_t pos,
                             const float *window, float *work,
                             double *accum)
{
    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        float w = window[i];
        work[2U*i] = buf[2U*(pos + i)] * w;
        work[2U*i + 1U] = buf[2U*(pos + i) + 1U] * w;
    }

    fft_c2c(work, (int)FFT_SIZE);

    for (uint32_t i = 0; i < FFT_SIZE; i++) {
        uint32_t fft_bin = (i + FFT_SIZE / 2U) % FFT_SIZE;
        float re = work[2U*fft_bin];
        float im = work[2U*fft_bin + 1U];
        accum[i] += sqrt((double)re * (double)re + (double)im * (double)im);
    }

    return 0;
}

static int capture_spectrum(struct fobos_sdr_dev_t *dev,
                            const comp_config_t *cfg, const float *window,
                            double *spectrum, uint64_t *out_fft_count)
{
    float *buf = malloc((size_t)SYNC_BUF_LEN * 2U * sizeof(float));
    float *work = malloc((size_t)FFT_SIZE * 2U * sizeof(float));
    double *accum = calloc((size_t)FFT_SIZE, sizeof(double));
    uint64_t fft_count = 0;
    float window_sum = 0.0f;
    int ret;

    if (!buf || !work || !accum) {
        fprintf(stderr, "Out of memory while preparing capture buffers\n");
        free(buf);
        free(work);
        free(accum);
        return -1;
    }

    if (!cfg->direct_sampling) {
        ret = fobos_sdr_set_frequency(dev, cfg->frequency_hz);
        if (check_ret("fobos_sdr_set_frequency", ret) != 0)
            goto fail;
    }

    ret = fobos_sdr_start_sync(dev, SYNC_BUF_LEN);
    if (check_ret("fobos_sdr_start_sync", ret) != 0)
        goto fail;

    for (uint32_t i = 0; i < cfg->settle_buffers; i++) {
        uint32_t actual = SYNC_BUF_LEN;
        ret = fobos_sdr_read_sync(dev, buf, &actual);
        if (ret != FOBOS_ERR_OK)
            break;
    }

    for (uint32_t b = 0; b < cfg->buffers; b++) {
        uint32_t actual = SYNC_BUF_LEN;
        ret = fobos_sdr_read_sync(dev, buf, &actual);
        if (ret != FOBOS_ERR_OK) {
            fprintf(stderr, "fobos_sdr_read_sync failed: %d\n", ret);
            break;
        }
        for (uint32_t pos = 0; pos + FFT_SIZE <= actual; pos += FFT_SIZE) {
            process_fft_block(buf, pos, window, work, accum);
            fft_count++;
        }
    }

    fobos_sdr_stop_sync(dev);

    if (fft_count == 0) {
        fprintf(stderr, "No complete 1024-point FFT blocks captured\n");
        goto fail_no_stop;
    }

    for (uint32_t i = 0; i < FFT_SIZE; i++)
        window_sum += window[i];
    if (window_sum <= 0.0f)
        window_sum = (float)FFT_SIZE;

    for (uint32_t i = 0; i < FFT_SIZE; i++)
        spectrum[i] = (accum[i] / (double)fft_count) / (double)window_sum;

    *out_fft_count = fft_count;
    free(buf);
    free(work);
    free(accum);
    return 0;

fail:
    fobos_sdr_stop_sync(dev);
fail_no_stop:
    free(buf);
    free(work);
    free(accum);
    return -1;
}

static void put_pixel(unsigned char *img, int w, int h, int x, int y,
                      unsigned char r, unsigned char g, unsigned char b)
{
    size_t idx;
    if (x < 0 || y < 0 || x >= w || y >= h)
        return;
    idx = ((size_t)y * (size_t)w + (size_t)x) * 3U;
    img[idx] = r;
    img[idx + 1U] = g;
    img[idx + 2U] = b;
}

static void draw_line(unsigned char *img, int w, int h,
                      int x0, int y0, int x1, int y1,
                      unsigned char r, unsigned char g, unsigned char b)
{
    int dx = abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        put_pixel(img, w, h, x0, y0, r, g, b);
        if (x0 == x1 && y0 == y1)
            break;
        {
            int e2 = 2 * err;
            if (e2 >= dy) {
                err += dy;
                x0 += sx;
            }
            if (e2 <= dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
}

static double nice_step(double range, int target_ticks)
{
    double rough;
    double base;
    double frac;
    if (range <= 0.0)
        return 1.0;
    rough = range / (double)target_ticks;
    base = pow(10.0, floor(log10(rough)));
    frac = rough / base;
    if (frac <= 1.0)
        return base;
    if (frac <= 2.0)
        return 2.0 * base;
    if (frac <= 5.0)
        return 5.0 * base;
    return 10.0 * base;
}

static void write_be32(FILE *f, uint32_t v)
{
    fputc((int)((v >> 24) & 0xffU), f);
    fputc((int)((v >> 16) & 0xffU), f);
    fputc((int)((v >> 8) & 0xffU), f);
    fputc((int)(v & 0xffU), f);
}

static uint32_t crc32_step(uint32_t crc, const unsigned char *buf, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xedb88320U & (uint32_t)-(int)(crc & 1U));
    }
    return ~crc;
}

static uint32_t adler32_step(uint32_t adler, const unsigned char *buf,
                             size_t len)
{
    uint32_t a = adler & 0xffffU;
    uint32_t b = (adler >> 16) & 0xffffU;
    for (size_t i = 0; i < len; i++) {
        a = (a + buf[i]) % 65521U;
        b = (b + a) % 65521U;
    }
    return (b << 16) | a;
}

static int write_chunk(FILE *f, const char type[4], const unsigned char *data,
                       uint32_t len)
{
    uint32_t crc = 0;
    write_be32(f, len);
    if (fwrite(type, 1, 4, f) != 4)
        return -1;
    if (len && fwrite(data, 1, len, f) != len)
        return -1;
    crc = crc32_step(crc, (const unsigned char *)type, 4);
    if (len)
        crc = crc32_step(crc, data, len);
    write_be32(f, crc);
    return ferror(f) ? -1 : 0;
}

static int write_png_rgb(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *f;
    unsigned char *raw = NULL;
    unsigned char *zdata = NULL;
    size_t stride = (size_t)w * 3U + 1U;
    size_t raw_len = stride * (size_t)h;
    size_t max_blocks = raw_len / 65535U + 1U;
    size_t zcap = 2U + raw_len + max_blocks * 5U + 4U;
    size_t rpos = 0;
    size_t zpos = 0;
    uint32_t adler = 1U;
    unsigned char ihdr[13];
    static const unsigned char sig[8] = {
        137, 80, 78, 71, 13, 10, 26, 10
    };

    raw = malloc(raw_len);
    zdata = malloc(zcap);
    if (!raw || !zdata) {
        free(raw);
        free(zdata);
        return -1;
    }

    for (int y = 0; y < h; y++) {
        raw[(size_t)y * stride] = 0;
        memcpy(raw + (size_t)y * stride + 1U,
               rgb + (size_t)y * (size_t)w * 3U,
               (size_t)w * 3U);
    }

    zdata[zpos++] = 0x78;
    zdata[zpos++] = 0x01;
    while (rpos < raw_len) {
        size_t block = raw_len - rpos;
        int final;
        if (block > 65535U)
            block = 65535U;
        final = (rpos + block == raw_len);
        zdata[zpos++] = (unsigned char)(final ? 1 : 0);
        zdata[zpos++] = (unsigned char)(block & 0xffU);
        zdata[zpos++] = (unsigned char)((block >> 8) & 0xffU);
        zdata[zpos++] = (unsigned char)((~block) & 0xffU);
        zdata[zpos++] = (unsigned char)(((~block) >> 8) & 0xffU);
        memcpy(zdata + zpos, raw + rpos, block);
        adler = adler32_step(adler, raw + rpos, block);
        zpos += block;
        rpos += block;
    }
    zdata[zpos++] = (unsigned char)((adler >> 24) & 0xffU);
    zdata[zpos++] = (unsigned char)((adler >> 16) & 0xffU);
    zdata[zpos++] = (unsigned char)((adler >> 8) & 0xffU);
    zdata[zpos++] = (unsigned char)(adler & 0xffU);

    f = fopen(path, "wb");
    if (!f) {
        perror(path);
        free(raw);
        free(zdata);
        return -1;
    }

    fwrite(sig, 1, sizeof(sig), f);
    ihdr[0] = (unsigned char)((w >> 24) & 0xff);
    ihdr[1] = (unsigned char)((w >> 16) & 0xff);
    ihdr[2] = (unsigned char)((w >> 8) & 0xff);
    ihdr[3] = (unsigned char)(w & 0xff);
    ihdr[4] = (unsigned char)((h >> 24) & 0xff);
    ihdr[5] = (unsigned char)((h >> 16) & 0xff);
    ihdr[6] = (unsigned char)((h >> 8) & 0xff);
    ihdr[7] = (unsigned char)(h & 0xff);
    ihdr[8] = 8;
    ihdr[9] = 2;
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;
    if (write_chunk(f, "IHDR", ihdr, sizeof(ihdr)) != 0 ||
        write_chunk(f, "IDAT", zdata, (uint32_t)zpos) != 0 ||
        write_chunk(f, "IEND", NULL, 0) != 0) {
        fclose(f);
        free(raw);
        free(zdata);
        return -1;
    }

    fclose(f);
    free(raw);
    free(zdata);
    return 0;
}

static void draw_axes(unsigned char *img, int w, int h, int left, int top,
                      int plot_w, int plot_h, double ymin, double ymax)
{
    double ystep = nice_step(ymax - ymin, 8);
    double ystart = ceil(ymin / ystep) * ystep;

    for (double yv = ystart; yv <= ymax + 1.0e-9; yv += ystep) {
        int y = top + (int)lrint((ymax - yv) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, left, y, left + plot_w, y, 228, 233, 238);
    }
    draw_line(img, w, h, left, top, left, top + plot_h, 25, 35, 45);
    draw_line(img, w, h, left, top + plot_h, left + plot_w, top + plot_h,
              25, 35, 45);
}

static void draw_curve(unsigned char *img, int w, int h, const double *data,
                       int count, int left, int top, int plot_w, int plot_h,
                       double ymin, double ymax,
                       unsigned char r, unsigned char g, unsigned char b,
                       int thick)
{
    for (int i = 1; i < count; i++) {
        int x0 = left + (int)(((int64_t)(i - 1) * plot_w) / (int64_t)(count - 1));
        int x1 = left + (int)(((int64_t)i * plot_w) / (int64_t)(count - 1));
        int y0 = top + (int)lrint((ymax - data[i - 1]) / (ymax - ymin) * plot_h);
        int y1 = top + (int)lrint((ymax - data[i]) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, x0, y0, x1, y1, r, g, b);
        if (thick)
            draw_line(img, w, h, x0, y0 + 1, x1, y1 + 1, r, g, b);
    }
}

static int write_plot_png(const char *path, const double *raw,
                          const double *corr_db,
                          const double *comp, int count,
                          double ymin, double ymax)
{
    const int w = 1100;
    const int h = 640;
    const int left = 70;
    const int right = 28;
    const int top = 32;
    const int bottom = 54;
    int plot_w = w - left - right;
    int plot_h = h - top - bottom;
    unsigned char *img = malloc((size_t)w * (size_t)h * 3U);

    if (!img)
        return -1;
    memset(img, 255, (size_t)w * (size_t)h * 3U);
    draw_axes(img, w, h, left, top, plot_w, plot_h, ymin, ymax);
    draw_curve(img, w, h, raw, count, left, top, plot_w, plot_h,
               ymin, ymax, 55, 105, 185, 0);
    draw_curve(img, w, h, corr_db, count, left, top, plot_w, plot_h,
               ymin, ymax, 18, 128, 104, 0);
    draw_curve(img, w, h, comp, count, left, top, plot_w, plot_h,
               ymin, ymax, 200, 85, 30, 1);

    if (write_png_rgb(path, img, w, h) != 0) {
        free(img);
        return -1;
    }
    free(img);
    return 0;
}

static void minmax_three(const double *a, const double *b, const double *c,
                         int count, double *out_min, double *out_max)
{
    double ymin = a[0];
    double ymax = a[0];

    for (int i = 0; i < count; i++) {
        if (a[i] < ymin) ymin = a[i];
        if (a[i] > ymax) ymax = a[i];
        if (b[i] < ymin) ymin = b[i];
        if (b[i] > ymax) ymax = b[i];
        if (c[i] < ymin) ymin = c[i];
        if (c[i] > ymax) ymax = c[i];
    }
    if (ymax - ymin < 1.0) {
        ymax += 0.5;
        ymin -= 0.5;
    } else {
        double pad = 0.12 * (ymax - ymin);
        ymax += pad;
        ymin -= pad;
    }
    *out_min = ymin;
    *out_max = ymax;
}

static int make_output_path(char *buf, size_t len, const comp_config_t *cfg,
                            int bw_tenths)
{
    char bw_tag[8];
    int written;

    if (bw_tenths >= 10)
        snprintf(bw_tag, sizeof(bw_tag), "1p0");
    else
        snprintf(bw_tag, sizeof(bw_tag), "0p%d", bw_tenths);

    written = snprintf(buf, len, "%s/%s_bw_%s.png",
                       cfg->out_dir, cfg->out_prefix, bw_tag);
    return written > 0 && (size_t)written < len ? 0 : -1;
}

static int write_bw_outputs(const comp_config_t *cfg,
                            const correction_table_t *corr,
                            const double *spectrum)
{
    for (int bw_tenths = 1; bw_tenths <= 10; bw_tenths++) {
        double bw = (double)bw_tenths / 10.0;
        int bins = (int)lrint((double)FFT_SIZE * bw);
        int start;
        double *raw_db;
        double *corr_db;
        double *comp_db;
        double ymin;
        double ymax;
        char path[1024];

        if (bins < 1)
            bins = 1;
        if (bins > (int)FFT_SIZE)
            bins = (int)FFT_SIZE;
        start = ((int)FFT_SIZE - bins) / 2;

        raw_db = malloc((size_t)bins * sizeof(double));
        corr_db = malloc((size_t)bins * sizeof(double));
        comp_db = malloc((size_t)bins * sizeof(double));
        if (!raw_db || !corr_db || !comp_db) {
            free(raw_db);
            free(corr_db);
            free(comp_db);
            return -1;
        }

        for (int i = 0; i < bins; i++) {
            int shifted_bin = start + i;
            double mag = spectrum[shifted_bin];
            double correction = correction_for_shifted_bin(corr, shifted_bin,
                                                           cfg->samplerate);
            double cmag = mag * correction;
            raw_db[i] = 20.0 * log10(mag + 1.0e-20);
            corr_db[i] = 20.0 * log10(correction + 1.0e-20);
            comp_db[i] = 20.0 * log10(cmag + 1.0e-20);
        }

        minmax_three(raw_db, corr_db, comp_db, bins, &ymin, &ymax);

        if (make_output_path(path, sizeof(path), cfg, bw_tenths) != 0 ||
            write_plot_png(path, raw_db, corr_db, comp_db, bins,
                           ymin, ymax) != 0) {
            fprintf(stderr, "Could not write PNG for BW %.1f\n", bw);
            free(raw_db);
            free(corr_db);
            free(comp_db);
            return -1;
        }
        printf("[COMP] Wrote %s\n", path);

        printf("[COMP] BW %.1f: bins %d, shifted bins %d..%d\n",
               bw, bins, start, start + bins - 1);
        free(raw_db);
        free(corr_db);
        free(comp_db);
    }

    return 0;
}

int main(int argc, char **argv)
{
    comp_config_t cfg;
    correction_table_t corr;
    struct fobos_sdr_dev_t *dev = NULL;
    char lib_version[FOBOS_INFO_LEN] = {0};
    char drv_version[FOBOS_INFO_LEN] = {0};
    char hw_revision[FOBOS_INFO_LEN] = {0};
    char fw_version[FOBOS_INFO_LEN] = {0};
    char manufacturer[FOBOS_INFO_LEN] = {0};
    char product[FOBOS_INFO_LEN] = {0};
    char serial[FOBOS_INFO_LEN] = {0};
    float window[FFT_SIZE];
    double spectrum[FFT_SIZE];
    uint64_t fft_count = 0;
    int ret;
    int exit_code = 1;

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return 2;
    }

    if (ensure_dir(cfg.out_dir) != 0) {
        fprintf(stderr, "Could not create output directory: %s\n", cfg.out_dir);
        return 1;
    }

    if (load_correction_table(cfg.fq_response_path, &corr) != 0)
        return 1;
    printf("[COMP] Loaded %zu correction rows from %s, %.3f MHz calibration rate\n",
           corr.count, cfg.fq_response_path, corr.samplerate_hz / 1.0e6);

    init_window(window);

    fobos_sdr_get_api_info(lib_version, drv_version);
    printf("[COMP] API: %s (drv: %s)\n", lib_version, drv_version);

    ret = fobos_sdr_get_device_count();
    if (ret <= 0) {
        fprintf(stderr, "[COMP] No Fobos SDR devices found.\n");
        goto done;
    }
    printf("[COMP] Devices found: %d\n", ret);

    ret = fobos_sdr_open(&dev, 0);
    if (check_ret("fobos_sdr_open", ret) != 0)
        goto done;

    ret = fobos_sdr_get_board_info(dev, hw_revision, fw_version,
                                   manufacturer, product, serial);
    if (ret == FOBOS_ERR_OK) {
        printf("[COMP] Device: %s %s\n", manufacturer, product);
        printf("[COMP]   HW: %s  FW: %s  S/N: %s\n",
               hw_revision, fw_version, serial);
    }

    if (check_ret("fobos_sdr_set_clk_source",
                  fobos_sdr_set_clk_source(dev, cfg.clk_source)) != 0)
        goto done;
    if (check_ret("fobos_sdr_set_direct_sampling",
                  fobos_sdr_set_direct_sampling(dev, cfg.direct_sampling)) != 0)
        goto done;
    if (!cfg.direct_sampling) {
        if (check_ret("fobos_sdr_set_frequency",
                      fobos_sdr_set_frequency(dev, cfg.frequency_hz)) != 0)
            goto done;
        if (check_ret("fobos_sdr_set_lna_gain",
                      fobos_sdr_set_lna_gain(dev, cfg.lna_gain)) != 0)
            goto done;
        if (check_ret("fobos_sdr_set_vga_gain",
                      fobos_sdr_set_vga_gain(dev, cfg.vga_gain)) != 0)
            goto done;
    }
    if (check_ret("fobos_sdr_set_samplerate",
                  fobos_sdr_set_samplerate(dev, cfg.samplerate)) != 0)
        goto done;
    if (check_ret("fobos_sdr_set_auto_bandwidth",
                  fobos_sdr_set_auto_bandwidth(dev, HARDWARE_AUTO_BW)) != 0)
        goto done;

    printf("[COMP] Capturing %.3f MHz, samplerate %.3f MHz, %u buffers, FFT %u\n",
           cfg.frequency_hz / 1.0e6, cfg.samplerate / 1.0e6,
           cfg.buffers, FFT_SIZE);

    if (capture_spectrum(dev, &cfg, window, spectrum, &fft_count) != 0)
        goto done;
    printf("[COMP] Averaged %llu FFT blocks\n",
           (unsigned long long)fft_count);

    if (write_bw_outputs(&cfg, &corr, spectrum) != 0)
        goto done;

    exit_code = 0;

done:
    if (dev)
        fobos_sdr_close(dev);
    free(corr.correction);
    return exit_code;
}
