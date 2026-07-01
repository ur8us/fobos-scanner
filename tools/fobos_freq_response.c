/*
 * Fobos SDR frequency-response calibration utility.
 *
 * With antennas disconnected, this program measures the receiver's noise-floor
 * shape across one complex-sampled passband. It repeats the measurement at
 * several receiver center frequencies, robust-averages the spectra, smooths the
 * result, optionally mirror-averages it around DC, and writes both a
 * machine-readable text table and a dependency-free PNG plot.
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
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEFAULT_SAMPLERATE      50000000.0
#define DEFAULT_BW_RATIO        1.0
#define DEFAULT_FFT_SIZE        65536U
#define DEFAULT_BUFFERS         128U
#define DEFAULT_PASSES          3U
#define DEFAULT_SETTLE_BUFFERS  8U
#define DEFAULT_LNA_GAIN        2U
#define DEFAULT_VGA_GAIN        15U
#define DEFAULT_DIRECT          0U
#define DEFAULT_CLK_SOURCE      0
#define DEFAULT_SMOOTH_HZ       3000000.0
#define DEFAULT_DESPUR_HZ       5000000.0
#define DEFAULT_PEAK_CLAMP_DB   0.25
#define DEFAULT_OUT_PREFIX      "fq_response"
#define DEFAULT_CENTER_LIST     "100,125,150,175,200,225,250,275,300,325,350"
#define MIN_SYNC_BUF_LEN        8192U
#define LNA_GAIN_MAX            3U
#define VGA_GAIN_MAX            31U

typedef struct {
    double *centers_hz;
    int center_count;
    double samplerate;
    double bw_ratio;
    double smooth_hz;
    double despur_hz;
    double peak_clamp_db;
    uint32_t fft_size;
    uint32_t buffers;
    uint32_t passes;
    uint32_t settle_buffers;
    uint32_t buf_len;
    unsigned int lna_gain;
    unsigned int vga_gain;
    unsigned int direct_sampling;
    int clk_source;
    int assume_yes;
    int symmetry;
    const char *out_prefix;
} response_config_t;

typedef struct {
    uint64_t buffers_processed;
    uint64_t short_buffers;
    uint64_t read_errors;
    double elapsed_s;
} capture_stats_t;

static long long now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + (long long)ts.tv_nsec;
}

static void sleep_ms(long ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static int is_power_of_two(uint32_t value)
{
    return value != 0 && (value & (value - 1U)) == 0;
}

static uint32_t round_buf_len(uint32_t value)
{
    uint32_t rem;
    if (value < MIN_SYNC_BUF_LEN)
        value = MIN_SYNC_BUF_LEN;
    rem = value % MIN_SYNC_BUF_LEN;
    if (rem)
        value += MIN_SYNC_BUF_LEN - rem;
    return value;
}

static void print_usage(const char *argv0)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "Measures receiver passband response with antennas disconnected.\n"
        "\n"
        "Options:\n"
        "  --centers-mhz LIST      comma-separated LO centers in MHz (default %s)\n"
        "  -r, --samplerate HZ     sample rate, suffixes k/m/g accepted (default %.0f)\n"
        "      --bw-ratio R        hardware auto bandwidth ratio (default %.2f)\n"
        "      --fft-size N        FFT size, power of two (default %u)\n"
        "      --buffers N         FFT buffers averaged per capture (default %u)\n"
        "      --passes N          full passes through the center list (default %u)\n"
        "      --settle-buffers N  sync buffers discarded after each retune (default %u)\n"
        "      --smooth-khz KHz    smoothing width (default %.0f kHz)\n"
        "      --despur-khz KHz    peak-removal baseline width (default %.0f kHz)\n"
        "      --peak-clamp-db DB  clamp peaks above baseline by this much (default %.2f)\n"
        "      --lna N             LNA gain 0..3 (default %u)\n"
        "      --vga N             VGA gain 0..31 (default %u)\n"
        "      --direct N          direct sampling mode 0/1/2 (default %u)\n"
        "      --clock internal|external|0|1\n"
        "      --out-prefix NAME   output prefix (default %s)\n"
        "      --no-symmetry       do not mirror-average around DC\n"
        "  -y, --yes              do not ask for antenna-disconnect confirmation\n"
        "  -h, --help             show this help\n"
        "\n"
        "Outputs:\n"
        "  PREFIX.txt              machine-readable response/correction table\n"
        "  PREFIX.png              PNG plot with axes and tick labels\n",
        argv0, DEFAULT_CENTER_LIST, DEFAULT_SAMPLERATE, DEFAULT_BW_RATIO,
        DEFAULT_FFT_SIZE, DEFAULT_BUFFERS, DEFAULT_PASSES, DEFAULT_SETTLE_BUFFERS,
        DEFAULT_SMOOTH_HZ / 1000.0, DEFAULT_DESPUR_HZ / 1000.0,
        DEFAULT_PEAK_CLAMP_DB, DEFAULT_LNA_GAIN, DEFAULT_VGA_GAIN,
        DEFAULT_DIRECT, DEFAULT_OUT_PREFIX);
}

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
    if (errno || end == text || value > 0xffffffffUL)
        return -1;
    while (*end && isspace((unsigned char)*end))
        end++;
    if (*end)
        return -1;
    *out = (uint32_t)value;
    return 0;
}

static int parse_clock_source(const char *text, int *out)
{
    if (strcmp(text, "0") == 0 || strcmp(text, "internal") == 0) {
        *out = 0;
        return 0;
    }
    if (strcmp(text, "1") == 0 || strcmp(text, "external") == 0 ||
        strcmp(text, "ext") == 0) {
        *out = 1;
        return 0;
    }
    return -1;
}

static int parse_center_list(const char *text, double **out_values, int *out_count)
{
    char *copy;
    char *token;
    char *save = NULL;
    double *values = NULL;
    int count = 0;
    int capacity = 0;

    copy = strdup(text);
    if (!copy)
        return -1;

    for (token = strtok_r(copy, ",", &save); token; token = strtok_r(NULL, ",", &save)) {
        double mhz;
        double *new_values;
        while (*token && isspace((unsigned char)*token))
            token++;
        if (parse_double_value(token, &mhz) != 0 || mhz <= 0.0) {
            free(values);
            free(copy);
            return -1;
        }
        if (count == capacity) {
            capacity = capacity ? capacity * 2 : 8;
            new_values = realloc(values, (size_t)capacity * sizeof(double));
            if (!new_values) {
                free(values);
                free(copy);
                return -1;
            }
            values = new_values;
        }
        values[count++] = mhz * 1.0e6;
    }

    free(copy);
    if (count <= 0) {
        free(values);
        return -1;
    }
    *out_values = values;
    *out_count = count;
    return 0;
}

static int parse_args(int argc, char **argv, response_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->samplerate = DEFAULT_SAMPLERATE;
    cfg->bw_ratio = DEFAULT_BW_RATIO;
    cfg->smooth_hz = DEFAULT_SMOOTH_HZ;
    cfg->despur_hz = DEFAULT_DESPUR_HZ;
    cfg->peak_clamp_db = DEFAULT_PEAK_CLAMP_DB;
    cfg->fft_size = DEFAULT_FFT_SIZE;
    cfg->buffers = DEFAULT_BUFFERS;
    cfg->passes = DEFAULT_PASSES;
    cfg->settle_buffers = DEFAULT_SETTLE_BUFFERS;
    cfg->buf_len = DEFAULT_FFT_SIZE;
    cfg->lna_gain = DEFAULT_LNA_GAIN;
    cfg->vga_gain = DEFAULT_VGA_GAIN;
    cfg->direct_sampling = DEFAULT_DIRECT;
    cfg->clk_source = DEFAULT_CLK_SOURCE;
    cfg->out_prefix = DEFAULT_OUT_PREFIX;
    cfg->symmetry = 1;

    if (parse_center_list(DEFAULT_CENTER_LIST, &cfg->centers_hz, &cfg->center_count) != 0)
        return -1;

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *value = NULL;
        double dtmp;
        uint32_t utmp;

        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(argv[0]);
            exit(0);
        }

        if (strcmp(arg, "--no-symmetry") == 0) {
            cfg->symmetry = 0;
            continue;
        }
        if (strcmp(arg, "-y") == 0 || strcmp(arg, "--yes") == 0) {
            cfg->assume_yes = 1;
            continue;
        }

        if (strcmp(arg, "--centers-mhz") == 0 || strcmp(arg, "-r") == 0 ||
            strcmp(arg, "--samplerate") == 0 || strcmp(arg, "--bw-ratio") == 0 ||
            strcmp(arg, "--fft-size") == 0 || strcmp(arg, "--buffers") == 0 ||
            strcmp(arg, "--passes") == 0 || strcmp(arg, "--settle-buffers") == 0 ||
            strcmp(arg, "--smooth-khz") == 0 || strcmp(arg, "--despur-khz") == 0 ||
            strcmp(arg, "--peak-clamp-db") == 0 ||
            strcmp(arg, "--lna") == 0 || strcmp(arg, "--vga") == 0 ||
            strcmp(arg, "--direct") == 0 || strcmp(arg, "--clock") == 0 ||
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

        if (strcmp(arg, "--centers-mhz") == 0) {
            double *centers = NULL;
            int center_count = 0;
            if (parse_center_list(value, &centers, &center_count) != 0)
                return -1;
            free(cfg->centers_hz);
            cfg->centers_hz = centers;
            cfg->center_count = center_count;
        } else if (strcmp(arg, "-r") == 0 || strcmp(arg, "--samplerate") == 0) {
            if (parse_hz_value(value, &dtmp) != 0 || dtmp <= 0.0)
                return -1;
            cfg->samplerate = dtmp;
        } else if (strcmp(arg, "--bw-ratio") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp <= 0.0 || dtmp > 1.0)
                return -1;
            cfg->bw_ratio = dtmp;
        } else if (strcmp(arg, "--fft-size") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || !is_power_of_two(utmp) ||
                utmp < 1024U || utmp > 1048576U)
                return -1;
            cfg->fft_size = utmp;
            cfg->buf_len = utmp;
        } else if (strcmp(arg, "--buffers") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp == 0)
                return -1;
            cfg->buffers = utmp;
        } else if (strcmp(arg, "--passes") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp == 0)
                return -1;
            cfg->passes = utmp;
        } else if (strcmp(arg, "--settle-buffers") == 0) {
            if (parse_uint_value(value, &utmp) != 0)
                return -1;
            cfg->settle_buffers = utmp;
        } else if (strcmp(arg, "--smooth-khz") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp < 0.0)
                return -1;
            cfg->smooth_hz = dtmp * 1000.0;
        } else if (strcmp(arg, "--despur-khz") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp < 0.0)
                return -1;
            cfg->despur_hz = dtmp * 1000.0;
        } else if (strcmp(arg, "--peak-clamp-db") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp < 0.0)
                return -1;
            cfg->peak_clamp_db = dtmp;
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
            if (parse_clock_source(value, &cfg->clk_source) != 0)
                return -1;
        } else if (strcmp(arg, "--out-prefix") == 0) {
            cfg->out_prefix = value;
        }
    }

    cfg->buf_len = round_buf_len(cfg->buf_len);
    if (cfg->buf_len < cfg->fft_size)
        cfg->buf_len = round_buf_len(cfg->fft_size);
    return 0;
}

static int check_ret(const char *name, int ret)
{
    if (ret != FOBOS_ERR_OK) {
        fprintf(stderr, "%s failed: %d\n", name, ret);
        return -1;
    }
    return 0;
}

static void fft_c2c(float *data, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = data[2*i], ti = data[2*i+1];
            data[2*i] = data[2*j];
            data[2*i+1] = data[2*j+1];
            data[2*j] = tr;
            data[2*j+1] = ti;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = 2.0f * (float)M_PI / (float)len;
        float w_re = cosf(ang), w_im = -sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cur_re = 1.0f, cur_im = 0.0f;
            for (int j = 0; j < len / 2; j++) {
                int i0 = i + j, i1 = i0 + len / 2;
                float re1 = data[2*i0], im1 = data[2*i0+1];
                float re2 = data[2*i1] * cur_re - data[2*i1+1] * cur_im;
                float im2 = data[2*i1] * cur_im + data[2*i1+1] * cur_re;
                float nr;
                data[2*i0] = re1 + re2;
                data[2*i0+1] = im1 + im2;
                data[2*i1] = re1 - re2;
                data[2*i1+1] = im1 - im2;
                nr = cur_re * w_re - cur_im * w_im;
                cur_im = cur_re * w_im + cur_im * w_re;
                cur_re = nr;
            }
        }
    }
}

static void init_window(float *window, uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(n - 1U)));
}

static int compare_double(const void *a, const void *b)
{
    double da = *(const double *)a;
    double db = *(const double *)b;
    return (da > db) - (da < db);
}

static double median_value(const double *values, uint32_t count)
{
    double *tmp;
    double result;
    if (count == 0)
        return 0.0;
    tmp = malloc((size_t)count * sizeof(double));
    if (!tmp)
        return 0.0;
    memcpy(tmp, values, (size_t)count * sizeof(double));
    qsort(tmp, count, sizeof(double), compare_double);
    if (count & 1U)
        result = tmp[count / 2U];
    else
        result = 0.5 * (tmp[count / 2U - 1U] + tmp[count / 2U]);
    free(tmp);
    return result;
}

static double robust_mean(double *values, int count)
{
    double sum = 0.0;
    int start = 0;
    int end = count;
    if (count <= 0)
        return 0.0;
    qsort(values, (size_t)count, sizeof(double), compare_double);
    if (count >= 3) {
        start = 1;
        end = count - 1;
    }
    for (int i = start; i < end; i++)
        sum += values[i];
    return sum / (double)(end - start);
}

static void process_fft_buffer(const float *buf, uint32_t actual_len,
                               uint32_t fft_size, const float *window,
                               float *work, double *power_accum)
{
    if (actual_len < fft_size)
        return;
    for (uint32_t i = 0; i < fft_size; i++) {
        float w = window[i];
        work[2U*i] = buf[2U*i] * w;
        work[2U*i + 1U] = buf[2U*i + 1U] * w;
    }
    fft_c2c(work, (int)fft_size);
    for (uint32_t i = 0; i < fft_size; i++) {
        uint32_t bin = (i + fft_size / 2U) % fft_size;
        double re = work[2U*bin];
        double im = work[2U*bin + 1U];
        power_accum[i] += re * re + im * im;
    }
}

static int capture_center(struct fobos_sdr_dev_t *dev,
                          const response_config_t *cfg,
                          double center_hz, const float *window,
                          double *out_db, capture_stats_t *stats)
{
    float *buf = NULL;
    float *work = NULL;
    double *power_accum = NULL;
    uint32_t processed = 0;
    long long start_ns;
    int ret;

    memset(stats, 0, sizeof(*stats));
    buf = malloc((size_t)cfg->buf_len * 2U * sizeof(float));
    work = malloc((size_t)cfg->fft_size * 2U * sizeof(float));
    power_accum = calloc((size_t)cfg->fft_size, sizeof(double));
    if (!buf || !work || !power_accum) {
        fprintf(stderr, "Out of memory while preparing capture buffers\n");
        ret = -1;
        goto done;
    }

    if (!cfg->direct_sampling) {
        ret = fobos_sdr_set_frequency(dev, center_hz);
        if (check_ret("fobos_sdr_set_frequency", ret) != 0)
            goto done;
    }
    sleep_ms(150);

    ret = fobos_sdr_start_sync(dev, cfg->buf_len);
    if (check_ret("fobos_sdr_start_sync", ret) != 0)
        goto done;

    start_ns = now_ns();
    for (uint32_t i = 0; i < cfg->settle_buffers; i++) {
        uint32_t actual = cfg->buf_len;
        ret = fobos_sdr_read_sync(dev, buf, &actual);
        if (ret != FOBOS_ERR_OK) {
            stats->read_errors++;
            break;
        }
    }

    while (processed < cfg->buffers) {
        uint32_t actual = cfg->buf_len;
        ret = fobos_sdr_read_sync(dev, buf, &actual);
        if (ret != FOBOS_ERR_OK) {
            stats->read_errors++;
            break;
        }
        if (actual < cfg->fft_size) {
            stats->short_buffers++;
            continue;
        }
        process_fft_buffer(buf, actual, cfg->fft_size, window, work, power_accum);
        processed++;
    }

    fobos_sdr_stop_sync(dev);
    stats->elapsed_s = (double)(now_ns() - start_ns) / 1.0e9;
    stats->buffers_processed = processed;
    if (processed == 0) {
        fprintf(stderr, "No usable buffers captured at %.3f MHz\n", center_hz / 1.0e6);
        ret = -1;
        goto done;
    }

    for (uint32_t i = 0; i < cfg->fft_size; i++)
        out_db[i] = 10.0 * log10(power_accum[i] / (double)processed + 1.0e-30);
    {
        double med = median_value(out_db, cfg->fft_size);
        for (uint32_t i = 0; i < cfg->fft_size; i++)
            out_db[i] -= med;
    }
    ret = 0;

done:
    free(buf);
    free(work);
    free(power_accum);
    return ret;
}

static void combine_centers(const double *center_db, int center_count,
                            uint32_t fft_size, double *raw_avg)
{
    double *scratch = malloc((size_t)center_count * sizeof(double));
    if (!scratch) {
        for (uint32_t i = 0; i < fft_size; i++)
            raw_avg[i] = 0.0;
        return;
    }
    for (uint32_t bin = 0; bin < fft_size; bin++) {
        for (int c = 0; c < center_count; c++)
            scratch[c] = center_db[(size_t)c * fft_size + bin];
        raw_avg[bin] = robust_mean(scratch, center_count);
    }
    free(scratch);
}

static double sampled_percentile(const double *in, uint32_t start, uint32_t end,
                                 double percentile, double *scratch,
                                 uint32_t max_samples)
{
    uint32_t len = end > start ? end - start : 0U;
    uint32_t stride;
    uint32_t count = 0;
    uint32_t index;

    if (len == 0)
        return 0.0;
    if (max_samples < 8U)
        max_samples = 8U;
    stride = (len + max_samples - 1U) / max_samples;
    if (stride < 1U)
        stride = 1U;

    for (uint32_t i = start; i < end; i += stride)
        scratch[count++] = in[i];
    if (count == 0)
        scratch[count++] = in[start];

    qsort(scratch, count, sizeof(double), compare_double);
    if (percentile < 0.0)
        percentile = 0.0;
    if (percentile > 1.0)
        percentile = 1.0;
    index = (uint32_t)lrint(percentile * (double)(count - 1U));
    if (index >= count)
        index = count - 1U;
    return scratch[index];
}

static void remove_response_peaks(const double *in, uint32_t n,
                                  double samplerate, double despur_hz,
                                  double peak_clamp_db, double *out)
{
    double bin_hz = samplerate / (double)n;
    uint32_t half_window;
    double *scratch;
    const uint32_t max_samples = 401U;

    if (despur_hz <= bin_hz || peak_clamp_db <= 0.0) {
        memcpy(out, in, (size_t)n * sizeof(double));
        return;
    }

    half_window = (uint32_t)lrint((despur_hz / bin_hz) * 0.5);
    if (half_window < 8U)
        half_window = 8U;
    if (half_window > n / 3U)
        half_window = n / 3U;

    scratch = malloc((size_t)max_samples * sizeof(double));
    if (!scratch) {
        memcpy(out, in, (size_t)n * sizeof(double));
        return;
    }

    for (uint32_t i = 0; i < n; i++) {
        uint32_t start = i > half_window ? i - half_window : 0U;
        uint32_t end = i + half_window + 1U;
        double baseline;
        double max_value;
        if (end > n)
            end = n;
        baseline = sampled_percentile(in, start, end, 0.30, scratch, max_samples);
        max_value = baseline + peak_clamp_db;
        out[i] = in[i] > max_value ? max_value : in[i];
    }

    free(scratch);
}

static void smooth_response(const double *in, uint32_t n, double smooth_hz,
                            double samplerate, double *out)
{
    double bin_hz = samplerate / (double)n;
    uint32_t half_window;
    double *prefix;

    if (smooth_hz <= bin_hz) {
        memcpy(out, in, (size_t)n * sizeof(double));
        return;
    }
    half_window = (uint32_t)lrint((smooth_hz / bin_hz) * 0.5);
    if (half_window < 1U)
        half_window = 1U;
    if (half_window > n / 4U)
        half_window = n / 4U;

    prefix = calloc((size_t)n + 1U, sizeof(double));
    if (!prefix) {
        memcpy(out, in, (size_t)n * sizeof(double));
        return;
    }
    for (uint32_t i = 0; i < n; i++)
        prefix[i + 1U] = prefix[i] + in[i];
    for (uint32_t i = 0; i < n; i++) {
        uint32_t start = i > half_window ? i - half_window : 0U;
        uint32_t end = i + half_window + 1U;
        if (end > n)
            end = n;
        out[i] = (prefix[end] - prefix[start]) / (double)(end - start);
    }
    free(prefix);
}

static void apply_symmetry(double *values, uint32_t n)
{
    for (uint32_t i = 0; i < n / 2U; i++) {
        uint32_t j = n - 1U - i;
        double avg = 0.5 * (values[i] + values[j]);
        values[i] = avg;
        values[j] = avg;
    }
}

static void normalize_mean(double *values, uint32_t n)
{
    double sum = 0.0;
    double mean;
    for (uint32_t i = 0; i < n; i++)
        sum += values[i];
    mean = sum / (double)n;
    for (uint32_t i = 0; i < n; i++)
        values[i] -= mean;
}

static int write_response_text(const char *path, const response_config_t *cfg,
                               const double *raw_avg, const double *despurred,
                               const double *smoothed, const capture_stats_t *stats,
                               int capture_count)
{
    FILE *f = fopen(path, "w");
    if (!f) {
        perror(path);
        return -1;
    }

    fprintf(f, "# Fobos SDR frequency response calibration\n");
    fprintf(f, "# samplerate_hz %.0f\n", cfg->samplerate);
    fprintf(f, "# fft_size %u\n", cfg->fft_size);
    fprintf(f, "# buffers_per_capture %u\n", cfg->buffers);
    fprintf(f, "# passes %u\n", cfg->passes);
    fprintf(f, "# total_captures %d\n", capture_count);
    fprintf(f, "# settle_buffers %u\n", cfg->settle_buffers);
    fprintf(f, "# bw_ratio %.6f\n", cfg->bw_ratio);
    fprintf(f, "# lna_gain %u\n", cfg->lna_gain);
    fprintf(f, "# vga_gain %u\n", cfg->vga_gain);
    fprintf(f, "# direct_sampling %u\n", cfg->direct_sampling);
    fprintf(f, "# despur_hz %.0f\n", cfg->despur_hz);
    fprintf(f, "# peak_clamp_db %.6f\n", cfg->peak_clamp_db);
    fprintf(f, "# smoothing_hz %.0f\n", cfg->smooth_hz);
    fprintf(f, "# correction_source response_db_smoothed\n");
    fprintf(f, "# symmetry %d\n", cfg->symmetry);
    fprintf(f, "# centers_mhz");
    for (int c = 0; c < cfg->center_count; c++)
        fprintf(f, " %.6f", cfg->centers_hz[c] / 1.0e6);
    fprintf(f, "\n");
    fprintf(f, "# captured_buffers");
    for (int c = 0; c < capture_count; c++)
        fprintf(f, " %llu", (unsigned long long)stats[c].buffers_processed);
    fprintf(f, "\n");
    fprintf(f, "# columns: bin offset_hz offset_mhz response_db correction_db correction_linear raw_avg_db corrected_raw_db despurred_db\n");

    for (uint32_t i = 0; i < cfg->fft_size; i++) {
        double offset_hz = ((double)i - (double)cfg->fft_size * 0.5) *
            cfg->samplerate / (double)cfg->fft_size;
        double response_db = smoothed[i];
        double correction_db = -response_db;
        double correction_linear = pow(10.0, correction_db / 20.0);
        double corrected_raw_db = raw_avg[i] + correction_db;
        fprintf(f, "%u %.6f %.9f %.6f %.6f %.9f %.6f %.6f %.6f\n",
                i, offset_hz, offset_hz / 1.0e6,
                response_db, correction_db, correction_linear, raw_avg[i],
                corrected_raw_db, despurred[i]);
    }

    fclose(f);
    return 0;
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

static void draw_rect(unsigned char *img, int w, int h, int x, int y,
                      int rw, int rh,
                      unsigned char r, unsigned char g, unsigned char b)
{
    for (int yy = y; yy < y + rh; yy++) {
        for (int xx = x; xx < x + rw; xx++)
            put_pixel(img, w, h, xx, yy, r, g, b);
    }
}

static const char *font_pattern(char ch)
{
    switch (ch) {
    case '0': return "111101101101101101111";
    case '1': return "010110010010010010111";
    case '2': return "111001001111100100111";
    case '3': return "111001001111001001111";
    case '4': return "101101101111001001001";
    case '5': return "111100100111001001111";
    case '6': return "111100100111101101111";
    case '7': return "111001001010010010010";
    case '8': return "111101101111101101111";
    case '9': return "111101101111001001111";
    case '-': return "000000000111000000000";
    case '.': return "000000000000000010010";
    case 'M': return "101111111101101101101";
    case 'A': return "010101101111101101101";
    case 'C': return "111100100100100100111";
    case 'E': return "111100100111100100111";
    case 'H': return "101101101111101101101";
    case 'O': return "111101101101101101111";
    case 'R': return "110101101110101101101";
    case 'S': return "111100100111001001111";
    case 'T': return "111010010010010010010";
    case 'z': case 'Z': return "111001010010010100111";
    case 'd': case 'D': return "110101101101101101110";
    case 'B': return "110101101110101101110";
    case ' ': return "000000000000000000000";
    default: return "000000000000000000000";
    }
}

static int text_width(const char *text, int scale)
{
    return (int)strlen(text) * 4 * scale;
}

static void draw_text(unsigned char *img, int w, int h, int x, int y,
                      const char *text, int scale,
                      unsigned char r, unsigned char g, unsigned char b)
{
    int cursor = x;
    for (const char *p = text; *p; p++) {
        const char *pat = font_pattern(*p);
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 3; col++) {
                if (pat[row * 3 + col] == '1')
                    draw_rect(img, w, h, cursor + col * scale, y + row * scale,
                              scale, scale, r, g, b);
            }
        }
        cursor += 4 * scale;
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
    for (size_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return crc;
}

static uint32_t adler32_buf(const unsigned char *buf, size_t len)
{
    uint32_t a = 1;
    uint32_t b = 0;
    for (size_t i = 0; i < len; i++) {
        a = (a + buf[i]) % 65521U;
        b = (b + a) % 65521U;
    }
    return (b << 16) | a;
}

static int write_chunk(FILE *f, const char type[4],
                       const unsigned char *data, uint32_t len)
{
    uint32_t crc;
    write_be32(f, len);
    fwrite(type, 1, 4, f);
    if (len > 0 && fwrite(data, 1, len, f) != len)
        return -1;
    crc = 0xffffffffU;
    crc = crc32_step(crc, (const unsigned char *)type, 4);
    if (len > 0)
        crc = crc32_step(crc, data, len);
    write_be32(f, ~crc);
    return ferror(f) ? -1 : 0;
}

static int write_png_rgb(const char *path, const unsigned char *rgb, int w, int h)
{
    FILE *f;
    unsigned char ihdr[13];
    unsigned char *raw = NULL;
    unsigned char *zdata = NULL;
    size_t row_bytes = (size_t)w * 3U;
    size_t raw_len = (row_bytes + 1U) * (size_t)h;
    size_t max_blocks = raw_len / 65535U + 1U;
    size_t zcap = 2U + raw_len + max_blocks * 5U + 4U;
    size_t pos = 0;
    size_t zpos = 0;
    uint32_t adler;
    static const unsigned char sig[8] = {137, 80, 78, 71, 13, 10, 26, 10};

    raw = malloc(raw_len);
    zdata = malloc(zcap);
    if (!raw || !zdata) {
        free(raw);
        free(zdata);
        return -1;
    }
    for (int y = 0; y < h; y++) {
        raw[(size_t)y * (row_bytes + 1U)] = 0;
        memcpy(raw + (size_t)y * (row_bytes + 1U) + 1U,
               rgb + (size_t)y * row_bytes, row_bytes);
    }

    zdata[zpos++] = 0x78;
    zdata[zpos++] = 0x01;
    while (pos < raw_len) {
        size_t remain = raw_len - pos;
        uint16_t block_len = remain > 65535U ? 65535U : (uint16_t)remain;
        int final = (pos + block_len == raw_len);
        zdata[zpos++] = (unsigned char)(final ? 1 : 0);
        zdata[zpos++] = (unsigned char)(block_len & 0xffU);
        zdata[zpos++] = (unsigned char)((block_len >> 8) & 0xffU);
        zdata[zpos++] = (unsigned char)((~block_len) & 0xffU);
        zdata[zpos++] = (unsigned char)(((~block_len) >> 8) & 0xffU);
        memcpy(zdata + zpos, raw + pos, block_len);
        zpos += block_len;
        pos += block_len;
    }
    adler = adler32_buf(raw, raw_len);
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

static int write_response_png(const char *path, const response_config_t *cfg,
                              const double *raw_avg, const double *response)
{
    const int w = 1200;
    const int h = 720;
    const int left = 88;
    const int right = 35;
    const int top = 36;
    const int bottom = 78;
    int plot_w = w - left - right;
    int plot_h = h - top - bottom;
    unsigned char *img;
    double ymin = response[0], ymax = response[0];
    double ystep;
    double ytick_start;
    double x_min_mhz = -cfg->samplerate * 0.5 / 1.0e6;
    double x_max_mhz = cfg->samplerate * 0.5 / 1.0e6;
    double xstep = nice_step(x_max_mhz - x_min_mhz, 10);

    img = malloc((size_t)w * (size_t)h * 3U);
    if (!img)
        return -1;
    memset(img, 255, (size_t)w * (size_t)h * 3U);

    for (uint32_t i = 0; i < cfg->fft_size; i++) {
        double corrected = raw_avg[i] - response[i];
        if (raw_avg[i] < ymin)
            ymin = raw_avg[i];
        if (raw_avg[i] > ymax)
            ymax = raw_avg[i];
        if (response[i] < ymin)
            ymin = response[i];
        if (response[i] > ymax)
            ymax = response[i];
        if (corrected < ymin)
            ymin = corrected;
        if (corrected > ymax)
            ymax = corrected;
    }
    if (ymax - ymin < 1.0) {
        ymax += 0.5;
        ymin -= 0.5;
    } else {
        double pad = 0.12 * (ymax - ymin);
        ymax += pad;
        ymin -= pad;
    }
    ystep = nice_step(ymax - ymin, 8);
    ymin = floor(ymin / ystep) * ystep;
    ymax = ceil(ymax / ystep) * ystep;
    ytick_start = ceil(ymin / ystep) * ystep;

    for (double yv = ytick_start; yv <= ymax + 1.0e-9; yv += ystep) {
        int y = top + (int)lrint((ymax - yv) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, left, y, left + plot_w, y, 226, 232, 238);
        draw_line(img, w, h, left - 5, y, left, y, 20, 35, 48);
        {
            char label[32];
            snprintf(label, sizeof(label), "%.0f", yv);
            draw_text(img, w, h, left - 12 - text_width(label, 2), y - 7,
                      label, 2, 20, 35, 48);
        }
    }
    for (double xv = ceil(x_min_mhz / xstep) * xstep;
         xv <= x_max_mhz + 1.0e-9; xv += xstep) {
        int x = left + (int)lrint((xv - x_min_mhz) / (x_max_mhz - x_min_mhz) * plot_w);
        draw_line(img, w, h, x, top, x, top + plot_h, 226, 232, 238);
        draw_line(img, w, h, x, top + plot_h, x, top + plot_h + 5, 20, 35, 48);
        {
            char label[32];
            snprintf(label, sizeof(label), "%.0f", xv);
            draw_text(img, w, h, x - text_width(label, 2) / 2, top + plot_h + 12,
                      label, 2, 20, 35, 48);
        }
    }

    draw_line(img, w, h, left, top, left, top + plot_h, 20, 35, 48);
    draw_line(img, w, h, left, top + plot_h, left + plot_w, top + plot_h, 20, 35, 48);
    draw_text(img, w, h, left + plot_w / 2 - text_width("MHz", 2) / 2,
              h - 30, "MHz", 2, 20, 35, 48);
    draw_text(img, w, h, 18, top + 4, "dB", 2, 20, 35, 48);

    for (uint32_t i = 1; i < cfg->fft_size; i++) {
        int x0 = left + (int)(((uint64_t)(i - 1U) * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int x1 = left + (int)(((uint64_t)i * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int y0 = top + (int)lrint((ymax - raw_avg[i - 1U]) / (ymax - ymin) * plot_h);
        int y1 = top + (int)lrint((ymax - raw_avg[i]) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, x0, y0, x1, y1, 190, 198, 208);
    }
    for (uint32_t i = 1; i < cfg->fft_size; i++) {
        double c0 = raw_avg[i - 1U] - response[i - 1U];
        double c1 = raw_avg[i] - response[i];
        int x0 = left + (int)(((uint64_t)(i - 1U) * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int x1 = left + (int)(((uint64_t)i * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int y0 = top + (int)lrint((ymax - c0) / (ymax - ymin) * plot_h);
        int y1 = top + (int)lrint((ymax - c1) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, x0, y0, x1, y1, 218, 122, 35);
    }
    for (uint32_t i = 1; i < cfg->fft_size; i++) {
        int x0 = left + (int)(((uint64_t)(i - 1U) * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int x1 = left + (int)(((uint64_t)i * (uint64_t)plot_w) / (uint64_t)(cfg->fft_size - 1U));
        int y0 = top + (int)lrint((ymax - response[i - 1U]) / (ymax - ymin) * plot_h);
        int y1 = top + (int)lrint((ymax - response[i]) / (ymax - ymin) * plot_h);
        draw_line(img, w, h, x0, y0, x1, y1, 18, 128, 104);
        draw_line(img, w, h, x0, y0 + 1, x1, y1 + 1, 18, 128, 104);
    }
    {
        const int lx = left + plot_w - 248;
        const int ly = top + 12;
        draw_rect(img, w, h, lx, ly + 3, 26, 5, 190, 198, 208);
        draw_text(img, w, h, lx + 34, ly, "MEAS", 2, 80, 88, 98);
        draw_rect(img, w, h, lx + 92, ly + 3, 26, 5, 218, 122, 35);
        draw_text(img, w, h, lx + 126, ly, "CORR", 2, 120, 68, 24);
        draw_rect(img, w, h, lx + 184, ly + 3, 26, 5, 18, 128, 104);
        draw_text(img, w, h, lx + 218, ly, "SMOOTH", 2, 18, 128, 104);
    }

    if (write_png_rgb(path, img, w, h) != 0) {
        free(img);
        return -1;
    }
    free(img);
    return 0;
}

static char *make_output_path(const char *prefix, const char *suffix)
{
    size_t len = strlen(prefix) + strlen(suffix) + 1U;
    char *path = malloc(len);
    if (!path)
        return NULL;
    snprintf(path, len, "%s%s", prefix, suffix);
    return path;
}

int main(int argc, char **argv)
{
    response_config_t cfg;
    struct fobos_sdr_dev_t *dev = NULL;
    char lib_version[FOBOS_INFO_LEN] = {0};
    char drv_version[FOBOS_INFO_LEN] = {0};
    char hw_revision[FOBOS_INFO_LEN] = {0};
    char fw_version[FOBOS_INFO_LEN] = {0};
    char manufacturer[FOBOS_INFO_LEN] = {0};
    char product[FOBOS_INFO_LEN] = {0};
    char serial[FOBOS_INFO_LEN] = {0};
    float *window = NULL;
    double *center_db = NULL;
    double *raw_avg = NULL;
    double *despurred = NULL;
    double *final_tmp = NULL;
    double *smoothed = NULL;
    capture_stats_t *stats = NULL;
    char *txt_path = NULL;
    char *png_path = NULL;
    int ret;
    int exit_code = 1;
    int capture_count;

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return 2;
    }

    capture_count = cfg.center_count * (int)cfg.passes;

    if (!cfg.assume_yes) {
        char answer[16];
        printf("\nDisconnect all antennas and signal sources from the Fobos SDR inputs.\n");
        printf("This calibration uses receiver noise to estimate passband response.\n");
        printf("Press ENTER to continue, or Ctrl+C to abort.");
        fflush(stdout);
        if (!fgets(answer, sizeof(answer), stdin)) {
            free(cfg.centers_hz);
            return 2;
        }
    }

    window = malloc((size_t)cfg.fft_size * sizeof(float));
    center_db = malloc((size_t)capture_count * (size_t)cfg.fft_size * sizeof(double));
    raw_avg = malloc((size_t)cfg.fft_size * sizeof(double));
    despurred = malloc((size_t)cfg.fft_size * sizeof(double));
    final_tmp = malloc((size_t)cfg.fft_size * sizeof(double));
    smoothed = malloc((size_t)cfg.fft_size * sizeof(double));
    stats = calloc((size_t)capture_count, sizeof(capture_stats_t));
    txt_path = make_output_path(cfg.out_prefix, ".txt");
    png_path = make_output_path(cfg.out_prefix, ".png");
    if (!window || !center_db || !raw_avg || !despurred || !final_tmp || !smoothed ||
        !stats || !txt_path || !png_path) {
        fprintf(stderr, "Out of memory\n");
        goto done;
    }
    init_window(window, cfg.fft_size);

    fobos_sdr_get_api_info(lib_version, drv_version);
    printf("[FQ] API: %s (drv: %s)\n", lib_version, drv_version);

    ret = fobos_sdr_get_device_count();
    if (ret <= 0) {
        fprintf(stderr, "[FQ] No Fobos SDR devices found.\n");
        goto done;
    }
    printf("[FQ] Devices found: %d\n", ret);

    ret = fobos_sdr_open(&dev, 0);
    if (check_ret("fobos_sdr_open", ret) != 0)
        goto done;

    ret = fobos_sdr_get_board_info(dev, hw_revision, fw_version,
                                   manufacturer, product, serial);
    if (ret == FOBOS_ERR_OK) {
        printf("[FQ] Device: %s %s\n", manufacturer, product);
        printf("[FQ]   HW: %s  FW: %s  S/N: %s\n",
               hw_revision, fw_version, serial);
    }

    if (check_ret("fobos_sdr_set_clk_source",
                  fobos_sdr_set_clk_source(dev, cfg.clk_source)) != 0)
        goto done;
    if (check_ret("fobos_sdr_set_direct_sampling",
                  fobos_sdr_set_direct_sampling(dev, cfg.direct_sampling)) != 0)
        goto done;
    if (!cfg.direct_sampling) {
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
                  fobos_sdr_set_auto_bandwidth(dev, cfg.bw_ratio)) != 0)
        goto done;

    printf("[FQ] Measuring %.3f MHz bandwidth with %u-point FFT\n",
           cfg.samplerate / 1.0e6, cfg.fft_size);
    printf("[FQ] Captures: %u passes x %d centers = %d captures\n",
           cfg.passes, cfg.center_count, capture_count);
    printf("[FQ] Averaging %u buffers per capture, de-spur %.0f kHz clamp %.2f dB, smoothing %.0f kHz, symmetry %s\n",
           cfg.buffers, cfg.despur_hz / 1000.0, cfg.peak_clamp_db,
           cfg.smooth_hz / 1000.0, cfg.symmetry ? "on" : "off");

    for (uint32_t pass = 0; pass < cfg.passes; pass++) {
        for (int c = 0; c < cfg.center_count; c++) {
            int capture_index = (int)pass * cfg.center_count + c;
            double center_hz = cfg.centers_hz[c];
            double *dst = center_db + (size_t)capture_index * cfg.fft_size;
            if (cfg.direct_sampling)
                printf("[FQ] Capture %d/%d pass %u/%u: direct sampling\n",
                       capture_index + 1, capture_count, pass + 1U, cfg.passes);
            else
                printf("[FQ] Capture %d/%d pass %u/%u: center %.6f MHz\n",
                       capture_index + 1, capture_count, pass + 1U, cfg.passes,
                       center_hz / 1.0e6);
            if (capture_center(dev, &cfg, center_hz, window, dst,
                               &stats[capture_index]) != 0)
                goto done;
            printf("[FQ]   buffers %llu, short %llu, read errors %llu, elapsed %.3f s\n",
                   (unsigned long long)stats[capture_index].buffers_processed,
                   (unsigned long long)stats[capture_index].short_buffers,
                   (unsigned long long)stats[capture_index].read_errors,
                   stats[capture_index].elapsed_s);
        }
    }

    combine_centers(center_db, capture_count, cfg.fft_size, raw_avg);
    normalize_mean(raw_avg, cfg.fft_size);
    remove_response_peaks(raw_avg, cfg.fft_size, cfg.samplerate,
                          cfg.despur_hz, cfg.peak_clamp_db, despurred);
    normalize_mean(despurred, cfg.fft_size);
    smooth_response(despurred, cfg.fft_size, cfg.smooth_hz, cfg.samplerate, smoothed);
    if (cfg.symmetry)
        apply_symmetry(smoothed, cfg.fft_size);
    smooth_response(smoothed, cfg.fft_size, cfg.smooth_hz, cfg.samplerate, final_tmp);
    memcpy(smoothed, final_tmp, (size_t)cfg.fft_size * sizeof(double));
    normalize_mean(smoothed, cfg.fft_size);

    if (write_response_text(txt_path, &cfg, raw_avg, despurred, smoothed,
                            stats, capture_count) != 0)
        goto done;
    if (write_response_png(png_path, &cfg, raw_avg, smoothed) != 0) {
        fprintf(stderr, "Could not write PNG plot: %s\n", png_path);
        goto done;
    }

    printf("[FQ] Wrote %s\n", txt_path);
    printf("[FQ] Wrote %s\n", png_path);
    printf("[FQ] Use correction_db or correction_linear as the future inverse frequency-domain window.\n");
    exit_code = 0;

done:
    if (dev)
        fobos_sdr_close(dev);
    free(cfg.centers_hz);
    free(window);
    free(center_db);
    free(raw_avg);
    free(despurred);
    free(final_tmp);
    free(smoothed);
    free(stats);
    free(txt_path);
    free(png_path);
    return exit_code;
}
