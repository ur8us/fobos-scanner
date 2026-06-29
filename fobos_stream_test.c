/*
 * Fobos SDR stream integrity test.
 *
 * This is intentionally independent from the scanner web backend. It opens the
 * receiver, starts normal single-frequency async streaming, runs for a fixed
 * duration, and reports stream health from callback timing and sample content.
 */

#define _POSIX_C_SOURCE 200809L

#include <ctype.h>
#include <errno.h>
#include <fobos_sdr.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DEFAULT_FREQ_MHZ      100.0
#define DEFAULT_SAMPLERATE    50000000.0
#define DEFAULT_SECONDS       10.0
#define DEFAULT_BW_RATIO      0.9
#define DEFAULT_BUF_COUNT     16U
#define DEFAULT_BUF_LEN       65536U
#define DEFAULT_LNA_GAIN      0U
#define DEFAULT_VGA_GAIN      0U
#define DEFAULT_DIRECT        0U
#define DEFAULT_CLK_SOURCE    0

typedef struct {
    double frequency_hz;
    double samplerate;
    double seconds;
    double bw_ratio;
    uint32_t buf_count;
    uint32_t buf_len;
    unsigned int lna_gain;
    unsigned int vga_gain;
    unsigned int direct_sampling;
    int clk_source;
} test_config_t;

typedef struct {
    struct fobos_sdr_dev_t *dev;
    test_config_t cfg;
    volatile int stop_requested;

    long long start_ns;
    long long first_cb_ns;
    long long last_cb_ns;

    uint64_t callbacks;
    uint64_t total_samples;
    uint64_t expected_missing;
    uint64_t large_gaps;
    uint64_t len_changes;
    uint64_t short_buffers;
    uint64_t zero_buffers;
    uint64_t duplicate_signatures;
    uint64_t invalid_samples;
    uint64_t clipped_samples;
    uint64_t zero_iq_pairs;

    uint32_t first_buf_len;
    uint32_t min_buf_len;
    uint32_t max_buf_len;

    double sum_gap_ns;
    double sum_gap2_ns;
    uint64_t gap_count;
    long long min_gap_ns;
    long long max_gap_ns;

    double sum_re;
    double sum_im;
    double sum_power;
    double max_abs;
    double boundary_jump_power;
    double max_boundary_jump_power;
    uint64_t boundary_count;
    float last_re;
    float last_im;
    int have_last_sample;

    uint64_t previous_signature;
    int have_signature;
} stream_stats_t;

static long long now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + (long long)ts.tv_nsec;
}

static void print_usage(const char *argv0)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "Options:\n"
        "  -f, --freq-mhz MHz       RF frequency in MHz (default %.3f)\n"
        "      --freq-hz HZ         RF frequency in Hz\n"
        "  -r, --samplerate HZ      sample rate, suffixes k/m/g accepted (default %.0f)\n"
        "  -t, --seconds SEC        run duration (default %.1f)\n"
        "      --bw-ratio R         auto bandwidth ratio (default %.2f)\n"
        "      --buf-len N          complex samples per async buffer (default %u)\n"
        "      --buf-count N        USB async buffer count (default %u)\n"
        "      --lna N              LNA gain 0..2 (default %u)\n"
        "      --vga N              VGA gain 0..15 (default %u)\n"
        "      --direct N           direct sampling mode 0/1 (default %u)\n"
        "      --clock internal|external|0|1\n"
        "  -h, --help              show this help\n"
        "\n"
        "Examples:\n"
        "  %s\n"
        "  %s --freq-mhz 315 --samplerate 50M --seconds 60\n",
        argv0,
        DEFAULT_FREQ_MHZ, DEFAULT_SAMPLERATE, DEFAULT_SECONDS, DEFAULT_BW_RATIO,
        DEFAULT_BUF_LEN, DEFAULT_BUF_COUNT, DEFAULT_LNA_GAIN, DEFAULT_VGA_GAIN,
        DEFAULT_DIRECT, argv0, argv0);
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

static int parse_args(int argc, char **argv, test_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->frequency_hz = DEFAULT_FREQ_MHZ * 1.0e6;
    cfg->samplerate = DEFAULT_SAMPLERATE;
    cfg->seconds = DEFAULT_SECONDS;
    cfg->bw_ratio = DEFAULT_BW_RATIO;
    cfg->buf_count = DEFAULT_BUF_COUNT;
    cfg->buf_len = DEFAULT_BUF_LEN;
    cfg->lna_gain = DEFAULT_LNA_GAIN;
    cfg->vga_gain = DEFAULT_VGA_GAIN;
    cfg->direct_sampling = DEFAULT_DIRECT;
    cfg->clk_source = DEFAULT_CLK_SOURCE;

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
            strcmp(arg, "--freq-hz") == 0 || strcmp(arg, "-r") == 0 ||
            strcmp(arg, "--samplerate") == 0 || strcmp(arg, "-t") == 0 ||
            strcmp(arg, "--seconds") == 0 || strcmp(arg, "--bw-ratio") == 0 ||
            strcmp(arg, "--buf-len") == 0 || strcmp(arg, "--buf-count") == 0 ||
            strcmp(arg, "--lna") == 0 || strcmp(arg, "--vga") == 0 ||
            strcmp(arg, "--direct") == 0 || strcmp(arg, "--clock") == 0) {
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
        } else if (strcmp(arg, "-t") == 0 || strcmp(arg, "--seconds") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp <= 0.0)
                return -1;
            cfg->seconds = dtmp;
        } else if (strcmp(arg, "--bw-ratio") == 0) {
            if (parse_double_value(value, &dtmp) != 0 || dtmp < 0.0 || dtmp > 1.0)
                return -1;
            cfg->bw_ratio = dtmp;
        } else if (strcmp(arg, "--buf-len") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp == 0)
                return -1;
            cfg->buf_len = utmp;
        } else if (strcmp(arg, "--buf-count") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp == 0)
                return -1;
            cfg->buf_count = utmp;
        } else if (strcmp(arg, "--lna") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > 2)
                return -1;
            cfg->lna_gain = utmp;
        } else if (strcmp(arg, "--vga") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > 15)
                return -1;
            cfg->vga_gain = utmp;
        } else if (strcmp(arg, "--direct") == 0) {
            if (parse_uint_value(value, &utmp) != 0 || utmp > 1)
                return -1;
            cfg->direct_sampling = utmp;
        } else if (strcmp(arg, "--clock") == 0) {
            if (parse_clock_source(value, &cfg->clk_source) != 0)
                return -1;
        }
    }
    return 0;
}

static uint64_t buffer_signature(const float *buf, uint32_t complex_len)
{
    const uint64_t fnv_offset = 1469598103934665603ULL;
    const uint64_t fnv_prime = 1099511628211ULL;
    uint64_t hash = fnv_offset;
    uint32_t floats = complex_len * 2U;
    uint32_t stride = floats > 8192U ? floats / 8192U : 1U;

    hash ^= (uint64_t)complex_len;
    hash *= fnv_prime;

    for (uint32_t i = 0; i < floats; i += stride) {
        uint32_t bits;
        memcpy(&bits, &buf[i], sizeof(bits));
        hash ^= (uint64_t)bits;
        hash *= fnv_prime;
    }
    if (floats > 0) {
        uint32_t bits;
        memcpy(&bits, &buf[floats - 1], sizeof(bits));
        hash ^= (uint64_t)bits;
        hash *= fnv_prime;
    }
    return hash;
}

static void stream_callback(float *buf, uint32_t buf_length,
                            struct fobos_sdr_dev_t *sender, void *user)
{
    stream_stats_t *st = (stream_stats_t *)user;
    long long now = now_ns();
    uint64_t signature;
    double buffer_power = 0.0;

    if (st->callbacks == 0) {
        st->first_cb_ns = now;
        st->first_buf_len = buf_length;
        st->min_buf_len = buf_length;
        st->max_buf_len = buf_length;
    } else {
        long long gap = now - st->last_cb_ns;
        double gap_d = (double)gap;
        double expected_gap = 0.0;

        st->gap_count++;
        st->sum_gap_ns += gap_d;
        st->sum_gap2_ns += gap_d * gap_d;
        if (st->min_gap_ns == 0 || gap < st->min_gap_ns)
            st->min_gap_ns = gap;
        if (gap > st->max_gap_ns)
            st->max_gap_ns = gap;

        if (st->first_buf_len > 0 && st->cfg.samplerate > 0.0) {
            expected_gap = ((double)st->first_buf_len * 1.0e9) / st->cfg.samplerate;
            if (gap_d > expected_gap * 1.75) {
                uint64_t missing = 0;
                double rounded = floor((gap_d / expected_gap) + 0.5);
                if (rounded > 1.0)
                    missing = (uint64_t)(rounded - 1.0);
                st->large_gaps++;
                st->expected_missing += missing;
            }
        }
    }

    if (buf_length != st->first_buf_len)
        st->len_changes++;
    if (buf_length < st->cfg.buf_len)
        st->short_buffers++;
    if (buf_length < st->min_buf_len)
        st->min_buf_len = buf_length;
    if (buf_length > st->max_buf_len)
        st->max_buf_len = buf_length;

    if (buf_length > 0 && st->have_last_sample) {
        double dr = (double)buf[0] - (double)st->last_re;
        double di = (double)buf[1] - (double)st->last_im;
        double jump = dr * dr + di * di;
        st->boundary_jump_power += jump;
        if (jump > st->max_boundary_jump_power)
            st->max_boundary_jump_power = jump;
        st->boundary_count++;
    }

    for (uint32_t i = 0; i < buf_length; i++) {
        float re = buf[2U * i];
        float im = buf[2U * i + 1U];

        if (!isfinite(re) || !isfinite(im)) {
            st->invalid_samples++;
            continue;
        }

        st->sum_re += re;
        st->sum_im += im;
        st->sum_power += (double)re * (double)re + (double)im * (double)im;
        buffer_power += (double)re * (double)re + (double)im * (double)im;
        if (fabs((double)re) > st->max_abs)
            st->max_abs = fabs((double)re);
        if (fabs((double)im) > st->max_abs)
            st->max_abs = fabs((double)im);
        if (fabs((double)re) >= 0.999 || fabs((double)im) >= 0.999)
            st->clipped_samples++;
        if (re == 0.0f && im == 0.0f)
            st->zero_iq_pairs++;
    }

    if (buf_length > 0) {
        st->last_re = buf[2U * (buf_length - 1U)];
        st->last_im = buf[2U * (buf_length - 1U) + 1U];
        st->have_last_sample = 1;
    }

    if (buffer_power == 0.0 && buf_length > 0)
        st->zero_buffers++;

    signature = buffer_signature(buf, buf_length);
    if (st->have_signature && signature == st->previous_signature)
        st->duplicate_signatures++;
    st->previous_signature = signature;
    st->have_signature = 1;

    st->callbacks++;
    st->total_samples += buf_length;
    st->last_cb_ns = now;

    if (!st->stop_requested &&
        now - st->start_ns >= (long long)(st->cfg.seconds * 1000000000.0)) {
        st->stop_requested = 1;
        fobos_sdr_cancel_async(sender);
    }
}

static void *timer_thread(void *arg)
{
    stream_stats_t *st = (stream_stats_t *)arg;
    long long deadline = st->start_ns + (long long)(st->cfg.seconds * 1000000000.0);
    const struct timespec sleep_time = {0, 100000000L};

    while (!st->stop_requested && now_ns() < deadline)
        nanosleep(&sleep_time, NULL);

    if (!st->stop_requested) {
        st->stop_requested = 1;
        fobos_sdr_cancel_async(st->dev);
    }
    return NULL;
}

static double safe_percent(double value, double total)
{
    if (total <= 0.0)
        return 0.0;
    return 100.0 * value / total;
}

static void print_report(const stream_stats_t *st, int read_result)
{
    double elapsed = 0.0;
    double callback_span = 0.0;
    double observed_rate = 0.0;
    double expected_callbacks = 0.0;
    double callback_ratio = 0.0;
    double gap_avg_ms = 0.0;
    double gap_std_ms = 0.0;
    double rms = 0.0;
    double mean_re = 0.0;
    double mean_im = 0.0;
    double boundary_rms = 0.0;
    double missing_ratio = 0.0;
    double score = 100.0;
    const char *verdict = "GOOD";

    if (st->last_cb_ns > st->first_cb_ns)
        callback_span = (double)(st->last_cb_ns - st->first_cb_ns) / 1.0e9;
    if (st->last_cb_ns > st->start_ns)
        elapsed = (double)(st->last_cb_ns - st->start_ns) / 1.0e9;
    else
        elapsed = st->cfg.seconds;

    if (callback_span > 0.0)
        observed_rate = (double)st->total_samples / callback_span;
    if (st->first_buf_len > 0 && st->cfg.samplerate > 0.0 && callback_span > 0.0)
        expected_callbacks =
            callback_span * st->cfg.samplerate / (double)st->first_buf_len;
    if (expected_callbacks > 0.0)
        callback_ratio = 100.0 * (double)st->callbacks / expected_callbacks;
    if (st->gap_count > 0) {
        double avg_ns = st->sum_gap_ns / (double)st->gap_count;
        double var = st->sum_gap2_ns / (double)st->gap_count - avg_ns * avg_ns;
        if (var < 0.0)
            var = 0.0;
        gap_avg_ms = avg_ns / 1.0e6;
        gap_std_ms = sqrt(var) / 1.0e6;
    }
    if (st->total_samples > 0) {
        rms = sqrt(st->sum_power / (double)st->total_samples);
        mean_re = st->sum_re / (double)st->total_samples;
        mean_im = st->sum_im / (double)st->total_samples;
    }
    if (st->boundary_count > 0)
        boundary_rms = sqrt(st->boundary_jump_power / (double)st->boundary_count);
    if (st->callbacks + st->expected_missing > 0)
        missing_ratio = 100.0 * (double)st->expected_missing /
            (double)(st->callbacks + st->expected_missing);

    score -= missing_ratio * 5.0;
    score -= (double)st->large_gaps * 0.25;
    score -= safe_percent((double)st->invalid_samples, (double)st->total_samples) * 10.0;
    score -= (double)st->duplicate_signatures * 2.0;
    score -= (double)st->len_changes * 2.0;
    if (read_result != FOBOS_ERR_OK)
        score -= 25.0;
    if (st->callbacks == 0)
        score = 0.0;
    if (score < 0.0)
        score = 0.0;

    if (score < 70.0 || st->callbacks == 0 || read_result != FOBOS_ERR_OK)
        verdict = "FAIL";
    else if (score < 95.0 || st->large_gaps > 0 || st->duplicate_signatures > 0 ||
             st->invalid_samples > 0 || st->len_changes > 0)
        verdict = "WARN";

    printf("\n=== Fobos SDR stream integrity report ===\n");
    printf("Verdict: %s  score %.1f / 100\n", verdict, score);
    printf("Frequency: %.3f MHz\n", st->cfg.frequency_hz / 1.0e6);
    printf("Samplerate setting: %.3f MHz\n", st->cfg.samplerate / 1.0e6);
    printf("Run time: requested %.3f s, callback span %.3f s, elapsed %.3f s\n",
           st->cfg.seconds, callback_span, elapsed);
    printf("Async buffers: requested len %u complex samples, count %u\n",
           st->cfg.buf_len, st->cfg.buf_count);
    printf("Actual buffer length: first %u, min %u, max %u, length changes %llu, short buffers %llu\n",
           st->first_buf_len, st->min_buf_len, st->max_buf_len,
           (unsigned long long)st->len_changes,
           (unsigned long long)st->short_buffers);
    printf("Callbacks: %llu, expected about %.1f, ratio %.1f%%\n",
           (unsigned long long)st->callbacks, expected_callbacks, callback_ratio);
    printf("Inferred missing buffers: %llu (%.4f%%), large timing gaps: %llu\n",
           (unsigned long long)st->expected_missing, missing_ratio,
           (unsigned long long)st->large_gaps);
    printf("Callback interval: avg %.3f ms, stddev %.3f ms, min %.3f ms, max %.3f ms\n",
           gap_avg_ms, gap_std_ms, (double)st->min_gap_ns / 1.0e6,
           (double)st->max_gap_ns / 1.0e6);
    printf("Samples: %llu complex, observed %.3f MS/s\n",
           (unsigned long long)st->total_samples, observed_rate / 1.0e6);
    printf("Observed rate vs samplerate: %.1f%%\n",
           safe_percent(observed_rate, st->cfg.samplerate));
    printf("Signal: RMS %.6f, mean I %.6g, mean Q %.6g, max abs %.6f\n",
           rms, mean_re, mean_im, st->max_abs);
    printf("Sample issues: non-finite %llu, clipped-ish %llu (%.4f%%), zero IQ pairs %llu\n",
           (unsigned long long)st->invalid_samples,
           (unsigned long long)st->clipped_samples,
           safe_percent((double)st->clipped_samples, (double)st->total_samples),
           (unsigned long long)st->zero_iq_pairs);
    printf("Buffer signatures: consecutive duplicates %llu, all-zero buffers %llu\n",
           (unsigned long long)st->duplicate_signatures,
           (unsigned long long)st->zero_buffers);
    printf("Boundary jump: RMS %.6f, max %.6f\n",
           boundary_rms, sqrt(st->max_boundary_jump_power));
    printf("Read result: %d\n", read_result);
    printf("Note: the public Fobos callback does not expose hardware sequence numbers; loss/order is inferred from callback timing and buffer counts.\n");
}

static int check_ret(const char *name, int ret)
{
    if (ret != FOBOS_ERR_OK) {
        fprintf(stderr, "%s failed: %d\n", name, ret);
        return -1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    test_config_t cfg;
    stream_stats_t stats;
    struct fobos_sdr_dev_t *dev = NULL;
    pthread_t timer;
    int timer_started = 0;
    int ret;
    char lib_version[FOBOS_INFO_LEN] = {0};
    char drv_version[FOBOS_INFO_LEN] = {0};
    char hw_revision[FOBOS_INFO_LEN] = {0};
    char fw_version[FOBOS_INFO_LEN] = {0};
    char manufacturer[FOBOS_INFO_LEN] = {0};
    char product[FOBOS_INFO_LEN] = {0};
    char serial[FOBOS_INFO_LEN] = {0};

    if (parse_args(argc, argv, &cfg) != 0) {
        print_usage(argv[0]);
        return 2;
    }

    memset(&stats, 0, sizeof(stats));
    stats.cfg = cfg;

    fobos_sdr_get_api_info(lib_version, drv_version);
    printf("[TEST] API: %s (drv: %s)\n", lib_version, drv_version);

    ret = fobos_sdr_get_device_count();
    if (ret <= 0) {
        fprintf(stderr, "[TEST] No Fobos SDR devices found.\n");
        return 1;
    }
    printf("[TEST] Devices found: %d\n", ret);

    ret = fobos_sdr_open(&dev, 0);
    if (check_ret("fobos_sdr_open", ret) != 0)
        return 1;

    stats.dev = dev;

    ret = fobos_sdr_get_board_info(dev, hw_revision, fw_version,
                                   manufacturer, product, serial);
    if (ret == FOBOS_ERR_OK) {
        printf("[TEST] Device: %s %s\n", manufacturer, product);
        printf("[TEST]   HW: %s  FW: %s  S/N: %s\n",
               hw_revision, fw_version, serial);
    }

    if (check_ret("fobos_sdr_set_clk_source",
                  fobos_sdr_set_clk_source(dev, cfg.clk_source)) != 0)
        goto fail;
    if (check_ret("fobos_sdr_set_direct_sampling",
                  fobos_sdr_set_direct_sampling(dev, cfg.direct_sampling ? 1 : 0)) != 0)
        goto fail;
    if (!cfg.direct_sampling) {
        if (check_ret("fobos_sdr_set_frequency",
                      fobos_sdr_set_frequency(dev, cfg.frequency_hz)) != 0)
            goto fail;
        if (check_ret("fobos_sdr_set_lna_gain",
                      fobos_sdr_set_lna_gain(dev, cfg.lna_gain)) != 0)
            goto fail;
        if (check_ret("fobos_sdr_set_vga_gain",
                      fobos_sdr_set_vga_gain(dev, cfg.vga_gain)) != 0)
            goto fail;
    }
    if (check_ret("fobos_sdr_set_samplerate",
                  fobos_sdr_set_samplerate(dev, cfg.samplerate)) != 0)
        goto fail;
    if (check_ret("fobos_sdr_set_auto_bandwidth",
                  fobos_sdr_set_auto_bandwidth(dev, cfg.bw_ratio)) != 0)
        goto fail;

    printf("[TEST] Streaming %.3f MHz at %.3f MHz sample-rate for %.3f s\n",
           cfg.frequency_hz / 1.0e6, cfg.samplerate / 1.0e6, cfg.seconds);
    printf("[TEST] Buffer length %u complex samples, USB buffers %u, BW ratio %.3f\n",
           cfg.buf_len, cfg.buf_count, cfg.bw_ratio);

    stats.start_ns = now_ns();
    if (pthread_create(&timer, NULL, timer_thread, &stats) == 0)
        timer_started = 1;
    else
        fprintf(stderr, "[TEST] Could not start timer thread; callback timer will stop the stream.\n");

    ret = fobos_sdr_read_async(dev, stream_callback, &stats,
                               cfg.buf_count, cfg.buf_len);
    stats.stop_requested = 1;
    if (timer_started)
        pthread_join(timer, NULL);

    print_report(&stats, ret);
    fobos_sdr_close(dev);
    return (ret == FOBOS_ERR_OK && stats.callbacks > 0 &&
            stats.invalid_samples == 0 && stats.expected_missing == 0) ? 0 : 1;

fail:
    fobos_sdr_close(dev);
    return 1;
}
