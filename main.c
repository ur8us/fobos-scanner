/*
 * Fobos SDR Scanner - Backend
 *
 * Scanning algorithm:
 *   Build a frequency table from freq_start to freq_end.
 *   Let the Fobos SDR firmware scan that table in hardware.
 *   The SDR callback only copies each indexed scan buffer into a bounded
 *   queue. A worker thread computes FFT magnitude, stores it in that
 *   frequency slot, and publishes one horizontal waterfall line after every
 *   frequency slot has contributed.
 */

#define _USE_MATH_DEFINES
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <sys/select.h>
#include <errno.h>
#include <limits.h>
#include <fobos_sdr.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Config                                                             */
/* ------------------------------------------------------------------ */
#define PORT                8080
#define MAX_HEADERS         4096
#define MAX_REQUEST         65536
#define MAX_CLIENTS         64
#define MAX_SSE_CLIENTS     16
#define MAX_FREQS           256
#define MAX_SAMPLE_RATES    32
#define MAX_PATH            1024
#define HTML_PATH           "index.html"
#define BANDS_PATH          "bands.ini"
#define MARKERS_PATH        "markers.ini"
#define MIN_FREQ_START_HZ   50.0e6
#define DISPLAY_BINS_MIN    64
#define DISPLAY_BINS_MAX    32768

/* FFT: powers of 2 from 1024 to 65536 */
#define FFT_SIZE_MIN        1024
#define FFT_SIZE_MAX        65536
#define SINGLE_FFT_SIZE_MAX FFT_SIZE_MAX
#define SINGLE_DECIM_MAX    4096
#define SINGLE_ZERO_SHIFT_HZ 2000.0
#define CIC_STAGES          3
#define FFT_LEVEL_REF_SIZE  FFT_SIZE_MIN
#define FFTS_PER_STEP       32
#define SCAN_FFTS_PER_STEP  1
#define SCAN_BUF_LEN        65536
#define PROCESS_QUEUE_LEN   8
#define DIRECT_SAMPLING_RATE_HZ 50.0e6
#define DIRECT_SAMPLING_MAX_HZ 25.0e6
#define DB_FLOOR            -100.0f
#define DB_CEIL             -20.0f
#define MAX_MARKERS         2048
#define MARKER_NAME_MAX     128

/* Max bins per step at largest FFT */
#define MAX_BINS_PER_STEP   FFT_SIZE_MAX

/* Max total bins per line (256 freq steps * 65536 bins) */
#define MAX_BINS_PER_LINE   (MAX_FREQS * MAX_BINS_PER_STEP)

static int g_fft_size = 1024;   /* runtime FFT size */
static int g_bins_per_step = 512;  /* g_fft_size / 2 */

/* ------------------------------------------------------------------ */
/* Global state                                                       */
/* ------------------------------------------------------------------ */
static struct fobos_sdr_dev_t *g_dev = NULL;
static pthread_t g_scan_thread;
static volatile int g_scanning = 0;
static volatile int g_scan_thread_joinable = 0;
static pthread_mutex_t g_cancel_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_cancel_requested = 0;
typedef enum {
    RUN_MODE_SCAN = 0,
    RUN_MODE_SINGLE = 1
} run_mode_t;
typedef struct {
    int fft_size;
    int decim_factor;
    int decim_hop;
    double fft_samplerate;
    double source_span;
    double extraction_ratio;
} single_fft_plan_t;
static volatile run_mode_t g_active_mode = RUN_MODE_SCAN;
static pthread_mutex_t g_fft_mutex = PTHREAD_MUTEX_INITIALIZER;
static uint32_t g_fft_generation = 1;

/* Scan parameters */
static double g_freq_start   = MIN_FREQ_START_HZ;
static double g_freq_end     = 2000.0e6;
static double g_visible_start = MIN_FREQ_START_HZ;
static double g_visible_end   = 2000.0e6;
static double g_converter_freq = 0.0;
static double g_samplerate   = 50.0e6;
static double g_bw_ratio     = 0.1;
static int g_display_bins    = 1024;
static uint32_t g_min_rate_lps = 0;
static uint32_t g_rate_limit_lps = 20;
static uint32_t g_view_id    = 1;
static uint32_t g_lna_gain   = 0;
static uint32_t g_vga_gain   = 0;
static uint32_t g_direct_sampling = 0;
static uint32_t g_clk_source = 0;

/* Device info */
static char g_hw_rev[64]    = "unknown";
static char g_fw_ver[64]    = "unknown";
static char g_serial[64]    = "unknown";
static char g_manufacturer[64] = "unknown";
static char g_product[64]   = "unknown";
static double g_sample_rates[MAX_SAMPLE_RATES];
static unsigned int g_sample_rate_count = 0;

/* SSE client list */
static int g_sse_fds[MAX_SSE_CLIENTS];
static pthread_mutex_t g_sse_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Frontend traffic accounting: actual SSE bytes successfully written. */
#define TRAFFIC_SAMPLE_COUNT 1024
#define TRAFFIC_WINDOW_MS    2000LL
typedef struct {
    long long msec;
    size_t bytes;
} traffic_sample_t;

static traffic_sample_t g_traffic_samples[TRAFFIC_SAMPLE_COUNT];
static int g_traffic_sample_pos = 0;
static pthread_mutex_t g_traffic_mutex = PTHREAD_MUTEX_INITIALIZER;

/* ------------------------------------------------------------------ */
/* SSE helpers                                                        */
/* ------------------------------------------------------------------ */
static int write_all(int fd, const char *data, size_t len)
{
    size_t done = 0;
    while (done < len) {
        ssize_t n = write(fd, data + done, len - done);
        if (n < 0 && errno == EINTR) continue;
        if (n <= 0) return -1;
        done += (size_t)n;
    }
    return 0;
}

static void sse_add_client(int fd)
{
    int added = 0;
    pthread_mutex_lock(&g_sse_mutex);
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (g_sse_fds[i] == 0) {
            g_sse_fds[i] = fd;
            added = 1;
            break;
        }
    }
    pthread_mutex_unlock(&g_sse_mutex);
    if (!added) close(fd);
}

static size_t sse_broadcast(const char *data, int len)
{
    size_t delivered = 0;

    pthread_mutex_lock(&g_sse_mutex);
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        int fd = g_sse_fds[i];
        if (fd > 0) {
            if (write_all(fd, data, (size_t)len) != 0) {
                close(fd); g_sse_fds[i] = 0;
            } else {
                delivered += (size_t)len;
            }
        }
    }
    pthread_mutex_unlock(&g_sse_mutex);
    return delivered;
}

/* ------------------------------------------------------------------ */
/* FFT: radix-2, in-place, interleaved float [re,im,...]              */
/* ------------------------------------------------------------------ */
static void fft_c2c(float *data, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) {
            float tr = data[2*i], ti = data[2*i+1];
            data[2*i] = data[2*j]; data[2*i+1] = data[2*j+1];
            data[2*j] = tr; data[2*j+1] = ti;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = 2.0f * (float)M_PI / (float)len;
        float w_re = cosf(ang), w_im = -sinf(ang);
        for (int i = 0; i < n; i += len) {
            float cur_re = 1.0f, cur_im = 0.0f;
            for (int j = 0; j < len/2; j++) {
                int i0 = i + j, i1 = i0 + len/2;
                float re1 = data[2*i0], im1 = data[2*i0+1];
                float re2 = data[2*i1]*cur_re - data[2*i1+1]*cur_im;
                float im2 = data[2*i1]*cur_im + data[2*i1+1]*cur_re;
                data[2*i0] = re1 + re2; data[2*i0+1] = im1 + im2;
                data[2*i1] = re1 - re2; data[2*i1+1] = im1 - im2;
                float nr = cur_re*w_re - cur_im*w_im;
                float ni = cur_re*w_im + cur_im*w_re;
                cur_re = nr; cur_im = ni;
            }
        }
    }
}

/* Hann window — sized for max FFT */
static float g_window[FFT_SIZE_MAX];

static void init_window_for_size(float *window, int fft_size)
{
    for (int i = 0; i < fft_size; i++)
        window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (float)(fft_size - 1)));
}

static void init_window(void)
{
    init_window_for_size(g_window, g_fft_size);
}

static int normalize_fft_size(int new_size)
{
    if (new_size < FFT_SIZE_MIN) new_size = FFT_SIZE_MIN;
    if (new_size > FFT_SIZE_MAX) new_size = FFT_SIZE_MAX;
    /* Round down to nearest power of 2 */
    int size = 1;
    while (size * 2 <= new_size) size *= 2;
    return size;
}

/* Call this when g_fft_size changes */
static void update_fft_size(int new_size)
{
    int size = normalize_fft_size(new_size);
    pthread_mutex_lock(&g_fft_mutex);
    if (g_fft_size == size) {
        pthread_mutex_unlock(&g_fft_mutex);
        return;
    }
    g_fft_size = size;
    g_bins_per_step = g_fft_size / 2;
    init_window();
    g_fft_generation++;
    pthread_mutex_unlock(&g_fft_mutex);
}

static int current_fft_size(void)
{
    int size;
    pthread_mutex_lock(&g_fft_mutex);
    size = g_fft_size;
    pthread_mutex_unlock(&g_fft_mutex);
    return size;
}

static int next_power_of_two_int(int value)
{
    int size = 1;
    if (value <= 1)
        return 1;
    while (size < value && size <= SINGLE_FFT_SIZE_MAX / 2)
        size *= 2;
    return size;
}

static uint32_t normalize_rate_limit_lps(uint32_t value)
{
    static const uint32_t allowed[] = { 1, 2, 5, 10, 20, 50, 100 };
    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
        if (value == allowed[i])
            return value;
    }
    return 20;
}

static uint32_t normalize_min_rate_lps(uint32_t value)
{
    static const uint32_t allowed[] = { 0, 1, 2, 5, 10, 20 };
    for (size_t i = 0; i < sizeof(allowed) / sizeof(allowed[0]); i++) {
        if (value == allowed[i])
            return value;
    }
    return 0;
}

static uint32_t normalize_direct_sampling(uint32_t value)
{
    return value <= 2 ? value : 0;
}

static uint32_t normalize_clk_source(uint32_t value)
{
    return value ? 1 : 0;
}

static int direct_sampling_enabled(void)
{
    return g_direct_sampling != 0;
}

static double direct_sampling_max_hz(void)
{
    double max_hz = g_samplerate * 0.5;
    if (max_hz > DIRECT_SAMPLING_MAX_HZ)
        max_hz = DIRECT_SAMPLING_MAX_HZ;
    if (max_hz < 1.0)
        max_hz = 1.0;
    return max_hz;
}

static void force_direct_sampling_defaults(int reset_visible)
{
    g_converter_freq = 0.0;
    g_samplerate = DIRECT_SAMPLING_RATE_HZ;
    g_bw_ratio = 1.0;
    g_freq_start = 0.0;
    g_freq_end = direct_sampling_max_hz();

    if (reset_visible) {
        g_visible_start = g_freq_start;
        g_visible_end = g_freq_end;
    } else {
        if (g_visible_start < g_freq_start)
            g_visible_start = g_freq_start;
        if (g_visible_end > g_freq_end)
            g_visible_end = g_freq_end;
        if (g_visible_end <= g_visible_start) {
            g_visible_start = g_freq_start;
            g_visible_end = g_freq_end;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Persistent config                                                   */
/* ------------------------------------------------------------------ */
#define CONFIG_FILE "fobos-scanner.conf"

static void save_config(void)
{
    FILE *f = fopen(CONFIG_FILE, "w");
    if (!f) return;
    fprintf(f, "freq_start = %.0f\n", g_freq_start);
    fprintf(f, "freq_end = %.0f\n", g_freq_end);
    fprintf(f, "converter_freq = %.0f\n", g_converter_freq);
    fprintf(f, "samplerate = %.0f\n", g_samplerate);
    fprintf(f, "bw_ratio = %g\n", g_bw_ratio);
    fprintf(f, "lna_gain = %u\n", g_lna_gain);
    fprintf(f, "vga_gain = %u\n", g_vga_gain);
    fprintf(f, "direct_sampling = %u\n", g_direct_sampling);
    fprintf(f, "clk_source = %u\n", g_clk_source);
    fprintf(f, "fft_size = %d\n", g_fft_size);
    fprintf(f, "min_rate_lps = %u\n", g_min_rate_lps);
    fprintf(f, "rate_limit_lps = %u\n", g_rate_limit_lps);
    fclose(f);
}

static void load_config(void)
{
    FILE *f = fopen(CONFIG_FILE, "r");
    if (!f) return;
    char key[64];
    double val;
    unsigned int uval;
    while (fscanf(f, " %63s = %lf", key, &val) == 2) {
        if      (strcmp(key, "freq_start") == 0)       g_freq_start = val;
        else if (strcmp(key, "freq_end") == 0)         g_freq_end = val;
        else if (strcmp(key, "converter_freq") == 0)   g_converter_freq = val;
        else if (strcmp(key, "samplerate") == 0)       g_samplerate = val;
        else if (strcmp(key, "bw_ratio") == 0)         g_bw_ratio = val;
        else if (strcmp(key, "lna_gain") == 0)        { uval = (unsigned int)val; g_lna_gain = uval; }
        else if (strcmp(key, "vga_gain") == 0)        { uval = (unsigned int)val; g_vga_gain = uval; }
        else if (strcmp(key, "direct_sampling") == 0) { uval = (unsigned int)val; g_direct_sampling = normalize_direct_sampling(uval); }
        else if (strcmp(key, "clk_source") == 0)      { uval = (unsigned int)val; g_clk_source = normalize_clk_source(uval); }
        else if (strcmp(key, "fft_size") == 0) { update_fft_size((int)val); }
        else if (strcmp(key, "min_rate_lps") == 0) { uval = (unsigned int)val; g_min_rate_lps = normalize_min_rate_lps(uval); }
        else if (strcmp(key, "rate_limit_lps") == 0) { uval = (unsigned int)val; g_rate_limit_lps = normalize_rate_limit_lps(uval); }
    }
    fclose(f);
    g_visible_start = g_freq_start;
    g_visible_end = g_freq_end;
    if (direct_sampling_enabled())
        force_direct_sampling_defaults(1);
    printf("[SDR] Loaded config from %s\n", CONFIG_FILE);
}

/* ------------------------------------------------------------------ */
/* File reader                                                        */
/* ------------------------------------------------------------------ */
static char *read_file(const char *path, size_t *out_len)
{
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long len = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (len <= 0) { fclose(f); return NULL; }
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return NULL; }
    buf[fread(buf, 1, (size_t)len, f)] = 0;
    fclose(f);
    if (out_len) *out_len = (size_t)len;
    return buf;
}

static void send_json_response(int client_fd, int code, const char *reason,
                               const char *cors, const char *body);

typedef struct {
    char name[MARKER_NAME_MAX];
    char group[MARKER_NAME_MAX];
    double frequency_hz;
} marker_t;

static char *trim_ws(char *s)
{
    char *end;
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n')
        s++;
    end = s + strlen(s);
    while (end > s &&
           (end[-1] == ' ' || end[-1] == '\t' ||
            end[-1] == '\r' || end[-1] == '\n')) {
        *--end = 0;
    }
    return s;
}

static void copy_marker_text(char *dst, size_t dst_size, const char *src)
{
    if (!dst || dst_size == 0)
        return;
    if (!src)
        src = "";
    snprintf(dst, dst_size, "%s", src);
}

static int marker_group_exists(char groups[][MARKER_NAME_MAX], int count, const char *group)
{
    for (int i = 0; i < count; i++) {
        if (strcmp(groups[i], group) == 0)
            return 1;
    }
    return 0;
}

static void add_marker_group(char groups[][MARKER_NAME_MAX], int *count, const char *group)
{
    const char *name = (group && *group) ? group : "Ungrouped";
    if (*count >= MAX_MARKERS || marker_group_exists(groups, *count, name))
        return;
    copy_marker_text(groups[*count], MARKER_NAME_MAX, name);
    (*count)++;
}

static int load_markers(marker_t *markers, int max_markers,
                        char groups[][MARKER_NAME_MAX], int *group_count)
{
    size_t len = 0;
    char *text = read_file(MARKERS_PATH, &len);
    char *saveptr = NULL;
    char *line;
    marker_t current;
    int have_marker = 0;
    int count = 0;

    *group_count = 0;
    add_marker_group(groups, group_count, "Ungrouped");

    if (!text)
        return 0;

    memset(&current, 0, sizeof(current));
    copy_marker_text(current.group, sizeof(current.group), "Ungrouped");

    for (line = strtok_r(text, "\n", &saveptr);
         line;
         line = strtok_r(NULL, "\n", &saveptr)) {
        char *trimmed = trim_ws(line);
        char *eq;

        if (!*trimmed || *trimmed == '#' || *trimmed == ';')
            continue;

        if (trimmed[0] == '[') {
            if (have_marker && current.frequency_hz > 0.0 && current.name[0] && count < max_markers) {
                markers[count++] = current;
                add_marker_group(groups, group_count, current.group);
            }
            memset(&current, 0, sizeof(current));
            copy_marker_text(current.group, sizeof(current.group), "Ungrouped");
            have_marker = 1;
            continue;
        }

        eq = strchr(trimmed, '=');
        if (!eq)
            continue;
        *eq = 0;
        {
            char *key = trim_ws(trimmed);
            char *value = trim_ws(eq + 1);
            if (strcmp(key, "name") == 0) {
                copy_marker_text(current.name, sizeof(current.name), value);
            } else if (strcmp(key, "group") == 0) {
                copy_marker_text(current.group, sizeof(current.group), value);
                if (!current.group[0])
                    copy_marker_text(current.group, sizeof(current.group), "Ungrouped");
            } else if (strcmp(key, "frequency_hz") == 0) {
                current.frequency_hz = atof(value);
            } else if (strcmp(key, "frequency_mhz") == 0) {
                current.frequency_hz = atof(value) * 1.0e6;
            }
        }
    }

    if (have_marker && current.frequency_hz > 0.0 && current.name[0] && count < max_markers) {
        markers[count++] = current;
        add_marker_group(groups, group_count, current.group);
    }

    free(text);
    return count;
}

static int append_json_escaped(char *dst, size_t dst_size, int pos, const char *src)
{
    const unsigned char *s = (const unsigned char *)(src ? src : "");
    while (*s && pos < (int)dst_size - 1) {
        unsigned char c = *s++;
        if (c == '"' || c == '\\') {
            if (pos >= (int)dst_size - 2)
                break;
            dst[pos++] = '\\';
            dst[pos++] = (char)c;
        } else if (c < 0x20) {
            if (pos >= (int)dst_size - 7)
                break;
            pos += snprintf(dst + pos, dst_size - (size_t)pos, "\\u%04x", c);
        } else {
            dst[pos++] = (char)c;
        }
    }
    dst[pos] = 0;
    return pos;
}

static void send_markers_json(int client_fd, const char *cors)
{
    marker_t markers[MAX_MARKERS];
    char groups[MAX_MARKERS][MARKER_NAME_MAX];
    int group_count = 0;
    int marker_count = load_markers(markers, MAX_MARKERS, groups, &group_count);
    size_t body_size = 1048576;
    char *body = malloc(body_size);
    int pos;

    if (!body) {
        send_json_response(client_fd, 500, "Internal Server Error", cors,
                           "{\"status\":\"error\"}");
        return;
    }

    pos = snprintf(body, body_size, "{\"status\":\"ok\",\"groups\":[");
    for (int i = 0; i < group_count; i++) {
        pos += snprintf(body + pos, body_size - (size_t)pos, "%s\"", i ? "," : "");
        pos = append_json_escaped(body, body_size, pos, groups[i]);
        pos += snprintf(body + pos, body_size - (size_t)pos, "\"");
    }
    pos += snprintf(body + pos, body_size - (size_t)pos, "],\"markers\":[");
    for (int i = 0; i < marker_count; i++) {
        pos += snprintf(body + pos, body_size - (size_t)pos,
                        "%s{\"id\":%d,\"frequency_hz\":%.0f,\"name\":\"",
                        i ? "," : "", i, markers[i].frequency_hz);
        pos = append_json_escaped(body, body_size, pos, markers[i].name);
        pos += snprintf(body + pos, body_size - (size_t)pos, "\",\"group\":\"");
        pos = append_json_escaped(body, body_size, pos, markers[i].group);
        pos += snprintf(body + pos, body_size - (size_t)pos, "\"}");
    }
    pos += snprintf(body + pos, body_size - (size_t)pos, "]}");
    send_json_response(client_fd, 200, "OK", cors, body);
    free(body);
}

static void url_decode(char *s)
{
    char *r = s;
    while (*s) {
        if (*s == '%' && s[1] && s[2]) {
            char hex[3] = {s[1], s[2], 0};
            *r++ = (char)strtol(hex, NULL, 16); s += 3;
        } else if (*s == '+') { *r++ = ' '; s++; }
        else { *r++ = *s++; }
    }
    *r = 0;
}

static void chdir_to_executable_dir(const char *argv0)
{
    char path[PATH_MAX];
    char *slash;

    if (argv0 && strchr(argv0, '/')) {
        snprintf(path, sizeof(path), "%s", argv0);
    } else {
        ssize_t n = readlink("/proc/self/exe", path, sizeof(path) - 1);
        if (n <= 0)
            return;
        path[n] = 0;
    }

    slash = strrchr(path, '/');
    if (!slash)
        return;
    if (slash == path)
        slash[1] = 0;
    else
        *slash = 0;
    if (chdir(path) != 0)
        fprintf(stderr, "[SDR] chdir(%s) failed: %s\n", path, strerror(errno));
}

static int http_content_length(const char *req)
{
    const char *headers[] = { "Content-Length:", "content-length:" };
    for (size_t i = 0; i < sizeof(headers) / sizeof(headers[0]); i++) {
        const char *p = strstr(req, headers[i]);
        if (p) {
            int len = 0;
            if (sscanf(p + strlen(headers[i]), " %d", &len) == 1 && len > 0)
                return len;
        }
    }
    return 0;
}

static void load_sample_rates(struct fobos_sdr_dev_t *dev)
{
    unsigned int count = 0;

    g_sample_rate_count = 0;
    if (fobos_sdr_get_samplerates(dev, g_sample_rates, &count) != FOBOS_ERR_OK)
        return;

    if (count > MAX_SAMPLE_RATES)
        count = MAX_SAMPLE_RATES;
    g_sample_rate_count = count;
}

static void format_sample_rates_json(char *buf, size_t len)
{
    size_t pos = 0;

    if (len == 0)
        return;

    pos += (size_t)snprintf(buf + pos, len - pos, "[");
    for (unsigned int i = 0; i < g_sample_rate_count && pos < len; i++) {
        int n = snprintf(buf + pos, len - pos, "%s%.0f",
                         (i ? "," : ""), g_sample_rates[i]);
        if (n < 0)
            break;
        pos += (size_t)n;
    }
    if (pos < len)
        snprintf(buf + pos, len - pos, "]");
    else
        buf[len - 1] = 0;
}

static int normalize_display_bins(int bins);
static int current_display_bins(void);
static single_fft_plan_t single_fft_plan_for_span(double span);
static int scan_effective_fft_size_for_span(double span, int selected_fft_size);
static int planned_required_points(void);
static run_mode_t planned_run_mode(void);
static void clamp_visible_to_config(void);
static void raw_visible_band(double *out_start, double *out_end);
static void active_scan_band(double *out_start, double *out_end);
static void send_json_response(int client_fd, int code, const char *reason,
                               const char *cors, const char *body);
static void load_sample_rates(struct fobos_sdr_dev_t *dev);
static long long now_msec(void);

static void reset_async_cancel_request(void)
{
    pthread_mutex_lock(&g_cancel_mutex);
    g_cancel_requested = 0;
    pthread_mutex_unlock(&g_cancel_mutex);
}

static void request_async_cancel(void)
{
    pthread_mutex_lock(&g_cancel_mutex);
    if (g_dev && !g_cancel_requested) {
        g_cancel_requested = 1;
        fobos_sdr_cancel_async(g_dev);
    }
    pthread_mutex_unlock(&g_cancel_mutex);
}

static void record_frontend_traffic(size_t bytes)
{
    if (bytes == 0)
        return;

    pthread_mutex_lock(&g_traffic_mutex);
    g_traffic_samples[g_traffic_sample_pos].msec = now_msec();
    g_traffic_samples[g_traffic_sample_pos].bytes = bytes;
    g_traffic_sample_pos = (g_traffic_sample_pos + 1) % TRAFFIC_SAMPLE_COUNT;
    pthread_mutex_unlock(&g_traffic_mutex);
}

static double measured_frontend_kbytes_s(void)
{
    long long now = now_msec();
    long long cutoff = now - TRAFFIC_WINDOW_MS;
    long long oldest = now;
    size_t bytes = 0;

    pthread_mutex_lock(&g_traffic_mutex);
    for (int i = 0; i < TRAFFIC_SAMPLE_COUNT; i++) {
        if (g_traffic_samples[i].msec >= cutoff) {
            bytes += g_traffic_samples[i].bytes;
            if (g_traffic_samples[i].msec < oldest)
                oldest = g_traffic_samples[i].msec;
        }
    }
    pthread_mutex_unlock(&g_traffic_mutex);

    if (bytes == 0)
        return 0.0;

    {
        long long elapsed = now - oldest;
        if (elapsed < 1000)
            elapsed = 1000;
        if (elapsed > TRAFFIC_WINDOW_MS)
            elapsed = TRAFFIC_WINDOW_MS;
        return ((double)bytes / 1024.0) * (1000.0 / (double)elapsed);
    }
}

static void reset_device_info(void)
{
    snprintf(g_hw_rev, sizeof(g_hw_rev), "%s", "unknown");
    snprintf(g_fw_ver, sizeof(g_fw_ver), "%s", "unknown");
    snprintf(g_serial, sizeof(g_serial), "%s", "unknown");
    snprintf(g_manufacturer, sizeof(g_manufacturer), "%s", "unknown");
    snprintf(g_product, sizeof(g_product), "%s", "unknown");
    g_sample_rate_count = 0;
}

static void close_device(void)
{
    if (!g_dev)
        return;
    fobos_sdr_close(g_dev);
    g_dev = NULL;
    reset_device_info();
}

static int open_first_device(int verbose)
{
    int count;
    int ret;

    if (g_dev)
        return FOBOS_ERR_OK;

    count = fobos_sdr_get_device_count();
    if (verbose)
        printf("[SDR] Devices found: %d\n", count);
    if (count <= 0) {
        if (verbose)
            printf("[SDR] No device connected.\n");
        return -1;
    }

    ret = fobos_sdr_open(&g_dev, 0);
    if (ret != FOBOS_ERR_OK) {
        if (verbose)
            printf("[SDR] Could not open device: %d\n", ret);
        g_dev = NULL;
        reset_device_info();
        return ret;
    }

    ret = fobos_sdr_get_board_info(g_dev, g_hw_rev, g_fw_ver,
                                   g_manufacturer, g_product, g_serial);
    if (ret != FOBOS_ERR_OK && verbose)
        printf("[SDR] Could not read board info: %d\n", ret);
    load_sample_rates(g_dev);
    if (verbose) {
        printf("[SDR] Device: %s %s\n", g_manufacturer, g_product);
        printf("[SDR]   HW: %s  FW: %s  S/N: %s\n",
               g_hw_rev, g_fw_ver, g_serial);
    }
    return FOBOS_ERR_OK;
}

/* ------------------------------------------------------------------ */
/* Hardware scan context                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    float *samples;
    uint32_t buf_len;
    int channel;
    int ready;
} sample_queue_item_t;

typedef struct {
    int total_steps;
    int bins_per_step;
    int last_bins;
    int line_bins;
    int steps_seen;
    int line_num;
    int fft_size;
    int selected_fft_size;
    int decim_factor;
    int decim_fill;
    int decim_phase;
    int decim_hop;
    double overlap_factor;
    int async_buf_len;
    int single_mode;
    int rate_drop_factor;
    int rate_drop_cycle;
    int rate_have_cycle;
    int max_ffts_per_buffer;
    uint32_t rate_limit_lps;
    double rate_keep_ratio;
    double rate_keep_credit;
    double estimated_line_rate;
    uint64_t rate_callback_seq;
    uint64_t rate_scan_cycle_seq;
    uint64_t rate_output_seq;
    uint64_t rate_dropped;
    uint64_t rate_output_dropped;
    volatile long long last_callback_msec;
    volatile int watchdog_stop;
    volatile int watchdog_triggered;
    uint32_t fft_generation;
    double configured_start;
    double configured_end;
    double visible_start;
    double visible_end;
    double scan_start;
    double scan_end;
    double samplerate;
    double raw_samplerate;
    double bw_ratio;
    double step_width;
    double last_width;
    double center_freq;
    uint32_t view_id;
    uint32_t direct_sampling;
    double direct_mix_phase;
    double direct_mix_inc;
    float mag_scale;
    float *window;
    float *fft_scratch;
    float *decim_accum;
    float *line_buf;
    double cic_integrator_re[CIC_STAGES];
    double cic_integrator_im[CIC_STAGES];
    double cic_comb_re[CIC_STAGES];
    double cic_comb_im[CIC_STAGES];
    uint8_t step_seen[MAX_FREQS];
    sample_queue_item_t queue[PROCESS_QUEUE_LEN];
    int queue_head;
    int queue_tail;
    int queue_len;
    int worker_stop;
    uint64_t queue_dropped;
    pthread_mutex_t queue_mutex;
    pthread_cond_t queue_cond;
    pthread_t worker_thread;
} scan_ctx_t;

static int build_scan_frequencies_for_band(double start, double end, double *freqs, double *out_step)
{
    double step = g_samplerate * g_bw_ratio;
    double span = end - start;
    int count;

    if (out_step) *out_step = 0.0;
    if (step <= 0.0 || span <= 0.0) return 0;

    count = (int)ceil(span / step);
    if (count > MAX_FREQS)
        count = MAX_FREQS;

    if (out_step) *out_step = step;

    if (freqs) {
        for (int i = 0; i < count; i++) {
            double slice_start = start + (double)i * step;
            double width = end - slice_start;
            if (width > step)
                width = step;
            freqs[i] = slice_start + width / 2.0;
        }
    }

    return count;
}

static int build_scan_frequencies(double *freqs, double *out_step)
{
    double start;
    double end;
    active_scan_band(&start, &end);
    return build_scan_frequencies_for_band(start, end, freqs, out_step);
}

static double build_scan_effective_end_for_band(double start, double end_limit, int count, double step)
{
    if (count <= 0 || step <= 0.0)
        return start;
    double end = start + (double)count * step;
    return (end < end_limit) ? end : end_limit;
}

static double build_scan_effective_end(int count, double step)
{
    double start;
    double end;
    active_scan_band(&start, &end);
    return build_scan_effective_end_for_band(start, end, count, step);
}

static double scan_last_width_for_band(double start, double end, int count, double step)
{
    double last_start;
    double width;

    if (count <= 0 || step <= 0.0)
        return 0.0;

    last_start = start + (double)(count - 1) * step;
    width = end - last_start;
    if (width > step)
        width = step;
    if (width <= 0.0)
        width = step;
    return width;
}

static void clamp_scan_end_to_hardware_limit(void)
{
    double step = g_samplerate * g_bw_ratio;
    double max_end;

    if (direct_sampling_enabled()) {
        force_direct_sampling_defaults(1);
        return;
    }

    if (step <= 0.0 || g_freq_end <= g_freq_start)
        return;

    max_end = g_freq_start + (double)MAX_FREQS * step;
    if (g_freq_end > max_end)
        g_freq_end = max_end;
    g_visible_start = g_freq_start;
    g_visible_end = g_freq_end;
}

static int current_scan_plan(double *out_step, double *out_freq_end)
{
    double step = 0.0;
    int total_steps;

    if (planned_run_mode() == RUN_MODE_SINGLE) {
        double start;
        double end;
        single_fft_plan_t plan;
        raw_visible_band(&start, &end);
        plan = single_fft_plan_for_span(end - start);
        step = plan.source_span;
        total_steps = 1;
        if (out_step)
            *out_step = step;
        if (out_freq_end)
            *out_freq_end = end;
        return total_steps;
    }

    total_steps = build_scan_frequencies(NULL, &step);

    if (out_step)
        *out_step = step;
    if (out_freq_end)
        *out_freq_end = build_scan_effective_end(total_steps, step);
    return total_steps;
}

static int build_device_scan_frequencies(const double *air_freqs, int count, double *device_freqs)
{
    for (int i = 0; i < count; i++) {
        if (g_converter_freq >= 0.0)
            device_freqs[i] = air_freqs[i] - g_converter_freq;
        else
            device_freqs[i] = fabs(-g_converter_freq - air_freqs[i]);
        if (device_freqs[i] < 0.0)
            return -1;
    }
    return 0;
}

static double receiver_frequency_from_radio(double air_freq)
{
    if (g_converter_freq >= 0.0)
        return air_freq - g_converter_freq;
    return fabs(-g_converter_freq - air_freq);
}

static int source_band_covers_visible(double center, double source_span,
                                      double visible_start, double visible_end)
{
    double source_start = center - source_span * 0.5;
    double source_end = center + source_span * 0.5;
    return source_start <= visible_start && source_end >= visible_end;
}

static double single_source_center_for_visible(double visible_start,
                                               double visible_end,
                                               double source_span)
{
    double visible_span = visible_end - visible_start;
    double margin = visible_span * 0.05;
    double candidates[3];

    if (margin < SINGLE_ZERO_SHIFT_HZ)
        margin = SINGLE_ZERO_SHIFT_HZ;

    candidates[0] = visible_start - margin;
    candidates[1] = visible_end + margin;
    candidates[2] = (visible_start + visible_end) * 0.5;

    for (int i = 0; i < 3; i++) {
        double center = candidates[i];
        if (!source_band_covers_visible(center, source_span, visible_start, visible_end))
            continue;
        if (receiver_frequency_from_radio(center) >= 0.0)
            return center;
    }

    return candidates[2];
}

static double direct_source_center_for_visible(double visible_start,
                                               double visible_end,
                                               double source_span)
{
    double direct_max = direct_sampling_max_hz();
    double visible_span = visible_end - visible_start;
    double margin = visible_span * 0.05;
    double candidates[3];

    if (source_span >= direct_max)
        return direct_max * 0.5;
    if (margin < SINGLE_ZERO_SHIFT_HZ)
        margin = SINGLE_ZERO_SHIFT_HZ;

    candidates[0] = visible_start - margin;
    candidates[1] = visible_end + margin;
    candidates[2] = (visible_start + visible_end) * 0.5;

    for (int i = 0; i < 3; i++) {
        double center = candidates[i];
        double source_start = center - source_span * 0.5;
        double source_end = center + source_span * 0.5;
        if (source_start < 0.0 || source_end > direct_max)
            continue;
        if (source_start <= visible_start && source_end >= visible_end)
            return center;
    }

    {
        double center = (visible_start + visible_end) * 0.5;
        double half = source_span * 0.5;
        if (center < half)
            center = half;
        if (center > direct_max - half)
            center = direct_max - half;
        return center;
    }
}

static void raw_visible_band(double *out_start, double *out_end)
{
    clamp_visible_to_config();
    *out_start = g_visible_start;
    *out_end = g_visible_end;
}

static int required_points_for_band(double start, double end)
{
    double step = g_samplerate * g_bw_ratio;
    double span = end - start;
    int points;

    if (step <= 0.0 || span <= 0.0)
        return 0;

    points = (int)ceil(span / step);
    if (points < 1)
        points = 1;
    if (points > MAX_FREQS)
        points = MAX_FREQS;
    return points;
}

static int planned_required_points(void)
{
    double start;
    double end;
    if (direct_sampling_enabled())
        return 1;
    raw_visible_band(&start, &end);
    return required_points_for_band(start, end);
}

static run_mode_t planned_run_mode(void)
{
    if (direct_sampling_enabled())
        return RUN_MODE_SINGLE;
    return (planned_required_points() <= 1) ? RUN_MODE_SINGLE : RUN_MODE_SCAN;
}

static single_fft_plan_t single_fft_plan_for_span(double span)
{
    single_fft_plan_t plan;
    int display_bins = current_display_bins();
    double bw_ratio = g_bw_ratio;
    double sample_span;
    double needed_product;

    if (bw_ratio <= 0.0)
        bw_ratio = 1.0;
    if (bw_ratio > 1.0)
        bw_ratio = 1.0;
    sample_span = g_samplerate * bw_ratio;

    if (direct_sampling_enabled()) {
        sample_span = direct_sampling_max_hz();
        bw_ratio = sample_span / g_samplerate;
        if (bw_ratio <= 0.0)
            bw_ratio = 0.5;
    }

    plan.fft_size = FFT_SIZE_MIN;
    plan.decim_factor = 1;
    plan.fft_samplerate = g_samplerate;
    plan.source_span = sample_span;
    plan.extraction_ratio = bw_ratio;

    if (span <= 0.0 || g_samplerate <= 0.0 || display_bins <= 0)
        return plan;

    /*
     * To avoid one FFT bin covering many screen pixels, the product
     * FFT_size * decimation must be at least display_bins * samplerate/span.
     * Decimated single mode keeps the full decimated spectrum, so a moderate
     * power-of-two FFT can resolve much narrower spans without allocating a
     * huge USB buffer.
     */
    needed_product = ceil(((double)display_bins * g_samplerate) / span);
    if (needed_product < FFT_SIZE_MIN)
        needed_product = FFT_SIZE_MIN;

    if (needed_product <= (double)FFT_SIZE_MAX) {
        int effective = next_power_of_two_int((int)needed_product);
        if (effective < FFT_SIZE_MIN)
            effective = FFT_SIZE_MIN;
        if (effective > FFT_SIZE_MAX)
            effective = FFT_SIZE_MAX;
        plan.fft_size = effective;
        return plan;
    }

    {
        int decim = 1;
        int needed_decim = (int)ceil(needed_product / (double)FFT_SIZE_MAX);
        int min_decim_for_bw = (int)ceil(1.0 / bw_ratio);
        double needed_fft;

        if (needed_decim < 2)
            needed_decim = 2;
        if (needed_decim < min_decim_for_bw)
            needed_decim = min_decim_for_bw;
        while (decim < needed_decim && decim < SINGLE_DECIM_MAX)
            decim <<= 1;
        if (decim > SINGLE_DECIM_MAX)
            decim = SINGLE_DECIM_MAX;
        while (decim > 1 && (g_samplerate / (double)decim) < span * 1.05)
            decim >>= 1;

        needed_fft = ceil(needed_product / (double)decim);
        if (needed_fft < FFT_SIZE_MIN)
            needed_fft = FFT_SIZE_MIN;
        if (needed_fft > FFT_SIZE_MAX)
            needed_fft = FFT_SIZE_MAX;
        plan.decim_factor = decim;
        plan.fft_size = next_power_of_two_int((int)needed_fft);
        if (plan.fft_size < FFT_SIZE_MIN)
            plan.fft_size = FFT_SIZE_MIN;
        if (plan.fft_size > FFT_SIZE_MAX)
            plan.fft_size = FFT_SIZE_MAX;
        plan.fft_samplerate = g_samplerate / (double)decim;
        plan.source_span = plan.fft_samplerate;
        plan.extraction_ratio = 1.0;
    }
    return plan;
}

static int single_effective_fft_size_for_span(double span)
{
    return single_fft_plan_for_span(span).fft_size;
}

static int scan_effective_fft_size_for_span(double span, int selected_fft_size)
{
    int display_bins = current_display_bins();
    int selected = normalize_fft_size(selected_fft_size);
    double needed_fft;
    int effective;

    if (span <= 0.0 || g_samplerate <= 0.0 || display_bins <= 0)
        return selected;

    needed_fft = ceil(((double)display_bins * g_samplerate) / span);
    if (needed_fft < FFT_SIZE_MIN)
        needed_fft = FFT_SIZE_MIN;
    if (needed_fft > (double)FFT_SIZE_MAX)
        needed_fft = FFT_SIZE_MAX;

    effective = next_power_of_two_int((int)needed_fft);
    if (effective < FFT_SIZE_MIN)
        effective = FFT_SIZE_MIN;
    if (effective > selected)
        effective = selected;
    if (effective > FFT_SIZE_MAX)
        effective = FFT_SIZE_MAX;
    return effective;
}

static int scan_effective_fft_size_for_current_view(void)
{
    double start;
    double end;
    raw_visible_band(&start, &end);
    return scan_effective_fft_size_for_span(end - start, current_fft_size());
}

static int single_decim_hop_for_plan(const single_fft_plan_t *plan);

static uint32_t single_line_sample_count_for_span(double span)
{
    single_fft_plan_t plan = single_fft_plan_for_span(span);
    int hop = single_decim_hop_for_plan(&plan);
    double samples = (double)hop * (double)plan.decim_factor;
    if (samples > (double)UINT32_MAX)
        return UINT32_MAX;
    return (uint32_t)samples;
}

static int single_decim_hop_for_plan(const single_fft_plan_t *plan)
{
    uint32_t min_rate = normalize_min_rate_lps(g_min_rate_lps);
    uint32_t rate_limit = normalize_rate_limit_lps(g_rate_limit_lps);
    double target_lps;
    int hop;

    if (!plan || plan->decim_factor <= 1 || min_rate == 0)
        return plan ? plan->fft_size : FFT_SIZE_MIN;

    target_lps = (double)min_rate;
    if (rate_limit > 0 && target_lps > (double)rate_limit)
        target_lps = (double)rate_limit;
    if (target_lps <= 0.0)
        return plan->fft_size;

    hop = (int)floor(plan->fft_samplerate / target_lps);
    if (hop < 1)
        hop = 1;
    if (hop > plan->fft_size)
        hop = plan->fft_size;
    return hop;
}

static int current_effective_fft_size(void)
{
    double start;
    double end;
    if (planned_run_mode() != RUN_MODE_SINGLE)
        return scan_effective_fft_size_for_current_view();
    raw_visible_band(&start, &end);
    return single_effective_fft_size_for_span(end - start);
}

static int current_decim_factor(void)
{
    double start;
    double end;
    if (planned_run_mode() != RUN_MODE_SINGLE)
        return 1;
    raw_visible_band(&start, &end);
    return single_fft_plan_for_span(end - start).decim_factor;
}

static uint32_t current_line_sample_count(void)
{
    double start;
    double end;
    if (planned_run_mode() != RUN_MODE_SINGLE)
        return SCAN_BUF_LEN;
    raw_visible_band(&start, &end);
    return single_line_sample_count_for_span(end - start);
}

static int current_decim_hop(void)
{
    double start;
    double end;
    single_fft_plan_t plan;
    if (planned_run_mode() != RUN_MODE_SINGLE)
        return current_effective_fft_size();
    raw_visible_band(&start, &end);
    plan = single_fft_plan_for_span(end - start);
    return single_decim_hop_for_plan(&plan);
}

static double current_overlap_factor(void)
{
    int fft_size = current_effective_fft_size();
    int hop = current_decim_hop();
    if (hop <= 0)
        return 1.0;
    return (double)fft_size / (double)hop;
}

static const char *run_mode_name(run_mode_t mode)
{
    return mode == RUN_MODE_SINGLE ? "single" : "scan";
}

static int rate_drop_factor_for_plan(double samplerate, uint32_t async_len,
                                     int total_steps, uint32_t limit_lps,
                                     double *out_line_rate)
{
    double buffers_per_second;
    double lines_per_second;
    int factor;

    if (out_line_rate)
        *out_line_rate = 0.0;
    if (samplerate <= 0.0 || async_len == 0 || total_steps <= 0)
        return 1;

    limit_lps = normalize_rate_limit_lps(limit_lps);
    /*
     * fobos_sdr_read_async() reports buf_len as complex I/Q sample count.
     * One fixed-mode line consumes async_len samples. One hardware-scan line
     * consumes async_len samples for each scan step.
     */
    buffers_per_second = samplerate / (double)async_len;
    lines_per_second = buffers_per_second / (double)total_steps;
    if (out_line_rate)
        *out_line_rate = lines_per_second;

    if (lines_per_second <= (double)limit_lps)
        return 1;
    factor = (int)ceil(lines_per_second / (double)limit_lps);
    if (factor < 1)
        factor = 1;
    return factor;
}

static double rate_keep_ratio_for_line_rate(double line_rate, uint32_t limit_lps)
{
    limit_lps = normalize_rate_limit_lps(limit_lps);
    if (line_rate <= 0.0 || line_rate <= (double)limit_lps)
        return 1.0;
    return (double)limit_lps / line_rate;
}

static int scan_bins_per_step_for_width_and_fft(double width, int fft_size)
{
    double max_width = g_samplerate * g_bw_ratio;
    int bins;
    if (width > max_width) width = max_width;
    bins = (int)lrint((width / g_samplerate) * (double)fft_size);
    if (bins < 1) bins = 1;
    if (bins > fft_size) bins = fft_size;
    return bins;
}

static int bins_for_width_and_rate(double width, double samplerate,
                                   double ratio, int fft_size)
{
    double max_width;
    int bins;
    if (ratio <= 0.0)
        ratio = 1.0;
    if (ratio > 1.0)
        ratio = 1.0;
    max_width = samplerate * ratio;
    if (width > max_width) width = max_width;
    bins = (int)lrint((width / samplerate) * (double)fft_size);
    if (bins < 1) bins = 1;
    if (bins > fft_size) bins = fft_size;
    return bins;
}

static int scan_bins_per_step_for_width(double width)
{
    return scan_bins_per_step_for_width_and_fft(width, scan_effective_fft_size_for_current_view());
}

static int scan_bins_per_step(void)
{
    double step = 0.0;
    double start;
    double end;
    if (planned_run_mode() == RUN_MODE_SINGLE) {
        single_fft_plan_t plan;
        raw_visible_band(&start, &end);
        plan = single_fft_plan_for_span(end - start);
        return bins_for_width_and_rate(plan.source_span, plan.fft_samplerate,
                                       plan.extraction_ratio, plan.fft_size);
    }
    if (build_scan_frequencies(NULL, &step) <= 0)
        return 1;
    return scan_bins_per_step_for_width(g_samplerate * g_bw_ratio);
}

static int current_line_bins(void)
{
    double scan_start;
    double scan_end;
    double step = 0.0;
    int total_steps;
    int bins_per_step;
    int last_bins;
    int fft_size;

    if (planned_run_mode() == RUN_MODE_SINGLE) {
        single_fft_plan_t plan;
        raw_visible_band(&scan_start, &scan_end);
        plan = single_fft_plan_for_span(scan_end - scan_start);
        return bins_for_width_and_rate(plan.source_span, plan.fft_samplerate,
                                       plan.extraction_ratio, plan.fft_size);
    }

    active_scan_band(&scan_start, &scan_end);
    total_steps = build_scan_frequencies_for_band(scan_start, scan_end, NULL, &step);
    if (total_steps <= 0)
        return 0;
    fft_size = scan_effective_fft_size_for_current_view();
    bins_per_step = scan_bins_per_step_for_width_and_fft(step, fft_size);
    last_bins = scan_bins_per_step_for_width_and_fft(scan_last_width_for_band(scan_start, scan_end, total_steps, step), fft_size);
    return (total_steps - 1) * bins_per_step + last_bins;
}

static int scan_ctx_apply_fft_config(scan_ctx_t *ctx)
{
    int fft_size;
    uint32_t fft_generation;
    int bins_per_step;
    int last_bins;
    int line_bins;
    int decim_factor;
    int decim_hop;
    double fft_samplerate;
    double fft_ratio;
    double step_width;
    double last_width;
    size_t bin_count;
    float *window = NULL;
    float *fft_scratch = NULL;
    float *decim_accum = NULL;
    float *line_buf = NULL;
    float mag_scale;

    for (;;) {
        pthread_mutex_lock(&g_fft_mutex);
        fft_size = g_fft_size;
        fft_generation = g_fft_generation;
        pthread_mutex_unlock(&g_fft_mutex);
        ctx->selected_fft_size = fft_size;
        decim_factor = 1;
        decim_hop = fft_size;
        ctx->max_ffts_per_buffer = ctx->single_mode ? FFTS_PER_STEP : SCAN_FFTS_PER_STEP;
        fft_samplerate = ctx->raw_samplerate > 0.0 ? ctx->raw_samplerate : ctx->samplerate;
        fft_ratio = ctx->bw_ratio;
        step_width = ctx->step_width;
        last_width = ctx->last_width;
        if (ctx->single_mode) {
            single_fft_plan_t plan = single_fft_plan_for_span(ctx->visible_end - ctx->visible_start);
            fft_size = plan.fft_size;
            decim_factor = plan.decim_factor;
            decim_hop = single_decim_hop_for_plan(&plan);
            fft_samplerate = plan.fft_samplerate;
            fft_ratio = plan.extraction_ratio;
            step_width = plan.source_span;
            last_width = plan.source_span;
        } else {
            fft_size = scan_effective_fft_size_for_span(ctx->visible_end - ctx->visible_start,
                                                        ctx->selected_fft_size);
            decim_hop = fft_size;
        }

        bins_per_step = bins_for_width_and_rate(step_width, fft_samplerate, fft_ratio, fft_size);
        last_bins = bins_for_width_and_rate(last_width, fft_samplerate, fft_ratio, fft_size);
        line_bins = (ctx->total_steps - 1) * bins_per_step + last_bins;
        bin_count = (size_t)line_bins;

        window = malloc((size_t)fft_size * sizeof(float));
        fft_scratch = malloc((size_t)fft_size * 2 * sizeof(float));
        if (decim_factor > 1)
            decim_accum = malloc((size_t)fft_size * 2 * sizeof(float));
        line_buf = malloc(bin_count * sizeof(float));
        if (!window || !fft_scratch || (decim_factor > 1 && !decim_accum) || !line_buf) {
            free(window);
            free(fft_scratch);
            free(decim_accum);
            free(line_buf);
            return -1;
        }

        pthread_mutex_lock(&g_fft_mutex);
        if (ctx->selected_fft_size == g_fft_size) {
            if (fft_size == g_fft_size)
                memcpy(window, g_window, (size_t)fft_size * sizeof(float));
            else
                init_window_for_size(window, fft_size);
            fft_generation = g_fft_generation;
            pthread_mutex_unlock(&g_fft_mutex);
            break;
        }
        pthread_mutex_unlock(&g_fft_mutex);

        free(window);
        free(fft_scratch);
        free(decim_accum);
        free(line_buf);
        window = NULL;
        fft_scratch = NULL;
        decim_accum = NULL;
        line_buf = NULL;
    }

    {
        float window_sum = 0.0f;
        double effective_bins = (double)fft_size * (double)decim_factor;
        float bin_width_scale = sqrtf((float)(effective_bins / (double)FFT_LEVEL_REF_SIZE));
        for (int i = 0; i < fft_size; i++)
            window_sum += window[i];
        mag_scale = (window_sum > 0.0f) ? (bin_width_scale / window_sum) : (bin_width_scale / (float)fft_size);
    }

    free(ctx->window);
    free(ctx->fft_scratch);
    free(ctx->decim_accum);
    free(ctx->line_buf);

    ctx->fft_size = fft_size;
    ctx->fft_generation = fft_generation;
    ctx->decim_factor = decim_factor;
    ctx->decim_hop = decim_hop;
    ctx->overlap_factor = decim_hop > 0 ? (double)fft_size / (double)decim_hop : 1.0;
    ctx->decim_fill = 0;
    ctx->decim_phase = 0;
    ctx->bins_per_step = bins_per_step;
    ctx->last_bins = last_bins;
    ctx->line_bins = line_bins;
    ctx->samplerate = fft_samplerate;
    ctx->bw_ratio = fft_ratio;
    ctx->step_width = step_width;
    ctx->last_width = last_width;
    if (ctx->single_mode) {
        ctx->scan_start = ctx->center_freq - step_width * 0.5;
        ctx->scan_end = ctx->center_freq + step_width * 0.5;
    }
    ctx->direct_mix_phase = 0.0;
    ctx->direct_mix_inc = 0.0;
    if (ctx->direct_sampling && decim_factor > 1 &&
        ctx->raw_samplerate > 0.0) {
        ctx->direct_mix_inc = 2.0 * M_PI * ctx->center_freq / ctx->raw_samplerate;
        while (ctx->direct_mix_inc >= 2.0 * M_PI)
            ctx->direct_mix_inc -= 2.0 * M_PI;
        while (ctx->direct_mix_inc < 0.0)
            ctx->direct_mix_inc += 2.0 * M_PI;
    }
    ctx->mag_scale = mag_scale;
    ctx->window = window;
    ctx->fft_scratch = fft_scratch;
    ctx->decim_accum = decim_accum;
    ctx->line_buf = line_buf;
    ctx->steps_seen = 0;
    memset(ctx->cic_integrator_re, 0, sizeof(ctx->cic_integrator_re));
    memset(ctx->cic_integrator_im, 0, sizeof(ctx->cic_integrator_im));
    memset(ctx->cic_comb_re, 0, sizeof(ctx->cic_comb_re));
    memset(ctx->cic_comb_im, 0, sizeof(ctx->cic_comb_im));
    memset(ctx->step_seen, 0, sizeof(ctx->step_seen));

    return 0;
}

static uint32_t current_fft_generation(void)
{
    uint32_t generation;
    pthread_mutex_lock(&g_fft_mutex);
    generation = g_fft_generation;
    pthread_mutex_unlock(&g_fft_mutex);
    return generation;
}

static int average_fft_magnitude(float *buf, uint32_t buf_len, int bins_per_step,
                                 int fft_size, const float *window,
                                 float *local_fft, float mag_scale,
                                 int positive_half, int max_ffts, float *out)
{
    int fft_count = 0;
    int shifted_start = (fft_size - bins_per_step) / 2;

    memset(out, 0, (size_t)bins_per_step * sizeof(float));

    if (max_ffts < 1)
        max_ffts = 1;

    for (uint32_t pos = 0; pos + (uint32_t)fft_size <= buf_len && fft_count < max_ffts; pos += (uint32_t)fft_size) {
        for (int i = 0; i < fft_size; i++) {
            float w = window[i];
            local_fft[2*i]   = buf[2*(pos+i)]   * w;
            local_fft[2*i+1] = buf[2*(pos+i)+1] * w;
        }

        fft_c2c(local_fft, fft_size);

        for (int i = 0; i < bins_per_step; i++) {
            int fft_bin;
            if (positive_half)
                fft_bin = i;
            else {
                int shifted_bin = shifted_start + i;
                fft_bin = (shifted_bin + fft_size / 2) % fft_size;
            }
            float re = local_fft[2*fft_bin];
            float im = local_fft[2*fft_bin+1];
            out[i] += sqrtf(re*re + im*im);
        }
        fft_count++;
    }

    if (fft_count <= 0) return 0;

    float inv = mag_scale / (float)fft_count;
    for (int i = 0; i < bins_per_step; i++)
        out[i] *= inv;

    return fft_count;
}

static void advance_decim_frame(scan_ctx_t *ctx)
{
    int hop = ctx->decim_hop;
    int keep;

    if (hop <= 0 || hop >= ctx->fft_size) {
        ctx->decim_fill = 0;
        return;
    }

    keep = ctx->fft_size - hop;
    memmove(ctx->decim_accum, ctx->decim_accum + (size_t)hop * 2,
            (size_t)keep * 2 * sizeof(float));
    ctx->decim_fill = keep;
}

static int fft_magnitude_from_accum(scan_ctx_t *ctx, int bins_per_step, float *out)
{
    int shifted_start = (ctx->fft_size - bins_per_step) / 2;

    memset(out, 0, (size_t)bins_per_step * sizeof(float));
    for (int i = 0; i < ctx->fft_size; i++) {
        float w = ctx->window[i];
        ctx->fft_scratch[2*i] = ctx->decim_accum[2*i] * w;
        ctx->fft_scratch[2*i+1] = ctx->decim_accum[2*i+1] * w;
    }

    fft_c2c(ctx->fft_scratch, ctx->fft_size);

    for (int i = 0; i < bins_per_step; i++) {
        int shifted_bin = shifted_start + i;
        int fft_bin = (shifted_bin + ctx->fft_size / 2) % ctx->fft_size;
        float re = ctx->fft_scratch[2*fft_bin];
        float im = ctx->fft_scratch[2*fft_bin+1];
        out[i] = sqrtf(re*re + im*im) * ctx->mag_scale;
    }

    return 1;
}

static int cic_decimated_fft_magnitude(scan_ctx_t *ctx, float *buf,
                                       uint32_t buf_len, int bins_per_step,
                                       float *out)
{
    int decim = ctx->decim_factor;
    int produced = 0;
    double gain;

    if (decim <= 1 || !ctx->decim_accum)
        return average_fft_magnitude(buf, buf_len, bins_per_step, ctx->fft_size,
                                     ctx->window, ctx->fft_scratch, ctx->mag_scale, 0,
                                     ctx->max_ffts_per_buffer,
                                     out);

    gain = pow((double)decim, (double)CIC_STAGES);

    for (uint32_t n = 0; n < buf_len; n++) {
        double re = buf[2*n];
        double im = buf[2*n + 1];

        ctx->cic_integrator_re[0] += re;
        ctx->cic_integrator_im[0] += im;
        for (int s = 1; s < CIC_STAGES; s++) {
            ctx->cic_integrator_re[s] += ctx->cic_integrator_re[s - 1];
            ctx->cic_integrator_im[s] += ctx->cic_integrator_im[s - 1];
        }

        ctx->decim_phase++;
        if (ctx->decim_phase < decim)
            continue;
        ctx->decim_phase = 0;

        re = ctx->cic_integrator_re[CIC_STAGES - 1];
        im = ctx->cic_integrator_im[CIC_STAGES - 1];
        for (int s = 0; s < CIC_STAGES; s++) {
            double next_re = re - ctx->cic_comb_re[s];
            double next_im = im - ctx->cic_comb_im[s];
            ctx->cic_comb_re[s] = re;
            ctx->cic_comb_im[s] = im;
            re = next_re;
            im = next_im;
        }

        ctx->decim_accum[2*ctx->decim_fill] = (float)(re / gain);
        ctx->decim_accum[2*ctx->decim_fill + 1] = (float)(im / gain);
        ctx->decim_fill++;

        if (ctx->decim_fill >= ctx->fft_size) {
            int ok = fft_magnitude_from_accum(ctx, bins_per_step, out);
            if (ok > 0)
                produced++;
            advance_decim_frame(ctx);
        }
    }

    return produced;
}

static void prepare_direct_sampling_samples(scan_ctx_t *ctx, float *buf, uint32_t buf_len)
{
    uint32_t direct_sampling = ctx->direct_sampling;

    if (direct_sampling == 0)
        return;

    if (ctx->decim_factor > 1) {
        double phase = ctx->direct_mix_phase;
        double inc = ctx->direct_mix_inc;

        for (uint32_t i = 0; i < buf_len; i++) {
            float sample = (direct_sampling == 2) ? buf[2*i + 1] : buf[2*i];
            buf[2*i] = sample * (float)cos(phase);
            buf[2*i + 1] = -sample * (float)sin(phase);
            phase += inc;
            if (phase >= 2.0 * M_PI)
                phase -= 2.0 * M_PI;
            else if (phase < 0.0)
                phase += 2.0 * M_PI;
        }

        ctx->direct_mix_phase = phase;
        return;
    }

    if (direct_sampling == 1) {
        for (uint32_t i = 0; i < buf_len; i++)
            buf[2*i + 1] = 0.0f;
    } else if (direct_sampling == 2) {
        for (uint32_t i = 0; i < buf_len; i++) {
            buf[2*i] = buf[2*i + 1];
            buf[2*i + 1] = 0.0f;
        }
    }
}

static uint8_t magnitude_to_u8(float mag)
{
    float db = 20.0f * log10f(mag + 1e-20f);
    float v = (db - DB_FLOOR) * (255.0f / (DB_CEIL - DB_FLOOR));
    if (v < 0.0f) v = 0.0f;
    if (v > 255.0f) v = 255.0f;
    return (uint8_t)lrintf(v);
}

static void publish_scan_line(scan_ctx_t *ctx)
{
    int source_bins = ctx->line_bins;
    int display_bins = current_display_bins();
    double source_span = ctx->scan_end - ctx->scan_start;
    double visible_span = ctx->visible_end - ctx->visible_start;
    uint8_t *line_packed;
    char *json;
    int pos;

    if (source_bins <= 0 || display_bins <= 0 || source_span <= 0.0 || visible_span <= 0.0)
        return;

    line_packed = malloc((size_t)display_bins);
    if (!line_packed)
        return;

    for (int x = 0; x < display_bins; x++) {
        double pixel_start = ctx->visible_start + ((double)x / (double)display_bins) * visible_span;
        double pixel_end = ctx->visible_start + ((double)(x + 1) / (double)display_bins) * visible_span;
        int first = (int)floor(((pixel_start - ctx->scan_start) / source_span) * (double)source_bins);
        int last = (int)ceil(((pixel_end - ctx->scan_start) / source_span) * (double)source_bins);
        float peak = 0.0f;

        if (first < 0) first = 0;
        if (first >= source_bins) first = source_bins - 1;
        if (last < first + 1) last = first + 1;
        if (last > source_bins) last = source_bins;

        for (int i = first; i < last; i++) {
            if (ctx->line_buf[i] > peak)
                peak = ctx->line_buf[i];
        }
        line_packed[x] = magnitude_to_u8(peak);
    }

    json = malloc((size_t)display_bins * 5 + 1024);
    if (!json) {
        free(line_packed);
        return;
    }

    pos = snprintf(json, 1024,
        "event: line\ndata: {\"view\":%u,\"n\":%d,\"b\":%d,"
        "\"mode\":\"%s\","
        "\"f0\":%.0f,\"f1\":%.0f,"
        "\"full_f0\":%.0f,\"full_f1\":%.0f,"
        "\"visible_start_hz\":%.0f,\"visible_end_hz\":%.0f,"
        "\"display_bins\":%d,\"source_bins\":%d,\"effective_fft_size\":%d,"
        "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,\"d\":[",
        ctx->view_id, ctx->line_num, ctx->total_steps,
        ctx->single_mode ? "single" : "scan",
        ctx->visible_start, ctx->visible_end,
        ctx->configured_start, ctx->configured_end,
        ctx->visible_start, ctx->visible_end,
        display_bins, source_bins, ctx->fft_size,
        ctx->decim_factor, ctx->decim_hop, ctx->overlap_factor);

    for (int i = 0; i < display_bins; i++)
        pos += snprintf(json + pos, 5, "%s%d", (i ? "," : ""), line_packed[i]);

    pos += snprintf(json + pos, 16, "]}\n\n");
    record_frontend_traffic(sse_broadcast(json, pos));
    free(line_packed);
    free(json);
}

static int rate_limiter_should_keep(scan_ctx_t *ctx);

static int should_publish_processed_line(scan_ctx_t *ctx)
{
    if (ctx->single_mode && ctx->decim_factor > 1 &&
        ctx->rate_keep_ratio < 0.999999) {
        if (rate_limiter_should_keep(ctx))
            return 1;
        ctx->rate_output_dropped++;
        return 0;
    }
    return 1;
}

static int rate_limiter_should_keep(scan_ctx_t *ctx)
{
    double ratio = ctx->rate_keep_ratio;

    if (ratio <= 0.0 || ratio >= 0.999999)
        return 1;

    ctx->rate_keep_credit += ratio;
    if (ctx->rate_keep_credit >= 1.0) {
        ctx->rate_keep_credit -= 1.0;
        if (ctx->rate_keep_credit > 1.0)
            ctx->rate_keep_credit = fmod(ctx->rate_keep_credit, 1.0);
        return 1;
    }
    return 0;
}

static void process_scan_buffer(scan_ctx_t *ctx, float *buf, uint32_t buf_len, int channel)
{
    int channel_bins;
    float *slot;

    if (channel < 0 || channel >= ctx->total_steps)
        return;

    if (ctx->fft_generation != current_fft_generation()) {
        if (scan_ctx_apply_fft_config(ctx) != 0) {
            fprintf(stderr, "[SDR] Failed to apply FFT size %d\n", current_fft_size());
            return;
        }
    }

    if (ctx->direct_sampling)
        prepare_direct_sampling_samples(ctx, buf, buf_len);

    channel_bins = (channel == ctx->total_steps - 1) ? ctx->last_bins : ctx->bins_per_step;
    slot = ctx->line_buf + (size_t)channel * ctx->bins_per_step;
    if (ctx->single_mode && ctx->decim_factor > 1) {
        if (cic_decimated_fft_magnitude(ctx, buf, buf_len, channel_bins, slot) <= 0)
            return;
    } else {
        if (average_fft_magnitude(buf, buf_len, channel_bins, ctx->fft_size,
                                  ctx->window, ctx->fft_scratch, ctx->mag_scale,
                                  ctx->direct_sampling != 0 && ctx->decim_factor <= 1,
                                  ctx->max_ffts_per_buffer,
                                  slot) <= 0)
            return;
    }

    if (!should_publish_processed_line(ctx))
        return;

    if (!ctx->step_seen[channel]) {
        ctx->step_seen[channel] = 1;
        ctx->steps_seen++;
    }

    if (ctx->steps_seen >= ctx->total_steps) {
        publish_scan_line(ctx);
        memset(ctx->step_seen, 0, sizeof(ctx->step_seen));
        ctx->steps_seen = 0;
        ctx->line_num++;
    }
}

static int normalize_display_bins(int bins)
{
    if (bins < DISPLAY_BINS_MIN) bins = DISPLAY_BINS_MIN;
    if (bins > DISPLAY_BINS_MAX) bins = DISPLAY_BINS_MAX;
    return bins;
}

static int current_display_bins(void)
{
    return normalize_display_bins(g_display_bins);
}

static void clamp_visible_to_config(void)
{
    if (direct_sampling_enabled()) {
        force_direct_sampling_defaults(0);
        return;
    }

    if (g_freq_start < MIN_FREQ_START_HZ)
        g_freq_start = MIN_FREQ_START_HZ;
    if (g_freq_end <= g_freq_start)
        g_freq_end = g_freq_start + g_samplerate;

    if (g_visible_start < g_freq_start)
        g_visible_start = g_freq_start;
    if (g_visible_end > g_freq_end)
        g_visible_end = g_freq_end;
    if (g_visible_end <= g_visible_start) {
        g_visible_start = g_freq_start;
        g_visible_end = g_freq_end;
    }
}

static void active_scan_band(double *out_start, double *out_end)
{
    double start = g_visible_start;
    double end = g_visible_end;
    double step = g_samplerate * g_bw_ratio;
    double config_span;
    double min_span;
    double center;

    clamp_visible_to_config();
    start = g_visible_start;
    end = g_visible_end;

    if (step <= 0.0 || end <= start) {
        *out_start = start;
        *out_end = end;
        return;
    }

    config_span = g_freq_end - g_freq_start;
    min_span = step * (double)FOBOS_MIN_FREQS_CNT;
    if (config_span < min_span)
        min_span = config_span;

    if (end - start < min_span && min_span > 0.0) {
        center = (start + end) * 0.5;
        start = center - min_span * 0.5;
        end = start + min_span;
        if (start < g_freq_start) {
            end += g_freq_start - start;
            start = g_freq_start;
        }
        if (end > g_freq_end) {
            start -= end - g_freq_end;
            end = g_freq_end;
        }
        if (start < g_freq_start)
            start = g_freq_start;
    }

    *out_start = start;
    *out_end = end;
}

static int scan_queue_init(scan_ctx_t *ctx, uint32_t sample_capacity)
{
    if (pthread_mutex_init(&ctx->queue_mutex, NULL) != 0)
        return -1;
    if (pthread_cond_init(&ctx->queue_cond, NULL) != 0) {
        pthread_mutex_destroy(&ctx->queue_mutex);
        return -1;
    }

    ctx->async_buf_len = (int)sample_capacity;
    for (int i = 0; i < PROCESS_QUEUE_LEN; i++) {
        ctx->queue[i].samples = malloc((size_t)sample_capacity * 2 * sizeof(float));
        if (!ctx->queue[i].samples) {
            for (int j = 0; j < i; j++) {
                free(ctx->queue[j].samples);
                ctx->queue[j].samples = NULL;
            }
            pthread_cond_destroy(&ctx->queue_cond);
            pthread_mutex_destroy(&ctx->queue_mutex);
            return -1;
        }
    }
    return 0;
}

static void scan_queue_destroy(scan_ctx_t *ctx)
{
    for (int i = 0; i < PROCESS_QUEUE_LEN; i++) {
        free(ctx->queue[i].samples);
        ctx->queue[i].samples = NULL;
    }
    pthread_cond_destroy(&ctx->queue_cond);
    pthread_mutex_destroy(&ctx->queue_mutex);
}

static void scan_queue_stop(scan_ctx_t *ctx)
{
    pthread_mutex_lock(&ctx->queue_mutex);
    ctx->worker_stop = 1;
    pthread_cond_broadcast(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_mutex);
}

static void scan_queue_push(scan_ctx_t *ctx, int channel, float *buf, uint32_t buf_len)
{
    sample_queue_item_t *item;

    if (buf_len > (uint32_t)ctx->async_buf_len)
        buf_len = (uint32_t)ctx->async_buf_len;

    pthread_mutex_lock(&ctx->queue_mutex);
    if (ctx->queue_len >= PROCESS_QUEUE_LEN) {
        ctx->queue_dropped++;
        pthread_mutex_unlock(&ctx->queue_mutex);
        return;
    }

    item = &ctx->queue[ctx->queue_tail];
    item->ready = 0;
    ctx->queue_tail = (ctx->queue_tail + 1) % PROCESS_QUEUE_LEN;
    ctx->queue_len++;
    pthread_mutex_unlock(&ctx->queue_mutex);

    memcpy(item->samples, buf, (size_t)buf_len * 2 * sizeof(float));

    pthread_mutex_lock(&ctx->queue_mutex);
    item->buf_len = buf_len;
    item->channel = channel;
    item->ready = 1;
    pthread_cond_signal(&ctx->queue_cond);
    pthread_mutex_unlock(&ctx->queue_mutex);
}

static void *scan_worker_thread(void *arg)
{
    scan_ctx_t *ctx = (scan_ctx_t *)arg;

    for (;;) {
        sample_queue_item_t *item;

        pthread_mutex_lock(&ctx->queue_mutex);
        while (ctx->queue_len == 0 || !ctx->queue[ctx->queue_head].ready) {
            if (ctx->worker_stop) {
                pthread_mutex_unlock(&ctx->queue_mutex);
                return NULL;
            }
            pthread_cond_wait(&ctx->queue_cond, &ctx->queue_mutex);
        }
        item = &ctx->queue[ctx->queue_head];
        pthread_mutex_unlock(&ctx->queue_mutex);

        process_scan_buffer(ctx, item->samples, item->buf_len, item->channel);

        pthread_mutex_lock(&ctx->queue_mutex);
        item->ready = 0;
        ctx->queue_head = (ctx->queue_head + 1) % PROCESS_QUEUE_LEN;
        ctx->queue_len--;
        pthread_cond_signal(&ctx->queue_cond);
        pthread_mutex_unlock(&ctx->queue_mutex);
    }
}

static int scan_callback_should_process(scan_ctx_t *ctx, int channel)
{
    if (ctx->rate_keep_ratio >= 0.999999)
        return 1;

    if (ctx->single_mode) {
        if (ctx->decim_factor > 1)
            return 1;
        ctx->rate_callback_seq++;
        if (rate_limiter_should_keep(ctx))
            return 1;
        ctx->rate_dropped++;
        return 0;
    }

    if (channel == 0) {
        uint64_t seq = ctx->rate_scan_cycle_seq++;
        ctx->rate_have_cycle = 1;
        (void)seq;
        ctx->rate_drop_cycle = !rate_limiter_should_keep(ctx);
    } else if (!ctx->rate_have_cycle) {
        ctx->rate_drop_cycle = 0;
    }

    if (ctx->rate_drop_cycle) {
        ctx->rate_dropped++;
        return 0;
    }
    return 1;
}

static void scan_callback(float *buf, uint32_t buf_len, struct fobos_sdr_dev_t *dev, void *user)
{
    scan_ctx_t *ctx = (scan_ctx_t *)user;
    int channel;

    (void)dev;

    if (!g_scanning) {
        return;
    }

    ctx->last_callback_msec = now_msec();
    channel = ctx->single_mode ? 0 : fobos_sdr_get_scan_index(dev);
    if (channel < 0 || channel >= ctx->total_steps)
        return;
    if (!scan_callback_should_process(ctx, channel))
        return;

    scan_queue_push(ctx, channel, buf, buf_len);
}

static void *scan_watchdog_thread(void *arg)
{
    scan_ctx_t *ctx = (scan_ctx_t *)arg;
    const long long startup_timeout_ms = 2500;
    const long long stall_timeout_ms = 5000;
    const struct timespec sleep_time = { 0, 100000000L };

    while (!ctx->watchdog_stop && g_scanning) {
        long long now = now_msec();
        long long last = ctx->last_callback_msec;
        long long timeout = ctx->line_num == 0 ? startup_timeout_ms : stall_timeout_ms;

        if (last > 0 && now - last > timeout) {
            ctx->watchdog_triggered = 1;
            fprintf(stderr,
                    "[SDR] No sample buffers received for %.1f s; canceling async read\n",
                    (double)(now - last) / 1000.0);
            g_scanning = 0;
            request_async_cancel();
            break;
        }
        nanosleep(&sleep_time, NULL);
    }
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Scanning thread                                                     */
/* ------------------------------------------------------------------ */
static void *scan_thread_func(void *arg)
{
    (void)arg;
    double air_freqs[MAX_FREQS];
    double device_freqs[MAX_FREQS];
    double scan_start = 0.0;
    double scan_end = 0.0;
    double step = 0.0;
    double device_center = 0.0;
    run_mode_t mode;
    int total_steps;
    scan_ctx_t ctx;
    int ret;
    int worker_started = 0;
    int device_error = 0;
    int watchdog_started = 0;
    pthread_t watchdog_thread;
    uint32_t async_len;
    uint32_t line_sample_count;
    single_fft_plan_t single_plan = {0};

    mode = planned_run_mode();
    g_active_mode = mode;

    if (mode == RUN_MODE_SINGLE) {
        double visible_start;
        double visible_end;
        double center;

        raw_visible_band(&visible_start, &visible_end);
        if (visible_end <= visible_start) {
            g_scanning = 0;
            return NULL;
        }
        single_plan = single_fft_plan_for_span(visible_end - visible_start);
        center = direct_sampling_enabled() ?
            direct_source_center_for_visible(visible_start, visible_end,
                                             single_plan.source_span) :
            single_source_center_for_visible(visible_start, visible_end,
                                             single_plan.source_span);
        total_steps = 1;
        step = single_plan.source_span;
        scan_start = center - single_plan.source_span * 0.5;
        scan_end = center + single_plan.source_span * 0.5;
        device_center = direct_sampling_enabled() ? 0.0 : receiver_frequency_from_radio(center);
        if (!direct_sampling_enabled() && device_center < 0.0) {
            fprintf(stderr, "[SDR] Converter frequency makes receiver frequency negative\n");
            g_scanning = 0;
            return NULL;
        }
    } else {
        active_scan_band(&scan_start, &scan_end);
        total_steps = build_scan_frequencies_for_band(scan_start, scan_end, air_freqs, &step);

        if (total_steps < FOBOS_MIN_FREQS_CNT) {
            fprintf(stderr, "[SDR] Hardware scan needs at least %d frequencies\n", FOBOS_MIN_FREQS_CNT);
            g_scanning = 0;
            return NULL;
        }

        if (build_device_scan_frequencies(air_freqs, total_steps, device_freqs) != 0) {
            fprintf(stderr, "[SDR] Converter frequency makes hardware scan frequency negative\n");
            g_scanning = 0;
            return NULL;
        }
    }

    memset(&ctx, 0, sizeof(ctx));
    ctx.total_steps = total_steps;
    ctx.single_mode = (mode == RUN_MODE_SINGLE);
    ctx.raw_samplerate = g_samplerate;
    ctx.samplerate = ctx.single_mode ? single_plan.fft_samplerate : g_samplerate;
    ctx.bw_ratio = ctx.single_mode ? single_plan.extraction_ratio : g_bw_ratio;
    ctx.step_width = step;
    ctx.last_width = ctx.single_mode ? (scan_end - scan_start) :
        scan_last_width_for_band(scan_start, scan_end, total_steps, step);
    ctx.configured_start = g_freq_start;
    ctx.configured_end = g_freq_end;
    ctx.visible_start = g_visible_start;
    ctx.visible_end = g_visible_end;
    ctx.scan_start = scan_start;
    ctx.scan_end = ctx.single_mode ? scan_end :
        build_scan_effective_end_for_band(scan_start, scan_end, total_steps, step);
    ctx.center_freq = (scan_start + scan_end) * 0.5;
    ctx.view_id = g_view_id;
    ctx.direct_sampling = normalize_direct_sampling(g_direct_sampling);

    if (scan_ctx_apply_fft_config(&ctx) != 0) {
        fprintf(stderr, "[SDR] Out of memory\n");
        g_scanning = 0;
        return NULL;
    }

    async_len = (ctx.single_mode && ctx.decim_factor > 1) ? SCAN_BUF_LEN :
        (ctx.single_mode ? (uint32_t)ctx.fft_size : SCAN_BUF_LEN);
    line_sample_count = (ctx.single_mode && ctx.decim_factor > 1) ?
        (uint32_t)((uint64_t)ctx.decim_hop * (uint64_t)ctx.decim_factor) : async_len;
    ctx.rate_limit_lps = normalize_rate_limit_lps(g_rate_limit_lps);
    ctx.rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                     ctx.total_steps,
                                                     ctx.rate_limit_lps,
                                                     &ctx.estimated_line_rate);
    ctx.rate_keep_ratio = rate_keep_ratio_for_line_rate(ctx.estimated_line_rate,
                                                        ctx.rate_limit_lps);
    ctx.rate_keep_credit = 1.0;
    if (scan_queue_init(&ctx, async_len) != 0) {
        fprintf(stderr, "[SDR] Out of memory\n");
        free(ctx.window);
        free(ctx.fft_scratch);
        free(ctx.decim_accum);
        free(ctx.line_buf);
        g_scanning = 0;
        return NULL;
    }

    if (pthread_create(&ctx.worker_thread, NULL, scan_worker_thread, &ctx) != 0) {
        fprintf(stderr, "[SDR] Failed to start scan processing worker\n");
        scan_queue_destroy(&ctx);
        free(ctx.window);
        free(ctx.fft_scratch);
        free(ctx.decim_accum);
        free(ctx.line_buf);
        g_scanning = 0;
        return NULL;
    }
    worker_started = 1;

    ret = fobos_sdr_set_clk_source(g_dev, (int)g_clk_source);
    if (ret != FOBOS_ERR_OK) { fprintf(stderr, "[SDR] set_clk_source failed: %d\n", ret); device_error = 1; }
    ret = fobos_sdr_set_samplerate(g_dev, g_samplerate);
    if (ret != FOBOS_ERR_OK) { fprintf(stderr, "[SDR] set_samplerate failed: %d\n", ret); device_error = 1; }
    ret = fobos_sdr_set_auto_bandwidth(g_dev, g_bw_ratio);
    if (ret != FOBOS_ERR_OK) { fprintf(stderr, "[SDR] set_auto_bandwidth failed: %d\n", ret); device_error = 1; }
    if (!direct_sampling_enabled()) {
        ret = fobos_sdr_set_lna_gain(g_dev, g_lna_gain);
        if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_lna_gain failed: %d\n", ret);
        ret = fobos_sdr_set_vga_gain(g_dev, g_vga_gain);
        if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_vga_gain failed: %d\n", ret);
    }
    ret = fobos_sdr_set_direct_sampling(g_dev, direct_sampling_enabled() ? 1 : 0);
    if (ret != FOBOS_ERR_OK) { fprintf(stderr, "[SDR] set_direct_sampling failed: %d\n", ret); device_error = 1; }

    if (ctx.single_mode) {
        fobos_sdr_stop_scan(g_dev);
        if (!direct_sampling_enabled()) {
            ret = fobos_sdr_set_frequency(g_dev, device_center);
        } else {
            ret = FOBOS_ERR_OK;
        }
        if (ret != FOBOS_ERR_OK) {
            fprintf(stderr, "[SDR] set_frequency failed: %d\n", ret);
            device_error = 1;
            g_scanning = 0;
        }
        if (direct_sampling_enabled()) {
            printf("[SDR] Direct stream HF%u: visible %.0f - %.0f Hz, source %.0f - %.0f Hz, fft %d effective %d, async %u, line samples %u, %.1f lines/s raw, limit %u, drop %d\n",
                   ctx.direct_sampling, ctx.visible_start, ctx.visible_end,
                   ctx.scan_start, ctx.scan_end, ctx.selected_fft_size, ctx.fft_size,
                   async_len, line_sample_count,
                   ctx.estimated_line_rate, ctx.rate_limit_lps, ctx.rate_drop_factor);
        } else {
            printf("[SDR] Single stream: visible %.0f - %.0f Hz, source %.0f - %.0f Hz, converter %.0f Hz, SDR center %.0f Hz, fft %d effective %d, decim %d, hop %d, overlap %.2f, async %u, line samples %u, %.1f lines/s raw, min %u, limit %u, drop %d\n",
                   ctx.visible_start, ctx.visible_end,
                   ctx.scan_start, ctx.scan_end, g_converter_freq,
                   device_center, ctx.selected_fft_size, ctx.fft_size, ctx.decim_factor,
                   ctx.decim_hop, ctx.overlap_factor,
                   async_len, line_sample_count,
                   ctx.estimated_line_rate, g_min_rate_lps, ctx.rate_limit_lps, ctx.rate_drop_factor);
        }
    } else {
        printf("[SDR] Hardware scan: air band %.0f - %.0f Hz, converter %.0f Hz, SDR centers %.0f - %.0f Hz, step %.0f Hz (%d freqs, max %d), %.1f lines/s raw, limit %u, drop %d\n",
               ctx.scan_start, ctx.scan_end, g_converter_freq,
               device_freqs[0], device_freqs[total_steps - 1], step, total_steps, MAX_FREQS,
               ctx.estimated_line_rate, ctx.rate_limit_lps, ctx.rate_drop_factor);

        ret = fobos_sdr_start_scan(g_dev, device_freqs, (unsigned int)total_steps);
        if (ret != FOBOS_ERR_OK) {
            fprintf(stderr, "[SDR] fobos_sdr_start_scan failed: %d\n", ret);
            device_error = 1;
            g_scanning = 0;
        }
    }

    if (g_scanning) {
        ctx.last_callback_msec = now_msec();
        ctx.watchdog_stop = 0;
        ctx.watchdog_triggered = 0;
        reset_async_cancel_request();
        if (pthread_create(&watchdog_thread, NULL, scan_watchdog_thread, &ctx) == 0) {
            watchdog_started = 1;
        } else {
            fprintf(stderr, "[SDR] Failed to start scan watchdog\n");
        }
        ret = fobos_sdr_read_async(g_dev, scan_callback, &ctx, 16, async_len);
        if (ret != FOBOS_ERR_OK && g_scanning) {
            fprintf(stderr, "[SDR] fobos_sdr_read_async failed: %d\n", ret);
            device_error = 1;
        }
    }

    if (watchdog_started) {
        ctx.watchdog_stop = 1;
        pthread_join(watchdog_thread, NULL);
        if (ctx.watchdog_triggered)
            device_error = 1;
    }

    if (!ctx.single_mode)
        fobos_sdr_stop_scan(g_dev);
    if (worker_started) {
        scan_queue_stop(&ctx);
        pthread_join(ctx.worker_thread, NULL);
    }
    if (ctx.queue_dropped > 0) {
        fprintf(stderr, "[SDR] Dropped %llu scan buffers in processing queue\n",
                (unsigned long long)ctx.queue_dropped);
    }
    if (ctx.rate_dropped > 0) {
        fprintf(stderr, "[SDR] Rate-limited %llu sample buffers before FFT\n",
                (unsigned long long)ctx.rate_dropped);
    }
    if (ctx.rate_output_dropped > 0) {
        fprintf(stderr, "[SDR] Rate-limited %llu processed FFT lines\n",
                (unsigned long long)ctx.rate_output_dropped);
    }
    g_scanning = 0;
    scan_queue_destroy(&ctx);
    free(ctx.window);
    free(ctx.fft_scratch);
    free(ctx.decim_accum);
    free(ctx.line_buf);
    if (device_error) {
        fprintf(stderr, "[SDR] Closing SDR device after I/O error; will poll for reconnect\n");
        close_device();
    }
    printf("[SDR] Scan thread stopped\n");
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Start / Stop scan                                                  */
/* ------------------------------------------------------------------ */
static int start_scan(void)
{
    double air_freqs[MAX_FREQS];
    double device_freqs[MAX_FREQS];
    double start;
    double end;
    double step = 0.0;
    int total_steps;
    run_mode_t mode;

    /* If already scanning, restart with new params */
    if (g_scan_thread_joinable) {
        g_scanning = 0;
        request_async_cancel();
        pthread_join(g_scan_thread, NULL);
        g_scan_thread_joinable = 0;
    }

    mode = planned_run_mode();
    if (mode == RUN_MODE_SINGLE) {
        single_fft_plan_t plan;
        double center;
        raw_visible_band(&start, &end);
        if (end <= start)
            return -1;
        plan = single_fft_plan_for_span(end - start);
        center = direct_sampling_enabled() ?
            direct_source_center_for_visible(start, end, plan.source_span) :
            single_source_center_for_visible(start, end, plan.source_span);
        if (!direct_sampling_enabled() && receiver_frequency_from_radio(center) < 0.0)
            return -1;
    } else {
        total_steps = build_scan_frequencies(air_freqs, &step);
        if (total_steps < FOBOS_MIN_FREQS_CNT) return -1;
        if (build_device_scan_frequencies(air_freqs, total_steps, device_freqs) != 0) return -1;
    }

    if (!g_dev && open_first_device(0) != FOBOS_ERR_OK)
        return -1;

    g_scanning = 1;
    if (pthread_create(&g_scan_thread, NULL, scan_thread_func, NULL) != 0) {
        g_scanning = 0;
        return -1;
    }
    g_scan_thread_joinable = 1;
    return 0;
}

static void stop_scan(void)
{
    if (!g_scan_thread_joinable) return;
    g_scanning = 0;
    request_async_cancel();
    pthread_join(g_scan_thread, NULL);
    g_scan_thread_joinable = 0;
}

static int apply_gain_settings(void)
{
    int ret_lna = FOBOS_ERR_OK;
    int ret_vga = FOBOS_ERR_OK;

    if (!g_dev || !g_scanning)
        return FOBOS_ERR_OK;
    if (direct_sampling_enabled())
        return FOBOS_ERR_OK;

    ret_lna = fobos_sdr_set_lna_gain(g_dev, g_lna_gain);
    ret_vga = fobos_sdr_set_vga_gain(g_dev, g_vga_gain);

    if (ret_lna != FOBOS_ERR_OK)
        fprintf(stderr, "[SDR] set_lna_gain failed: %d\n", ret_lna);
    if (ret_vga != FOBOS_ERR_OK)
        fprintf(stderr, "[SDR] set_vga_gain failed: %d\n", ret_vga);

    return (ret_lna == FOBOS_ERR_OK && ret_vga == FOBOS_ERR_OK) ? FOBOS_ERR_OK : -1;
}

/* ------------------------------------------------------------------ */
/* HTTP request handler                                               */
/* ------------------------------------------------------------------ */
__attribute__((unused)) static void _nowarn(void *x) { (void)x; }
#define WRITE(fd, buf, len) do { ssize_t _r = write(fd, buf, len); _nowarn((void*)_r); } while(0)

static void send_json_response(int client_fd, int code, const char *reason,
                               const char *cors, const char *body)
{
    char header[512];
    size_t len = strlen(body);
    int n = snprintf(header, sizeof(header),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: application/json\r\n"
        "Content-Length: %zu\r\n"
        "Connection: close\r\n"
        "%s\r\n",
        code, reason, len, cors);
    WRITE(client_fd, header, (size_t)n);
    WRITE(client_fd, body, len);
}

static void handle_request(int client_fd, const char *req)
{
    char method[16], path[MAX_PATH];
    if (sscanf(req, "%15s %1023s", method, path) < 2) { close(client_fd); return; }
    url_decode(path);
    char *qs = strchr(path, '?');
    if (qs) *qs = 0;

    const char *cors = "Access-Control-Allow-Origin: *\r\n"
                       "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
                       "Access-Control-Allow-Headers: Content-Type\r\n";

    if (strcmp(method, "OPTIONS") == 0) {
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 204 No Content\r\n%s\r\n", cors);
        WRITE(client_fd, resp, (size_t)n);
        close(client_fd);
        return;
    }

    /* GET / or /index.html */
    if (strcmp(method, "GET") == 0 && (strcmp(path, "/") == 0 || strcmp(path, "/index.html") == 0)) {
        size_t len;
        char *html = read_file(HTML_PATH, &len);
        if (!html) {
            const char *err = "HTTP/1.1 500\r\nContent-Length: 0\r\n\r\n";
            WRITE(client_fd, err, strlen(err));
            close(client_fd);
            return;
        }
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html; charset=utf-8\r\n"
            "Content-Length: %zu\r\n%s\r\n", len, cors);
        WRITE(client_fd, resp, (size_t)n);
        WRITE(client_fd, html, len);
        free(html);
        close(client_fd);
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/bands.ini") == 0) {
        size_t len;
        char *text = read_file(BANDS_PATH, &len);
        if (!text) {
            const char *err = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
            WRITE(client_fd, err, strlen(err));
            close(client_fd);
            return;
        }
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain; charset=utf-8\r\n"
            "Content-Length: %zu\r\n%s\r\n", len, cors);
        WRITE(client_fd, resp, (size_t)n);
        WRITE(client_fd, text, len);
        free(text);
        close(client_fd);
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/markers.ini") == 0) {
        size_t len;
        char *text = read_file(MARKERS_PATH, &len);
        if (!text) {
            const char *empty =
                "# Human-editable frequency markers\n"
                "# Each marker section supports name, group, frequency_hz or frequency_mhz.\n";
            char resp[512];
            int n = snprintf(resp, sizeof(resp),
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/plain; charset=utf-8\r\n"
                "Content-Length: %zu\r\n%s\r\n",
                strlen(empty), cors);
            WRITE(client_fd, resp, (size_t)n);
            WRITE(client_fd, empty, strlen(empty));
            close(client_fd);
            return;
        }
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/plain; charset=utf-8\r\n"
            "Content-Length: %zu\r\n%s\r\n", len, cors);
        WRITE(client_fd, resp, (size_t)n);
        WRITE(client_fd, text, len);
        free(text);
        close(client_fd);
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/markers") == 0) {
        send_markers_json(client_fd, cors);
        close(client_fd);
        return;
    }

    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/markers/save") == 0) {
        const char *body = strstr(req, "\r\n\r\n");
        int content_len = http_content_length(req);
        FILE *f;
        if (!body) { close(client_fd); return; }
        body += 4;
        if (content_len <= 0)
            content_len = (int)strlen(body);
        f = fopen(MARKERS_PATH, "wb");
        if (!f) {
            send_json_response(client_fd, 500, "Internal Server Error", cors,
                               "{\"status\":\"error\"}");
            close(client_fd);
            return;
        }
        fwrite(body, 1, (size_t)content_len, f);
        fclose(f);
        send_json_response(client_fd, 200, "OK", cors, "{\"status\":\"ok\"}");
        close(client_fd);
        return;
    }

    /* GET /api/status */
    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/status") == 0) {
        char body[4096];
        char sample_rates[1024];
        double step = 0.0;
        double scan_start = 0.0;
        double scan_end = 0.0;
        int total_steps;
        int required_points = planned_required_points();
        run_mode_t mode = planned_run_mode();
        int bins_per_step = scan_bins_per_step();
        int line_bins = current_line_bins();
        int display_bins = current_display_bins();
        int effective_fft = current_effective_fft_size();
        int decim_factor = current_decim_factor();
        int decim_hop = current_decim_hop();
        double overlap_factor = current_overlap_factor();
        uint32_t line_sample_count;
        double raw_line_rate = 0.0;
        double traffic_kbytes_s;
        int rate_drop_factor;

        if (mode == RUN_MODE_SINGLE) {
            single_fft_plan_t plan;
            double center;
            raw_visible_band(&scan_start, &scan_end);
            plan = single_fft_plan_for_span(scan_end - scan_start);
            center = direct_sampling_enabled() ?
                direct_source_center_for_visible(scan_start, scan_end,
                                                 plan.source_span) :
                single_source_center_for_visible(scan_start, scan_end,
                                                 plan.source_span);
            step = plan.source_span;
            scan_start = center - plan.source_span * 0.5;
            scan_end = center + plan.source_span * 0.5;
            total_steps = 1;
        } else {
            active_scan_band(&scan_start, &scan_end);
            total_steps = build_scan_frequencies_for_band(scan_start, scan_end, NULL, &step);
            scan_end = build_scan_effective_end_for_band(scan_start, scan_end, total_steps, step);
        }
        line_sample_count = current_line_sample_count();
        rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                     total_steps,
                                                     g_rate_limit_lps,
                                                     &raw_line_rate);
        traffic_kbytes_s = measured_frontend_kbytes_s();
        format_sample_rates_json(sample_rates, sizeof(sample_rates));

        int n = snprintf(body, sizeof(body),
            "{\"device\":\"Fobos SDR\","
            "\"hardware\":\"%s\",\"firmware\":\"%s\",\"serial\":\"%s\","
            "\"manufacturer\":\"%s\",\"product\":\"%s\","
            "\"scanning\":%d,"
            "\"freq_start\":%.0f,\"freq_end\":%.0f,\"converter_freq\":%.0f,"
            "\"configured_start_hz\":%.0f,\"configured_end_hz\":%.0f,"
            "\"visible_start_hz\":%.0f,\"visible_end_hz\":%.0f,"
            "\"scan_start_hz\":%.0f,\"scan_end_hz\":%.0f,"
            "\"samplerate\":%.0f,\"bw_ratio\":%.2f,"
            "\"step_hz\":%.0f,\"steps\":%d,"
            "\"required_points\":%d,\"mode\":\"%s\",\"active_mode\":\"%s\","
            "\"min_rate_lps\":%u,\"rate_limit_lps\":%u,"
            "\"rate_drop_factor\":%d,\"raw_line_rate\":%.3f,"
            "\"traffic_kbytes_s\":%.1f,"
            "\"bins_per_step\":%d,\"line_bins\":%d,\"display_bins\":%d,"
            "\"view_id\":%u,\"fft_size\":%d,\"effective_fft_size\":%d,"
            "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,"
            "\"effective_input_samples\":%u,"
            "\"lna_gain\":%u,\"vga_gain\":%u,\"direct_sampling\":%u,"
            "\"clk_source\":%u,"
            "\"direct_sampling_max_hz\":%.0f,"
            "\"sample_rates\":%s}",
            g_hw_rev, g_fw_ver, g_serial,
            g_manufacturer, g_product,
            g_scanning,
            g_freq_start, g_freq_end, g_converter_freq,
            g_freq_start, g_freq_end,
            g_visible_start, g_visible_end,
            scan_start,
            scan_end,
            g_samplerate, g_bw_ratio,
            step, total_steps,
            required_points, run_mode_name(mode), run_mode_name(g_active_mode),
            g_min_rate_lps, g_rate_limit_lps, rate_drop_factor, raw_line_rate,
            traffic_kbytes_s,
            bins_per_step, line_bins, display_bins,
            g_view_id, current_fft_size(), effective_fft,
            decim_factor, decim_hop, overlap_factor, line_sample_count,
            g_lna_gain, g_vga_gain, g_direct_sampling, g_clk_source,
            direct_sampling_max_hz(),
            sample_rates);

        char resp[4096];
        int m = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %d\r\n%s\r\n%s", n, cors, body);
        WRITE(client_fd, resp, (size_t)m);
        close(client_fd);
        return;
    }

    /* GET /api/waterfall (SSE) */
    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/waterfall") == 0) {
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/event-stream\r\n"
            "Cache-Control: no-cache\r\n"
            "Connection: keep-alive\r\n%s\r\n", cors);
        WRITE(client_fd, resp, (size_t)n);
        sse_add_client(client_fd);
        return; /* keep open */
    }

    /* POST /api/start */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/start") == 0) {
        uint32_t fft_tmp = 0;
        uint32_t display_tmp = 0;
        uint32_t min_rate_tmp = 0;
        uint32_t rate_tmp = 0;
        double visible_start_tmp = 0.0;
        double visible_end_tmp = 0.0;
        int have_visible_start = 0;
        int have_visible_end = 0;
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        #define GET_DOUBLE(key, target, scale, minval) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    double v; \
                    if (sscanf(k+1, " %lf", &v) == 1 && v >= (minval)) \
                        (target) = v * (scale); \
                } \
            } \
        } while(0)
        #define GET_UINT(key, target) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    unsigned int v; \
                    if (sscanf(k+1, " %u", &v) == 1) \
                        (target) = (uint32_t)v; \
                } \
            } \
        } while(0)
        #define GET_DOUBLE_RAW_OPT(key, target, flag) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    double v; \
                    if (sscanf(k+1, " %lf", &v) == 1) { \
                        (target) = v; \
                        (flag) = 1; \
                    } \
                } \
            } \
        } while(0)

        GET_DOUBLE("\"freq_start\"", g_freq_start, 1.0e6, 1.0);
        GET_DOUBLE("\"freq_end\"", g_freq_end, 1.0e6, 1.0);
        GET_DOUBLE("\"converter_freq\"", g_converter_freq, 1.0e6, -1.0e12);
        GET_DOUBLE("\"samplerate\"", g_samplerate, 1.0, 1.0);
        GET_DOUBLE("\"bw_ratio\"", g_bw_ratio, 1.0, 0.000001);
        GET_UINT("\"lna_gain\"", g_lna_gain);
        GET_UINT("\"vga_gain\"", g_vga_gain);
        GET_UINT("direct_sampling", g_direct_sampling);
        GET_UINT("\"clk_source\"", g_clk_source);
        GET_UINT("\"clock_source\"", g_clk_source);
        GET_UINT("fft_size", fft_tmp);
        GET_UINT("display_bins", display_tmp);
        GET_UINT("min_rate_lps", min_rate_tmp);
        GET_UINT("rate_limit_lps", rate_tmp);
        GET_DOUBLE_RAW_OPT("\"visible_start_hz\"", visible_start_tmp, have_visible_start);
        GET_DOUBLE_RAW_OPT("\"visible_end_hz\"", visible_end_tmp, have_visible_end);
        if (fft_tmp >= FFT_SIZE_MIN && fft_tmp <= FFT_SIZE_MAX)
            update_fft_size((int)fft_tmp);
        if (display_tmp > 0)
            g_display_bins = normalize_display_bins((int)display_tmp);
        g_direct_sampling = normalize_direct_sampling(g_direct_sampling);
        g_clk_source = normalize_clk_source(g_clk_source);
        g_min_rate_lps = normalize_min_rate_lps(min_rate_tmp);
        if (rate_tmp > 0)
            g_rate_limit_lps = normalize_rate_limit_lps(rate_tmp);
        if (direct_sampling_enabled()) {
            force_direct_sampling_defaults(!have_visible_start || !have_visible_end);
            if (have_visible_start && have_visible_end) {
                g_visible_start = visible_start_tmp;
                g_visible_end = visible_end_tmp;
                clamp_visible_to_config();
            }
        } else {
            if (g_bw_ratio > 1.0) g_bw_ratio = 1.0;
            if (g_freq_start < MIN_FREQ_START_HZ)
                g_freq_start = MIN_FREQ_START_HZ;
            clamp_scan_end_to_hardware_limit();
            if (have_visible_start && have_visible_end) {
                g_visible_start = visible_start_tmp;
                g_visible_end = visible_end_tmp;
                clamp_visible_to_config();
            } else {
                g_visible_start = g_freq_start;
                g_visible_end = g_freq_end;
            }
        }
        g_view_id++;

        #undef GET_DOUBLE
        #undef GET_UINT
        #undef GET_DOUBLE_RAW_OPT

        save_config();

        int ret = start_scan();
        const char *status = (ret == 0) ? "ok" : "error";
        double step = 0.0;
        double scan_end = 0.0;
        int total_steps = current_scan_plan(&step, &scan_end);
        run_mode_t mode = planned_run_mode();
        int effective_fft = current_effective_fft_size();
        int decim_factor = current_decim_factor();
        int decim_hop = current_decim_hop();
        double overlap_factor = current_overlap_factor();
        uint32_t line_sample_count = current_line_sample_count();
        double raw_line_rate = 0.0;
        int rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                         total_steps,
                                                         g_rate_limit_lps,
                                                         &raw_line_rate);
        double traffic_kbytes_s = measured_frontend_kbytes_s();
        char json_body[1536];
        snprintf(json_body, sizeof(json_body),
            "{\"status\":\"%s\",\"freq_start\":%.0f,\"freq_end\":%.0f,"
            "\"configured_start_hz\":%.0f,\"configured_end_hz\":%.0f,"
            "\"visible_start_hz\":%.0f,\"visible_end_hz\":%.0f,"
            "\"converter_freq\":%.0f,\"steps\":%d,\"step_hz\":%.0f,"
            "\"required_points\":%d,\"mode\":\"%s\","
            "\"min_rate_lps\":%u,\"rate_limit_lps\":%u,"
            "\"rate_drop_factor\":%d,\"raw_line_rate\":%.3f,"
            "\"traffic_kbytes_s\":%.1f,"
            "\"display_bins\":%d,\"view_id\":%u,\"effective_fft_size\":%d,"
            "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,"
            "\"effective_input_samples\":%u,\"direct_sampling\":%u,"
            "\"clk_source\":%u,\"samplerate\":%.0f,\"bw_ratio\":%.2f,"
            "\"direct_sampling_max_hz\":%.0f}",
            status,
            g_freq_start, g_freq_end,
            g_freq_start, g_freq_end,
            g_visible_start, g_visible_end,
            g_converter_freq, total_steps, step,
            planned_required_points(), run_mode_name(mode),
            g_min_rate_lps, g_rate_limit_lps, rate_drop_factor, raw_line_rate,
            traffic_kbytes_s,
            current_display_bins(), g_view_id, effective_fft,
            decim_factor, decim_hop, overlap_factor, line_sample_count,
            g_direct_sampling, g_clk_source, g_samplerate, g_bw_ratio,
            direct_sampling_max_hz());
        send_json_response(client_fd, 200, "OK", cors, json_body);
        close(client_fd);
        return;
    }

    /* POST /api/view */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/view") == 0) {
        double visible_start = g_visible_start;
        double visible_end = g_visible_end;
        uint32_t display_tmp = 0;
        int visible_changed;
        int ret = 0;
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        #define GET_DOUBLE_RAW(key, target) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    double v; \
                    if (sscanf(k+1, " %lf", &v) == 1) \
                        (target) = v; \
                } \
            } \
        } while(0)
        #define GET_UINT_RAW(key, target) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    unsigned int v; \
                    if (sscanf(k+1, " %u", &v) == 1) \
                        (target) = (uint32_t)v; \
                } \
            } \
        } while(0)

        GET_DOUBLE_RAW("\"visible_start_hz\"", visible_start);
        GET_DOUBLE_RAW("\"visible_end_hz\"", visible_end);
        GET_UINT_RAW("\"display_bins\"", display_tmp);

        #undef GET_DOUBLE_RAW
        #undef GET_UINT_RAW

        if (display_tmp > 0)
            g_display_bins = normalize_display_bins((int)display_tmp);

        if (visible_start < g_freq_start) visible_start = g_freq_start;
        if (visible_end > g_freq_end) visible_end = g_freq_end;
        if (visible_end <= visible_start) {
            visible_start = g_freq_start;
            visible_end = g_freq_end;
        }

        visible_changed = fabs(visible_start - g_visible_start) >= 1.0 ||
                          fabs(visible_end - g_visible_end) >= 1.0;
        if (visible_changed) {
            g_visible_start = visible_start;
            g_visible_end = visible_end;
            g_view_id++;
            if (g_scanning)
                ret = start_scan();
        }

        {
            double step = 0.0;
            double scan_end = 0.0;
            int total_steps = current_scan_plan(&step, &scan_end);
            run_mode_t mode = planned_run_mode();
            int effective_fft = current_effective_fft_size();
            int decim_factor = current_decim_factor();
            int decim_hop = current_decim_hop();
            double overlap_factor = current_overlap_factor();
            uint32_t line_sample_count = current_line_sample_count();
            double raw_line_rate = 0.0;
            int rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                             total_steps,
                                                             g_rate_limit_lps,
                                                             &raw_line_rate);
            double traffic_kbytes_s = measured_frontend_kbytes_s();
            char json_body[1536];
            snprintf(json_body, sizeof(json_body),
                "{\"status\":\"%s\",\"view_id\":%u,"
                "\"freq_start\":%.0f,\"freq_end\":%.0f,"
                "\"configured_start_hz\":%.0f,\"configured_end_hz\":%.0f,"
                "\"visible_start_hz\":%.0f,\"visible_end_hz\":%.0f,"
                "\"steps\":%d,\"step_hz\":%.0f,\"display_bins\":%d,"
                "\"required_points\":%d,\"mode\":\"%s\","
                "\"min_rate_lps\":%u,\"rate_limit_lps\":%u,"
                "\"rate_drop_factor\":%d,\"raw_line_rate\":%.3f,"
                "\"traffic_kbytes_s\":%.1f,"
                "\"effective_fft_size\":%d,\"decim_factor\":%d,"
                "\"decim_hop\":%d,\"overlap_factor\":%.3f,"
                "\"effective_input_samples\":%u,\"scanning\":%d,"
                "\"direct_sampling\":%u,\"clk_source\":%u,"
                "\"direct_sampling_max_hz\":%.0f}",
                (ret == 0) ? "ok" : "error", g_view_id,
                g_freq_start, g_freq_end,
                g_freq_start, g_freq_end,
                g_visible_start, g_visible_end,
                total_steps, step, current_display_bins(),
                planned_required_points(), run_mode_name(mode),
                g_min_rate_lps, g_rate_limit_lps, rate_drop_factor, raw_line_rate,
                traffic_kbytes_s,
                effective_fft, decim_factor, decim_hop, overlap_factor, line_sample_count,
                g_scanning, g_direct_sampling, g_clk_source,
                direct_sampling_max_hz());
            send_json_response(client_fd, 200, "OK", cors, json_body);
        }
        close(client_fd);
        return;
    }

    /* POST /api/fft */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/fft") == 0) {
        uint32_t fft_tmp = 0;
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        const char *k = strstr(body, "fft_size");
        if (k) {
            k = strchr(k, ':');
            if (k) {
                unsigned int v;
                if (sscanf(k + 1, " %u", &v) == 1)
                    fft_tmp = (uint32_t)v;
            }
        }

        if (fft_tmp >= FFT_SIZE_MIN && fft_tmp <= FFT_SIZE_MAX) {
            update_fft_size((int)fft_tmp);
            save_config();
            char json_body[384];
            snprintf(json_body, sizeof(json_body),
                "{\"status\":\"ok\",\"fft_size\":%d,\"effective_fft_size\":%d,"
                "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,"
                "\"scanning\":%d}",
                current_fft_size(), current_effective_fft_size(),
                current_decim_factor(), current_decim_hop(), current_overlap_factor(),
                g_scanning);
            send_json_response(client_fd, 200, "OK", cors, json_body);
        } else {
            char json_body[256];
            snprintf(json_body, sizeof(json_body),
                "{\"status\":\"error\",\"fft_size\":%d}",
                current_fft_size());
            send_json_response(client_fd, 400, "Bad Request", cors, json_body);
        }
        close(client_fd);
        return;
    }

    /* POST /api/gain */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/gain") == 0) {
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        #define GET_UINT_LIMITED(key, target, maxval) do { \
            const char *k = strstr(body, key); \
            if (k) { \
                k = strchr(k, ':'); \
                if (k) { \
                    unsigned int v; \
                    if (sscanf(k+1, " %u", &v) == 1) { \
                        if (v > (maxval)) v = (maxval); \
                        (target) = (uint32_t)v; \
                    } \
                } \
            } \
        } while(0)

        GET_UINT_LIMITED("\"lna_gain\"", g_lna_gain, 2);
        GET_UINT_LIMITED("\"vga_gain\"", g_vga_gain, 15);

        #undef GET_UINT_LIMITED

        int ret = apply_gain_settings();
        const char *status = (ret == FOBOS_ERR_OK) ? "ok" : "error";
        char json_body[256];
        snprintf(json_body, sizeof(json_body),
            "{\"status\":\"%s\",\"lna_gain\":%u,\"vga_gain\":%u}",
            status, g_lna_gain, g_vga_gain);
        send_json_response(client_fd, 200, "OK", cors, json_body);
        close(client_fd);
        return;
    }

    /* POST /api/rate */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/rate") == 0) {
        uint32_t rate_tmp = 0;
        int ret = 0;
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        const char *k = strstr(body, "rate_limit_lps");
        if (k) {
            k = strchr(k, ':');
            if (k) {
                unsigned int v;
                if (sscanf(k + 1, " %u", &v) == 1)
                    rate_tmp = (uint32_t)v;
            }
        }

        if (rate_tmp > 0) {
            g_rate_limit_lps = normalize_rate_limit_lps(rate_tmp);
            save_config();
            if (g_scanning)
                ret = start_scan();
        }

        {
            double step = 0.0;
            double scan_end = 0.0;
            int total_steps = current_scan_plan(&step, &scan_end);
            run_mode_t mode = planned_run_mode();
            int effective_fft = current_effective_fft_size();
            int decim_factor = current_decim_factor();
            int decim_hop = current_decim_hop();
            double overlap_factor = current_overlap_factor();
            uint32_t line_sample_count = current_line_sample_count();
            double raw_line_rate = 0.0;
            int rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                             total_steps,
                                                             g_rate_limit_lps,
                                                             &raw_line_rate);
            double traffic_kbytes_s = measured_frontend_kbytes_s();
            char json_body[768];
            snprintf(json_body, sizeof(json_body),
                "{\"status\":\"%s\",\"min_rate_lps\":%u,\"rate_limit_lps\":%u,"
                "\"rate_drop_factor\":%d,\"raw_line_rate\":%.3f,"
                "\"traffic_kbytes_s\":%.1f,"
                "\"mode\":\"%s\",\"effective_fft_size\":%d,"
                "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,"
                "\"effective_input_samples\":%u,"
                "\"scanning\":%d}",
                (ret == 0) ? "ok" : "error",
                g_min_rate_lps, g_rate_limit_lps, rate_drop_factor, raw_line_rate,
                traffic_kbytes_s,
                run_mode_name(mode), effective_fft,
                decim_factor, decim_hop, overlap_factor, line_sample_count,
                g_scanning);
            send_json_response(client_fd, 200, "OK", cors, json_body);
        }
        close(client_fd);
        return;
    }

    /* POST /api/min-rate */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/min-rate") == 0) {
        uint32_t min_rate_tmp = 0;
        int ret = 0;
        const char *body = strstr(req, "\r\n\r\n");
        if (!body) { close(client_fd); return; }
        body += 4;

        const char *k = strstr(body, "min_rate_lps");
        if (k) {
            k = strchr(k, ':');
            if (k) {
                unsigned int v;
                if (sscanf(k + 1, " %u", &v) == 1)
                    min_rate_tmp = (uint32_t)v;
            }
        }

        g_min_rate_lps = normalize_min_rate_lps(min_rate_tmp);
        save_config();
        if (g_scanning && planned_run_mode() == RUN_MODE_SINGLE)
            ret = start_scan();

        {
            double step = 0.0;
            double scan_end = 0.0;
            int total_steps = current_scan_plan(&step, &scan_end);
            run_mode_t mode = planned_run_mode();
            int effective_fft = current_effective_fft_size();
            int decim_factor = current_decim_factor();
            int decim_hop = current_decim_hop();
            double overlap_factor = current_overlap_factor();
            uint32_t line_sample_count = current_line_sample_count();
            double raw_line_rate = 0.0;
            int rate_drop_factor = rate_drop_factor_for_plan(g_samplerate, line_sample_count,
                                                             total_steps,
                                                             g_rate_limit_lps,
                                                             &raw_line_rate);
            double traffic_kbytes_s = measured_frontend_kbytes_s();
            char json_body[768];
            snprintf(json_body, sizeof(json_body),
                "{\"status\":\"%s\",\"min_rate_lps\":%u,"
                "\"rate_drop_factor\":%d,\"raw_line_rate\":%.3f,"
                "\"traffic_kbytes_s\":%.1f,"
                "\"mode\":\"%s\",\"effective_fft_size\":%d,"
                "\"decim_factor\":%d,\"decim_hop\":%d,\"overlap_factor\":%.3f,"
                "\"effective_input_samples\":%u,\"scanning\":%d}",
                (ret == 0) ? "ok" : "error",
                g_min_rate_lps, rate_drop_factor, raw_line_rate,
                traffic_kbytes_s,
                run_mode_name(mode), effective_fft,
                decim_factor, decim_hop, overlap_factor,
                line_sample_count, g_scanning);
            send_json_response(client_fd, 200, "OK", cors, json_body);
        }
        close(client_fd);
        return;
    }

    /* POST /api/stop */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/stop") == 0) {
        stop_scan();
        send_json_response(client_fd, 200, "OK", cors, "{\"status\":\"ok\"}");
        close(client_fd);
        return;
    }

    /* 404 */
    const char *nf = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
    WRITE(client_fd, nf, strlen(nf));
    close(client_fd);
}

/* ------------------------------------------------------------------ */
/* Main                                                               */
/* ------------------------------------------------------------------ */
static volatile int g_exit = 0;
static void sigint_handler(int sig) { (void)sig; g_exit = 1; }

static int create_http_server_socket(void)
{
    int fd = socket(AF_INET6, SOCK_STREAM, 0);
    int opt = 1;
    int v6only = 0;

    if (fd >= 0) {
        struct sockaddr_in6 addr6;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only));

        memset(&addr6, 0, sizeof(addr6));
        addr6.sin6_family = AF_INET6;
        addr6.sin6_addr = in6addr_any;
        addr6.sin6_port = htons(PORT);

        if (bind(fd, (struct sockaddr *)&addr6, sizeof(addr6)) == 0 &&
            listen(fd, MAX_CLIENTS) == 0)
            return fd;

        close(fd);
    }

    fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd >= 0) {
        struct sockaddr_in addr;
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(PORT);

        if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0 &&
            listen(fd, MAX_CLIENTS) == 0)
            return fd;

        close(fd);
    }

    return -1;
}

static long long now_msec(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (long long)tv.tv_sec * 1000LL + (long long)tv.tv_usec / 1000LL;
}

static void poll_device_reconnect(void)
{
    static long long last_poll = 0;
    long long now = now_msec();

    if (g_dev || g_scanning || now - last_poll < 5000)
        return;

    last_poll = now;
    if (open_first_device(0) == FOBOS_ERR_OK) {
        printf("[SDR] Device reconnected: %s %s\n", g_manufacturer, g_product);
        printf("[SDR]   HW: %s  FW: %s  S/N: %s\n",
               g_hw_rev, g_fw_ver, g_serial);
    }
}

int main(int argc, char **argv)
{
    (void)argc;
    chdir_to_executable_dir(argv ? argv[0] : NULL);

    signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN);

    load_config();

    init_window();
    memset(g_sse_fds, 0, sizeof(g_sse_fds));

    /* Open device to read info */
    {
        char lib_ver[64], drv_ver[64];
        fobos_sdr_get_api_info(lib_ver, drv_ver);
        printf("[SDR] API: %s (drv: %s)\n", lib_ver, drv_ver);

        open_first_device(1);
    }

    int server_fd = create_http_server_socket();
    if (server_fd < 0) {
        perror("bind/listen");
        if (g_dev) {
            fobos_sdr_close(g_dev);
            g_dev = NULL;
        }
        return 1;
    }

    {
        char url[64];
        snprintf(url, sizeof(url), "http://localhost:%d", PORT);

        printf("\n");
        printf("╔══════════════════════════════════════════╗\n");
        printf("║ %-40s ║\n", "Fobos SDR Scanner");
        printf("║ %-40s ║\n", url);
        printf("╚══════════════════════════════════════════╝\n");
        printf("\n");
    }

    printf("[SDR] Waiting for scan start from web UI\n");

    while (!g_exit) {
        struct sockaddr_storage client_addr;
        socklen_t client_len = sizeof(client_addr);
        fd_set readfds;
        struct timeval wait_to;
        int ready;
        int client_fd;

        poll_device_reconnect();

        FD_ZERO(&readfds);
        FD_SET(server_fd, &readfds);
        wait_to.tv_sec = 1;
        wait_to.tv_usec = 0;
        ready = select(server_fd + 1, &readfds, NULL, NULL, &wait_to);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("select"); break;
        }
        if (ready == 0)
            continue;

        client_fd = accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            perror("accept"); break;
        }

        struct timeval to;
        to.tv_sec = 5; to.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));

        char req[MAX_REQUEST + 1] = {0};
        int total = 0;
        int header_len = 0;
        int body_len = 0;

        while (total < MAX_REQUEST) {
            int n = (int)read(client_fd, req + total, (size_t)(MAX_REQUEST - total));
            if (n <= 0) break;
            total += n;
            req[total] = 0;
            if (!header_len) {
                char *header_end = strstr(req, "\r\n\r\n");
                if (header_end) {
                    header_len = (int)(header_end - req) + 4;
                    body_len = http_content_length(req);
                }
            }
            if (header_len && total >= header_len + body_len)
                break;
        }

        if (total > 0) handle_request(client_fd, req);
        else close(client_fd);
    }

    stop_scan();
    if (g_dev) fobos_sdr_close(g_dev);
    close(server_fd);
    printf("\n[Server] Shutdown.\n");
    return 0;
}
