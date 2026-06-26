/*
 * Fobos SDR Scanner - Backend
 *
 * Scanning algorithm:
 *   Build a frequency table from freq_start to freq_end.
 *   Let the Fobos SDR firmware scan that table in hardware.
 *   For each indexed scan buffer, compute FFT magnitude and store it in
 *   that frequency slot. One horizontal waterfall line is published after
 *   every frequency slot has contributed.
 */

#define _USE_MATH_DEFINES
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/time.h>
#include <errno.h>
#include <fobos_sdr.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ------------------------------------------------------------------ */
/* Config                                                             */
/* ------------------------------------------------------------------ */
#define PORT                8080
#define MAX_HEADERS         4096
#define MAX_CLIENTS         64
#define MAX_SSE_CLIENTS     16
#define MAX_FREQS           256
#define MAX_PATH            1024
#define HTML_PATH           "index.html"
#define MIN_FREQ_START_HZ   50.0e6

/* FFT: 1024 complex samples per read (~32 us at 50 MHz) */
#define FFT_SIZE            1024
#define FFT_LOG2            10
#define BINS_PER_STEP       (FFT_SIZE / 2)
#define FFTS_PER_STEP       32
#define SCAN_BUF_LEN        65536

/* Max total bins per line (256 freq steps * 512 bins) */
#define MAX_BINS_PER_LINE   (MAX_FREQS * BINS_PER_STEP)

/* ------------------------------------------------------------------ */
/* Global state                                                       */
/* ------------------------------------------------------------------ */
static struct fobos_sdr_dev_t *g_dev = NULL;
static pthread_t g_scan_thread;
static volatile int g_scanning = 0;

/* Scan parameters */
static double g_freq_start   = MIN_FREQ_START_HZ;
static double g_freq_end     = 2000.0e6;
static double g_samplerate   = 50.0e6;
static double g_bw_ratio     = 0.9;
static uint32_t g_lna_gain   = 0;
static uint32_t g_vga_gain   = 0;
static uint32_t g_direct_sampling = 0;
static uint32_t g_clk_source __attribute__((unused)) = 0;

/* Device info */
static char g_hw_rev[64]    = "unknown";
static char g_fw_ver[64]    = "unknown";
static char g_serial[64]    = "unknown";
static char g_manufacturer[64] = "unknown";
static char g_product[64]   = "unknown";

/* SSE client list */
static int g_sse_fds[MAX_SSE_CLIENTS];
static pthread_mutex_t g_sse_mutex = PTHREAD_MUTEX_INITIALIZER;

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

static void sse_broadcast(const char *data, int len)
{
    pthread_mutex_lock(&g_sse_mutex);
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        int fd = g_sse_fds[i];
        if (fd > 0) {
            if (write_all(fd, data, (size_t)len) != 0) {
                close(fd); g_sse_fds[i] = 0;
            }
        }
    }
    pthread_mutex_unlock(&g_sse_mutex);
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

/* Hann window */
static float g_window[FFT_SIZE];
static void init_window(void)
{
    for (int i = 0; i < FFT_SIZE; i++)
        g_window[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (float)(FFT_SIZE - 1)));
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

/* ------------------------------------------------------------------ */
/* Hardware scan context                                              */
/* ------------------------------------------------------------------ */
typedef struct {
    int total_steps;
    int steps_seen;
    int line_num;
    double freq_start;
    double freq_end;
    float *line_buf;
    uint8_t *line_packed;
    uint8_t step_seen[MAX_FREQS];
} scan_ctx_t;

static int build_scan_frequencies(double *freqs, double *out_step)
{
    double step = g_samplerate * g_bw_ratio;
    int count = 0;

    if (out_step) *out_step = step;
    if (step <= 0.0 || g_freq_end <= g_freq_start) return 0;

    while (count < MAX_FREQS) {
        double freq = g_freq_start + (double)count * step;
        if (freq > g_freq_end) break;
        freqs[count++] = freq;
    }

    if (count == 1 && g_freq_end > g_freq_start)
        freqs[count++] = g_freq_end;

    return count;
}

static int average_fft_magnitude(float *buf, uint32_t buf_len, float *out)
{
    float local_fft[FFT_SIZE * 2];
    int fft_count = 0;

    memset(out, 0, (size_t)BINS_PER_STEP * sizeof(float));

    for (uint32_t pos = 0; pos + FFT_SIZE <= buf_len && fft_count < FFTS_PER_STEP; pos += FFT_SIZE) {
        for (int i = 0; i < FFT_SIZE; i++) {
            float w = g_window[i];
            local_fft[2*i]   = buf[2*(pos+i)]   * w;
            local_fft[2*i+1] = buf[2*(pos+i)+1] * w;
        }

        fft_c2c(local_fft, FFT_SIZE);

        for (int i = 0; i < BINS_PER_STEP; i++) {
            float re = local_fft[2*i];
            float im = local_fft[2*i+1];
            out[i] += sqrtf(re*re + im*im);
        }
        fft_count++;
    }

    if (fft_count <= 0) return 0;

    float inv = 1.0f / (float)fft_count;
    for (int i = 0; i < BINS_PER_STEP; i++)
        out[i] *= inv;

    return fft_count;
}

static void publish_scan_line(scan_ctx_t *ctx)
{
    int bin_count = ctx->total_steps * BINS_PER_STEP;

    float mag_max = 1e-15f;
    for (int i = 0; i < bin_count; i++)
        if (ctx->line_buf[i] > mag_max) mag_max = ctx->line_buf[i];

    float scale = 255.0f / mag_max;
    for (int i = 0; i < bin_count; i++) {
        float v = ctx->line_buf[i] * scale;
        ctx->line_packed[i] = (uint8_t)(v > 255.0f ? 255 : v);
    }

    char *json = malloc((size_t)bin_count * 5 + 256);
    if (!json) return;

    int pos = snprintf(json, 256,
        "event: line\ndata: {\"n\":%d,\"b\":%d,\"f0\":%.0f,\"f1\":%.0f,\"d\":[",
        ctx->line_num, ctx->total_steps, ctx->freq_start, ctx->freq_end);

    for (int i = 0; i < bin_count; i++)
        pos += snprintf(json + pos, 5, "%s%d", (i ? "," : ""), ctx->line_packed[i]);

    pos += snprintf(json + pos, 16, "]}\n\n");
    sse_broadcast(json, pos);
    free(json);
}

static void scan_callback(float *buf, uint32_t buf_len, struct fobos_sdr_dev_t *dev, void *user)
{
    scan_ctx_t *ctx = (scan_ctx_t *)user;
    int channel;
    float *slot;

    if (!g_scanning) {
        fobos_sdr_cancel_async(dev);
        return;
    }

    channel = fobos_sdr_get_scan_index(dev);
    if (channel < 0 || channel >= ctx->total_steps)
        return;

    slot = ctx->line_buf + (size_t)channel * BINS_PER_STEP;
    if (average_fft_magnitude(buf, buf_len, slot) <= 0)
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

/* ------------------------------------------------------------------ */
/* Scanning thread — the core algorithm                               */
/* ------------------------------------------------------------------ */
static void *scan_thread_func(void *arg)
{
    (void)arg;
    double freqs[MAX_FREQS];
    double step = 0.0;
    int total_steps = build_scan_frequencies(freqs, &step);
    scan_ctx_t ctx;
    int ret;
    size_t bin_count;

    if (total_steps < FOBOS_MIN_FREQS_CNT) {
        fprintf(stderr, "[SDR] Hardware scan needs at least %d frequencies\n", FOBOS_MIN_FREQS_CNT);
        g_scanning = 0;
        return NULL;
    }

    memset(&ctx, 0, sizeof(ctx));
    ctx.total_steps = total_steps;
    ctx.freq_start = freqs[0];
    ctx.freq_end = freqs[total_steps - 1];

    bin_count = (size_t)total_steps * BINS_PER_STEP;
    ctx.line_buf = malloc(bin_count * sizeof(float));
    ctx.line_packed = malloc(bin_count);
    if (!ctx.line_buf || !ctx.line_packed) {
        fprintf(stderr, "[SDR] Out of memory\n");
        free(ctx.line_buf);
        free(ctx.line_packed);
        g_scanning = 0;
        return NULL;
    }

    ret = fobos_sdr_set_samplerate(g_dev, g_samplerate);
    if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_samplerate failed: %d\n", ret);
    ret = fobos_sdr_set_auto_bandwidth(g_dev, g_bw_ratio);
    if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_auto_bandwidth failed: %d\n", ret);
    ret = fobos_sdr_set_lna_gain(g_dev, g_lna_gain);
    if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_lna_gain failed: %d\n", ret);
    ret = fobos_sdr_set_vga_gain(g_dev, g_vga_gain);
    if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_vga_gain failed: %d\n", ret);
    ret = fobos_sdr_set_direct_sampling(g_dev, g_direct_sampling);
    if (ret != FOBOS_ERR_OK) fprintf(stderr, "[SDR] set_direct_sampling failed: %d\n", ret);

    printf("[SDR] Hardware scan: %.0f - %.0f Hz, requested step %.0f Hz (%d freqs)\n",
           ctx.freq_start, ctx.freq_end, step, total_steps);

    ret = fobos_sdr_start_scan(g_dev, freqs, (unsigned int)total_steps);
    if (ret != FOBOS_ERR_OK) {
        fprintf(stderr, "[SDR] fobos_sdr_start_scan failed: %d\n", ret);
        g_scanning = 0;
    }

    if (g_scanning) {
        ret = fobos_sdr_read_async(g_dev, scan_callback, &ctx, 16, SCAN_BUF_LEN);
        if (ret != FOBOS_ERR_OK && g_scanning)
            fprintf(stderr, "[SDR] fobos_sdr_read_async failed: %d\n", ret);
    }

    fobos_sdr_stop_scan(g_dev);
    g_scanning = 0;
    free(ctx.line_buf);
    free(ctx.line_packed);
    printf("[SDR] Scan thread stopped\n");
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Start / Stop scan                                                  */
/* ------------------------------------------------------------------ */
static int start_scan(void)
{
    double freqs[MAX_FREQS];

    if (g_scanning) return 0;
    if (build_scan_frequencies(freqs, NULL) < FOBOS_MIN_FREQS_CNT) return -1;

    if (!g_dev) {
        if (fobos_sdr_get_device_count() <= 0) return -1;
        if (fobos_sdr_open(&g_dev, 0) != FOBOS_ERR_OK) return -1;
    }

    g_scanning = 1;
    if (pthread_create(&g_scan_thread, NULL, scan_thread_func, NULL) != 0) {
        g_scanning = 0;
        return -1;
    }
    return 0;
}

static void stop_scan(void)
{
    if (!g_scanning) return;
    g_scanning = 0;
    fobos_sdr_cancel_async(g_dev);
    pthread_join(g_scan_thread, NULL);
}

static int apply_gain_settings(void)
{
    int ret_lna = FOBOS_ERR_OK;
    int ret_vga = FOBOS_ERR_OK;

    if (!g_dev || !g_scanning)
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

    /* GET /api/status */
    if (strcmp(method, "GET") == 0 && strcmp(path, "/api/status") == 0) {
        char body[2048];
        double freqs[MAX_FREQS];
        double step = 0.0;
        int total_steps = build_scan_frequencies(freqs, &step);

        int n = snprintf(body, sizeof(body),
            "{\"device\":\"Fobos SDR\","
            "\"hardware\":\"%s\",\"firmware\":\"%s\",\"serial\":\"%s\","
            "\"manufacturer\":\"%s\",\"product\":\"%s\","
            "\"scanning\":%d,"
            "\"freq_start\":%.0f,\"freq_end\":%.0f,"
            "\"samplerate\":%.0f,\"bw_ratio\":%.2f,"
            "\"step_hz\":%.0f,\"steps\":%d,"
            "\"bins_per_step\":%d,\"fft_size\":%d,"
            "\"lna_gain\":%u,\"vga_gain\":%u,\"direct_sampling\":%u}",
            g_hw_rev, g_fw_ver, g_serial,
            g_manufacturer, g_product,
            g_scanning,
            g_freq_start, g_freq_end,
            g_samplerate, g_bw_ratio,
            step, total_steps,
            BINS_PER_STEP, FFT_SIZE,
            g_lna_gain, g_vga_gain, g_direct_sampling);

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

        GET_DOUBLE("\"freq_start\"", g_freq_start, 1.0e6, 1.0);
        GET_DOUBLE("\"freq_end\"", g_freq_end, 1.0e6, 1.0);
        GET_DOUBLE("\"samplerate\"", g_samplerate, 1.0, 1.0);
        GET_DOUBLE("\"bw_ratio\"", g_bw_ratio, 1.0, 0.000001);
        GET_UINT("\"lna_gain\"", g_lna_gain);
        GET_UINT("\"vga_gain\"", g_vga_gain);
        GET_UINT("\"direct_sampling\"", g_direct_sampling);
        if (g_freq_start < MIN_FREQ_START_HZ) g_freq_start = MIN_FREQ_START_HZ;

        #undef GET_DOUBLE
        #undef GET_UINT

        int ret = start_scan();
        const char *status = (ret == 0) ? "ok" : "error";
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n%s\r\n"
            "{\"status\":\"%s\"}", cors, status);
        WRITE(client_fd, resp, (size_t)n);
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
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n%s\r\n"
            "{\"status\":\"%s\",\"lna_gain\":%u,\"vga_gain\":%u}",
            cors, status, g_lna_gain, g_vga_gain);
        WRITE(client_fd, resp, (size_t)n);
        close(client_fd);
        return;
    }

    /* POST /api/stop */
    if (strcmp(method, "POST") == 0 && strcmp(path, "/api/stop") == 0) {
        stop_scan();
        char resp[512];
        int n = snprintf(resp, sizeof(resp),
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n%s\r\n"
            "{\"status\":\"ok\"}", cors);
        WRITE(client_fd, resp, (size_t)n);
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

int main(void)
{
    signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN);

    init_window();
    memset(g_sse_fds, 0, sizeof(g_sse_fds));

    /* Open device to read info */
    {
        char lib_ver[64], drv_ver[64];
        fobos_sdr_get_api_info(lib_ver, drv_ver);
        printf("[SDR] API: %s (drv: %s)\n", lib_ver, drv_ver);

        int count = fobos_sdr_get_device_count();
        printf("[SDR] Devices found: %d\n", count);

        if (count > 0) {
            int ret = fobos_sdr_open(&g_dev, 0);
            if (ret == FOBOS_ERR_OK) {
                fobos_sdr_get_board_info(g_dev, g_hw_rev, g_fw_ver,
                                         g_manufacturer, g_product, g_serial);
                printf("[SDR] Device: %s %s\n", g_manufacturer, g_product);
                printf("[SDR]   HW: %s  FW: %s  S/N: %s\n",
                       g_hw_rev, g_fw_ver, g_serial);
                fobos_sdr_close(g_dev);
                g_dev = NULL;
            } else {
                printf("[SDR] Could not open device: %d\n", ret);
            }
        } else {
            printf("[SDR] No device connected.\n");
        }
    }

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) { perror("socket"); return 1; }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(server_fd); return 1;
    }
    if (listen(server_fd, MAX_CLIENTS) < 0) {
        perror("listen"); close(server_fd); return 1;
    }

    printf("\n");
    printf("╔══════════════════════════════════════════╗\n");
    printf("║   Fobos SDR Scanner                     ║\n");
    printf("║   http://localhost:%d              ║\n", PORT);
    printf("╚══════════════════════════════════════════╝\n");
    printf("\n");

    while (!g_exit) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (errno == EINTR) continue;
            perror("accept"); break;
        }

        struct timeval to;
        to.tv_sec = 5; to.tv_usec = 0;
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));

        char req[MAX_HEADERS + 1] = {0};
        int total = 0;
        while (total < MAX_HEADERS) {
            int n = (int)read(client_fd, req + total, (size_t)(MAX_HEADERS - total));
            if (n <= 0) break;
            total += n;
            req[total] = 0;
            if (strstr(req, "\r\n\r\n")) break;
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
