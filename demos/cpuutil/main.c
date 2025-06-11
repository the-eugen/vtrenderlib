#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

#include <vtrenderlib.h>

#define TICK_HZ 100

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

struct cpu_times
{
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;
};

static struct cpu_times g_tlast;
static double* g_util_history;
static size_t g_history_depth;
static size_t g_history_pos;
static double g_utilavg;
static double g_decay_factor;
static struct vtr_canvas* g_vt;

static int read_cpu_times(struct cpu_times *t)
{
    FILE *f = fopen("/proc/stat", "r");
    if (!f) return -1;
    char buf[256];
    if (!fgets(buf, sizeof(buf), f)) {
        fclose(f);
        return -1;
    }
    fclose(f);

    sscanf(buf, "cpu  %llu %llu %llu %llu %llu %llu %llu %llu",
           &t->user, &t->nice, &t->system, &t->idle,
           &t->iowait, &t->irq, &t->softirq, &t->steal);

    return 0;
}

static void update(uint32_t dtimems)
{
    struct cpu_times t;
    if (read_cpu_times(&t) < 0) {
        return;
    }

    unsigned long long idle1 = g_tlast.idle + g_tlast.iowait;
    unsigned long long idle2 = t.idle + t.iowait;
    unsigned long long busy1 = g_tlast.user + g_tlast.nice + g_tlast.system;
    unsigned long long busy2 = t.user + t.nice + t.system;

    unsigned long long total1 = idle1 + busy1;
    unsigned long long total2 = idle2 + busy2;

    double u = (double)(busy2 - busy1) / (double)(total2 - total1);
    double uavg = g_utilavg * g_decay_factor + u * (1.0 - g_decay_factor);

    g_util_history[g_history_pos] = uavg;
    g_history_pos = (g_history_pos + 1) % g_history_depth;
    g_tlast = t;
    g_utilavg = uavg;
}

static void draw(void)
{
    for (size_t i = 0; i < g_history_depth; i++) {
        double u = g_util_history[(g_history_pos + i) % g_history_depth];
        uint16_t x = (uint16_t) (vtr_xdots(g_vt) - i - 1);
        uint16_t h = (uint16_t) ((double)vtr_ydots(g_vt) * u);

        if (h > 0) {
            vtr_scan_line(g_vt, x, vtr_ydots(g_vt) - h - 1, x, vtr_ydots(g_vt) - 1);
        } else {
            vtr_render_dot(g_vt, x, vtr_ydots(g_vt) - 1);
        }
    }
}

static void restore_tty_attrs(void)
{
    vtr_close(g_vt);
}

static void handle_signal(int signo)
{
    if (signo == SIGWINCH) {
        vtr_set_resize_pending(g_vt);
    } else {
        restore_tty_attrs();
        signal(signo, SIG_DFL);
        raise(signo);
    }
}

int main(void)
{
    int error;

    g_vt = vtr_canvas_create(STDOUT_FILENO);
    if (!g_vt) {
        exit(EXIT_FAILURE);
    }

    atexit(restore_tty_attrs);
    signal(SIGINT, handle_signal);
    signal(SIGWINCH, handle_signal);

    error = vtr_reset(g_vt);
    if (error) {
        exit(error);
    }

    g_history_depth = vtr_xdots(g_vt);
    g_util_history = calloc(sizeof(double), g_history_depth);
    if (!g_util_history) {
        exit(ENOMEM);
    }

    g_decay_factor = exp(-(1.0f / TICK_HZ));

    while (true) {
        vtr_resize(g_vt);
        update(1000 / TICK_HZ);
        draw();
        vtr_swap_buffers(g_vt);
        usleep(1000000 / TICK_HZ);
    }

    return 0;
}
