#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>

#include <vtrenderlib.h>

static struct vtr_canvas* g_vt;

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

    // Dots clipped
    {
        vtr_render_dot(g_vt, -1, -1);
        vtr_render_dot(g_vt, UINT16_MAX, UINT16_MAX);
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Move a triangle across the screen
    for (int y = 0; y < vtr_ydots(g_vt) + 10; y += 1) {
        struct vtr_vertex v3[] = {
            {50, y - 10},
            {45, y},
            {55, y},
        };

        vtr_trace_poly(g_vt, 3, v3);
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    for (int x = 0; x < vtr_xdots(g_vt) + 10; x += 1) {
        struct vtr_vertex v3[] = {
            {x - 10, 50},
            {x, 45},
            {x, 55},
        };

        vtr_trace_poly(g_vt, 3, v3);
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Move a vertical line across the canvas and clip it
    for (int x = -1; x <= vtr_xdots(g_vt); x += 1) {
        vtr_scan_line(g_vt, x, -1, x, vtr_ydots(g_vt));
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Move a horizontal line across the canvas and clip it
    for (int y = -1; y <= vtr_ydots(g_vt); y += 1) {
        vtr_scan_line(g_vt, -1, y, vtr_xdots(g_vt), y);
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Sloped line moving horizontally
    for (int x = -50; x <= vtr_xdots(g_vt); x += 1) {
        vtr_scan_line(g_vt, x, -1, x + 50, vtr_ydots(g_vt));
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Sloped line moving vertically
    for (int y = -50; y <= vtr_ydots(g_vt); y += 1) {
        vtr_scan_line(g_vt, -1, y, vtr_xdots(g_vt), y + 50);
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    // Half-segment moving across the screen
    for (int x = 0; x < vtr_xdots(g_vt); x += vtr_xdots(g_vt) / 4) {
        for (int y = 0; y < vtr_ydots(g_vt); y++) {
            vtr_scan_line(g_vt, x, y - vtr_ydots(g_vt) / 4, x + vtr_xdots(g_vt) / 2, y + vtr_ydots(g_vt) / 4);
            vtr_swap_buffers(g_vt);
            usleep(1000000 / 60);
        }
    }

    return 0;
}
