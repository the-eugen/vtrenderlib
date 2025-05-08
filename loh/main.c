#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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
    restore_tty_attrs();
    signal(signo, SIG_DFL);
    raise(signo);
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

    error = vtr_reset(g_vt);
    if (error) {
        exit(error);
    }

    uint32_t wbox = 150, hbox = 80, margin = 4, charw = wbox / 3;
    int x = 0, y = 0;
    int xdir = 1, ydir = 1;

    while(true) {
        vtr_scan_line(g_vt, x + margin, y + hbox - margin, x + charw / 2, y + margin);
        vtr_scan_line(g_vt, x + charw / 2, y + margin, x + charw - margin, y + hbox - margin);

        vtr_scan_line(g_vt, x + charw + margin, y + margin, x + charw * 2 - margin, y + margin);
        vtr_scan_line(g_vt, x + charw + margin, y + margin, x + charw + margin, y + hbox - margin);
        vtr_scan_line(g_vt, x + charw + margin, y + hbox - margin, x + charw * 2 - margin, y + hbox - margin);
        vtr_scan_line(g_vt, x + charw * 2 - margin, y + hbox - margin, x + charw * 2 - margin, y + margin);

        vtr_scan_line(g_vt, x + charw * 2 + margin, y + margin, x + charw * 3 - margin, y + hbox - margin);
        vtr_scan_line(g_vt, x + charw * 3 - margin, y + margin, x + charw * 2 + margin, y + hbox - margin);

        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);

        x += xdir * 1; y += ydir * 1;

        if (x <= 0) {
            xdir = 1;
        } else if (x + wbox >= vtr_xdots(g_vt)) {
            xdir = -1;
        }

        if (y <= 0) {
            ydir = 1;
        } else if (y + hbox >= vtr_ydots(g_vt)) {
            ydir = -1;
        }
    }

    return 0;
}
