#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef _LINUX_
#include <unistd.h>
#include <signal.h>
#else
#include <windows.h>
#endif

#include <vtrenderlib.h>

static struct vtr_canvas* g_vt;

static void restore_tty_attrs(void)
{
    vtr_close(g_vt);
}

#ifdef _LINUX_
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
#endif

#ifdef _WIN32_
static void usleep(unsigned long long us)
{
    Sleep(us / 1000);
}
#endif

int main(void)
{
    int error;

#ifdef _LINUX_
    g_vt = vtr_canvas_create(STDOUT_FILENO);
    signal(SIGINT, handle_signal);
    signal(SIGWINCH, handle_signal);
#else
    SetConsoleOutputCP(CP_UTF8);
    g_vt = vtr_canvas_create(GetStdHandle(STD_OUTPUT_HANDLE));
#endif
    if (!g_vt) {
        exit(EXIT_FAILURE);
    }

    atexit(restore_tty_attrs);

    error = vtr_reset(g_vt);
    if (error) {
        exit(error);
    }

    uint32_t wbox = 150, hbox = 80, margin = 4, charw = wbox / 3;
    int x = 0, y = 0;
    int xdir = 1, ydir = 1;

    while(true) {
        vtr_resize(g_vt);

        vtr_scan_linec(g_vt, x + margin, y + hbox - margin, x + charw / 2, y + margin, VTR_COLOR_RED);
        vtr_scan_linec(g_vt, x + charw / 2, y + margin, x + charw - margin, y + hbox - margin, VTR_COLOR_GREEN);

        vtr_scan_linec(g_vt, x + charw + margin, y + margin, x + charw * 2 - margin, y + margin, VTR_COLOR_YELLOW);
        vtr_scan_linec(g_vt, x + charw + margin, y + margin, x + charw + margin, y + hbox - margin, VTR_COLOR_BLUE);
        vtr_scan_linec(g_vt, x + charw + margin, y + hbox - margin, x + charw * 2 - margin, y + hbox - margin, VTR_COLOR_MAGENTA);
        vtr_scan_linec(g_vt, x + charw * 2 - margin, y + hbox - margin, x + charw * 2 - margin, y + margin, VTR_COLOR_CYAN);

        vtr_scan_linec(g_vt, x + charw * 2 + margin, y + margin, x + charw * 3 - margin, y + hbox - margin, VTR_COLOR_WHITE);
        vtr_scan_linec(g_vt, x + charw * 3 - margin, y + margin, x + charw * 2 + margin, y + hbox - margin, VTR_COLOR_DEFAULT);

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
