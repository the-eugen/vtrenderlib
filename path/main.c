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

    struct vtr_vertex vlist[] = {
        {10, 10},
        {30, 25},
        {60, 25},
        {80, 10},
        {75, 40},
        {66, 60},
        {45, 65},
        {24, 60},
        {15, 40}
    };

    while(true) {
        vtr_resize(g_vt);

        vtr_trace_poly(g_vt, sizeof(vlist) / sizeof(vlist[0]), vlist);

        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }
   
    return 0;
}

