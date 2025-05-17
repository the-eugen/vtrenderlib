//
// TTY devices
// ------------
//
// - Kernel implementes PTYs (pseudoterminals) as a pair of chardevs: slave (e.g. /dev/pts/3) and master (e.g. /dev/ptmx).
//   Slave behaves like a classic serial-line tty.
//
// - Terminal emulator opens the PTY master which allocates a slave for that terminal. The terminal then forks a shell
//   and attaches STDIN/OUT/ERR to the allocated slave. The terminal emulator sits on the other end of the master/slave pair.
//
// - Shell/app can use ttyname to get its slave PTY node (or just STDOUT_FILENO).
//   Or "tty" from a shell directly.
//   Or "ll `readlink /dev/stdout`"
//
// - Bytes written by the terminal emulator into the master PTY appear on the slave FD as if typed by the user and vice versa.
//   User keystroke -> terminal emulator -> write to PTY master -> read from PTY slave -> deliver input to shell/app via STDIN.
//
// - VT/xterm escape sequences work in the same manner:
//   1. Shell/app: 0x1B 0x5B 0x33 0x31 0x6D   # ESC [ 3 1 m  → “set foreground color to red” -> written to PTY slave via STDOUT_FILENO
//   2. Kernel PTY layer: handles line discipline, raw mode (bytes pass through unchanged) vs canonical mode (control chars handled),
//      forwards data to PTY slave.
//   3. Terminal emulator: reads data from PTY slave, recognizes `ESC [ 31 m`, changes rendering color, paints to the screen.
//

//
// termios
// -------
//
// Below are some flag bits that make sense for a software emulated TTY
//
// c_iflag - Controls how input bytes are processed by PTY before being send to the slave
//
//   ICRNL - turns incoming \r into \n unless IGNCR is set. Disable it to see raw \r bytes.
//   INLCR - turns incoming \n into \r. 
//   IGNCR - ignores incoming \r.
//   IXON/IXOFF - Enable XON/XOFF flow control.
//                When enabled, ctrl+s (XOFF) will pause output from kernel, and ctrl+q (XON) will resume it.
//   IXANY - Any character will act as XON.
//   ISTRIP - Strip the MSB in each incoming byte. Turns input into 7-bit bytes.
//
// c_oflags - Controls how output bytes are processed by PTY after recieving them from the slave
//
//   OPOST - enable output processing.
//           when cleared all bytes are passed through as-is and all following oflags have no effect.
//   ONCLR - turns each outgoing \n into \r\n
//   OCRNL - turns each outgoing \r into \n
//   ONOCR - suppress \r at column 0, instead do nothing and drop the \r.
//           avoids unncecessary processing in PTY or something.
//   ONLRET - perform a carriage return on each \r.
//            almost the same as turning \r into \r\n but doesn't add a \n explicitly.
//   TABDLY (TAB0/TAB1/TAB2/TAB3/XTABS) - Linux specific, expands tabs to 0/1/2/8 spaces.
//                                        Historically was inserting line delays with tabs, not relevant anymore.
//   NL0 - drops every \n
//   NL1 - turns every \n into \r\n, similar to ONLCR.
//
// c_cflags - Line control flags. Most are noops for software TTYs.
//
//   CSIZE (CS5/CS6/CS7/CS8) - char bitsize mask.
//   CREAD - enable char ingress. when disabled the app will not see any input.
//   CLOCAL - enable local mode. when enabled will ignore modem control signals (like carrier-detect).
//   PARENB/PARODD - even/odd parity bits. just disable.
//   CSTOPB - two stop bits if set, 1 if cleared. just disable.
//   CRTSCTS - hardware RTS/CTS flow control. just disable.
//
// c_lflags - More input editing flags.
//
//   ICANON - canonical mode.
//            when set input is line-buffered, special-char editing is enabled.
//            when cleared input is raw and unbuffered.
//   ECHO - echo typed characters back to the terminal automatically.
//   ECHOE - If ICANON is set, typing backspace will erase the last char.
//   ECHOK - If ICANON is set, kill-line char (ctrl+u) erases the current line.
//   ECHONL - If ECHO is set, \n is still echoed.
//   ISIG - enable VINTR/VQUIT/VSUSP signals.
//   NOFLSH - don't flush the buffer in canonical mode on signals.
//   TOSTOP - background processes that attempt to write to the terminal will generate SIGSTOU
//   PENDIN - when a SIGUSP is received, TTY reprints any pending input after SIGCONT.
//   IEXTEN - Linux specific, enables special input processing.
//

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <unistd.h>
#include <termios.h>
#include <sys/unistd.h>
#include <sys/ioctl.h>

#include "vtrenderlib.h"

#ifndef NDEBUG
#define DBG
#endif

#ifdef DBG
#define DBG_LOG(fmt, ...) fprintf(stderr, fmt, ##__VA_ARGS__)
#else
#define DBG_LOG(fmt, ...)
#endif

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// Character cell dims in dots
#define VT_CELL_YDOTS ((uint16_t)4)
#define VT_CELL_XDOTS ((uint16_t)2)

struct vtr_stencil_buf
{
    uint16_t ydots;
    uint16_t xdots;
    uint8_t* buffer;
};

struct vtr_canvas
{
    int fd;
    struct termios origattrs;
    bool resize_pending;

    // Canvas dimentions in char cells
    uint16_t nrows;
    uint16_t ncols;

    // Canvas dimensions in dots
    uint16_t ydots;
    uint16_t xdots;

    // Double-buffered stencil
    struct vtr_stencil_buf sb[2];
    struct vtr_stencil_buf* cur_sb;

    // Bitmap buffer to cover xdots number of bit. Used for line scanning.
    uint8_t* linemap;

    // Escape sequence list buffer
    char* seqlist;
    size_t seqcap;
};

static int create_stencil_buf(struct vtr_stencil_buf* sb, uint16_t rows, uint16_t cols)
{
    sb->buffer = calloc(rows, cols);
    if (!sb->buffer) {
        return -1;
    }

    sb->xdots = cols * VT_CELL_XDOTS;
    sb->ydots = rows * VT_CELL_YDOTS;

    return 0;
}

static void free_stencil_buf(struct vtr_stencil_buf* sb)
{
    if (sb) {
        free(sb->buffer);
        sb->ydots = sb->xdots = 0;
        sb->buffer = NULL;
    }
}

struct vtr_canvas* vtr_canvas_create(int ttyfd)
{
    int error = 0;

    struct termios attrs;
    error = tcgetattr(ttyfd, &attrs);
    if (error) {
        return NULL;
    }

    struct winsize ws;
    error = ioctl(ttyfd, TIOCGWINSZ, &ws);
    if (error) {
        return NULL;
    }

    struct vtr_canvas* vt = malloc(sizeof(*vt));
    if (!vt) {
        return NULL;
    }

    struct vtr_stencil_buf sb1 = {0};
    struct vtr_stencil_buf sb2 = {0};
    char* seqlist = NULL;
    uint8_t* linemap = NULL;

    if (0 != create_stencil_buf(&sb1, ws.ws_row, ws.ws_col) || 0 != create_stencil_buf(&sb2, ws.ws_row, ws.ws_col)) {
        goto error_out;
    }

    linemap = malloc((ws.ws_col * VT_CELL_XDOTS + 7) >> 3);
    if (!linemap) {
        goto error_out;
    }

    // We preallocate enough space to hold the biggest number of draw calls (3 bytes per escape seq)
    // plus some slack to hold a cursor reset command (6 bytes) and some change.
    size_t seqcap = (size_t)((ws.ws_row + 1) * ws.ws_col + 1) * 3;
    seqlist = malloc(seqcap);
    if (!seqlist) {
        goto error_out;
    }

    vt->fd = ttyfd;
    vt->nrows = ws.ws_row;
    vt->ncols = ws.ws_col;
    vt->ydots = ws.ws_row * VT_CELL_YDOTS;
    vt->xdots = ws.ws_col * VT_CELL_XDOTS;
    vt->sb[0] = sb1;
    vt->sb[1] = sb2;
    vt->cur_sb = &vt->sb[0];
    vt->linemap = linemap;
    vt->seqlist = seqlist;
    vt->seqcap = seqcap;
    vt->resize_pending = false;
    memcpy(&vt->origattrs, &attrs, sizeof(attrs));

    return vt;

error_out:

    free_stencil_buf(&sb1);
    free_stencil_buf(&sb2);
    free(seqlist);
    free(linemap);
    free(vt);

    return NULL;
}

static int sendseq(int ttyfd, const char* seq, size_t nbytes)
{
    ssize_t res = write(ttyfd, seq, nbytes);
    if (res == -1 || res != nbytes) {
        return -1;
    }

    return 0;
}

int vtr_reset(struct vtr_canvas* vt)
{
    int error;

    struct termios attrs;
    error = tcgetattr(vt->fd, &attrs);
    if (error) {
        return error;
    }
    
    attrs.c_oflag &= ~OPOST;
    attrs.c_cflag &= ~CREAD;
    attrs.c_lflag &= ~(ICANON | ECHO | IEXTEN);

    error = tcsetattr(vt->fd, TCSANOW, &attrs);
    if (error) {
        return error;
    }

    // switch to alternate buffer and hide cursor
    error |= sendseq(vt->fd, "\x1B[?1049h", 8);
    error |= sendseq(vt->fd, "\x1B[?25l", 6);
    error |= sendseq(vt->fd, "\x1B[2J", 4);

    return error;
}

int vtr_resize(struct vtr_canvas* vt)
{
    if (!vt->resize_pending) {
        return 0;
    }

    int error;
    struct winsize ws;
    error = ioctl(vt->fd, TIOCGWINSZ, &ws);
    if (error) {
        return error;
    }

    struct vtr_stencil_buf sb1 = {0};
    struct vtr_stencil_buf sb2 = {0};
    char* seqlist = NULL;
    uint8_t* linemap = NULL;

    if (0 != create_stencil_buf(&sb1, ws.ws_row, ws.ws_col) || 0 != create_stencil_buf(&sb2, ws.ws_row, ws.ws_col)) {
        goto error_out;
    }

    linemap = malloc((ws.ws_col * VT_CELL_XDOTS + 7) >> 3);
    if (!linemap) {
        goto error_out;
    }

    size_t seqcap = (size_t)((ws.ws_row + 1) * ws.ws_col + 1) * 3;
    seqlist = malloc(seqcap);
    if (!seqlist) {
        goto error_out;
    }

    // No use keeping the previous buffer contents since those
    // are invalid in the new dimentions anyway.
    free_stencil_buf(&vt->sb[0]);
    free_stencil_buf(&vt->sb[1]);
    free(vt->seqlist);

    vt->nrows = ws.ws_row;
    vt->ncols = ws.ws_col;
    vt->ydots = ws.ws_row * VT_CELL_YDOTS;
    vt->xdots = ws.ws_col * VT_CELL_XDOTS;
    vt->sb[0] = sb1;
    vt->sb[1] = sb2;
    vt->cur_sb = &vt->sb[0];
    vt->linemap = linemap;
    vt->seqlist = seqlist;
    vt->seqcap = seqcap;
    vt->resize_pending = false;

    vtr_clear_screen(vt);

    return 0;

error_out:

    free_stencil_buf(&sb1);
    free_stencil_buf(&sb2);
    free(linemap);
    free(seqlist);

    return -1;
}

void vtr_set_resize_pending(struct vtr_canvas* vt)
{
    vt->resize_pending = true;
}

bool vtr_is_resize_pending(const struct vtr_canvas* vt)
{
    return vt->resize_pending;
}

uint16_t vtr_xdots(struct vtr_canvas* vt)
{
    return vt->xdots;
}

uint16_t vtr_ydots(struct vtr_canvas* vt)
{
    return vt->ydots;
}
    
void vtr_close(struct vtr_canvas* vt)
{
    tcsetattr(vt->fd, TCSANOW, &vt->origattrs);
    free_stencil_buf(&vt->sb[0]);
    free_stencil_buf(&vt->sb[1]);
    free(vt->seqlist);

    // switch back to main buffer and restore cursor
    (void) sendseq(vt->fd, "\x1B[?1049l", 8);
    (void) sendseq(vt->fd, "\x1B[?25h", 6);

    free(vt);
}

int vtr_clear_screen(struct vtr_canvas* vt)
{
    return sendseq(vt->fd, "\x1B[2J", 4);
}

void render_dot(struct vtr_stencil_buf* sb, uint16_t x, uint16_t y)
{
    assert(x < sb->xdots && y < sb->ydots);

    uint16_t row = y / VT_CELL_YDOTS;
    uint16_t col = x / VT_CELL_XDOTS;
    uint16_t ncols = sb->xdots / VT_CELL_XDOTS;
    uint16_t stencil = (1u << (y & (VT_CELL_YDOTS - 1))) << ((x & (VT_CELL_XDOTS - 1)) * 4);
    sb->buffer[row * ncols + col] |= stencil;
}

void vtr_render_dot(struct vtr_canvas* vt, uint16_t x, uint16_t y)
{
    if (x >= vt->xdots || y >= vt->ydots) {
        return;
    }

    render_dot(vt->cur_sb, x, y);
}

static char* extend_seq_buf(struct vtr_canvas* vt)
{
    DBG_LOG("Ran out of sequence list capacity %zu\n", vt->seqcap);

    vt->seqcap <<= 1;
    assert(vt->seqcap > 0);

    char* seqlist = realloc(vt->seqlist, vt->seqcap);
    if (!seqlist) {
        vt->seqcap >>= 1;
        return NULL;
    }

    vt->seqlist = seqlist;
    return seqlist;
}

static size_t set_pos_s(char* seq, size_t seqcap, uint16_t row, uint16_t col)
{
    // Not having enough space for \0 is fine (which is the nwritten == seqcap case).
    size_t nwritten = snprintf(seq, seqcap, "\x1B[%d;%dH", row, col);
    if (nwritten > seqcap) {
        return 0;
    }

    return nwritten;
}

static uint8_t draw_current_cell_s(char* seq, size_t seqcap, uint8_t mask)
{
    if (seqcap < 3) {
        return 0;
    }

    seq[0] = 0xE2;
    seq[1] = 0xA0 | (mask >> 6);
    seq[2] = 0x80 | (mask & 0x3f);

    return 3;
}

int vtr_swap_buffers(struct vtr_canvas* vt)
{
    struct vtr_stencil_buf* cur_sb = vt->cur_sb;
    struct vtr_stencil_buf* prev_sb = (vt->cur_sb == &vt->sb[0] ? &vt->sb[1] : &vt->sb[0]);
    size_t seqlen = 0;
    size_t cmdlen = 0;
    size_t cell_idx = 0;
    bool skip_cell = true;

    for (uint16_t row = 1; row <= vt->nrows; row++) {
        for (uint16_t col = 1; col <= vt->ncols; col++, cell_idx++) {
            if (cur_sb->buffer[cell_idx] == prev_sb->buffer[cell_idx]) {
                skip_cell = true;
                continue;
            }

            if (skip_cell) {
                do {
                    cmdlen = set_pos_s(vt->seqlist + seqlen, vt->seqcap - seqlen, row, col);
                    seqlen += cmdlen;

                    if (cmdlen == 0 && !extend_seq_buf(vt)) {
                        return -1;
                    }
                } while (cmdlen == 0);
            }

            // Actual braille cell has a different mask layout, bit numbers displayed below.
            // So we need to convert our stencil first.
            //
            // +---+---+
            // | 1 | 4 |
            // +---+---+
            // | 2 | 5 |
            // +---+---+
            // | 3 | 6 |
            // +---+---+
            // | 7 | 8 |
            // +---+---+

            uint8_t stencil = cur_sb->buffer[cell_idx];
            uint8_t bcell = (stencil & 0x7) | (stencil & 0x8) << 3 | (stencil & 0x70) >> 1 | (stencil & 0x80);

            do {
                cmdlen = draw_current_cell_s(vt->seqlist + seqlen, vt->seqcap - seqlen, bcell);
                seqlen += cmdlen;

                if (cmdlen == 0 && !extend_seq_buf(vt)) {
                    return -1;
                }
            } while (cmdlen == 0);

            skip_cell = false;
        }
    }

    if (sendseq(vt->fd, vt->seqlist, seqlen) != 0) {
        return -1;
    }

    vt->cur_sb = prev_sb;
    memset(vt->cur_sb->buffer, 0, vt->nrows * vt->ncols);

    return 0;
}

/**
 * Calculate a line slope.
 * Returns INFINITY if slope is undefined.
 */
static float calc_slope(uint32_t x0, uint32_t y0, uint32_t x1, uint32_t y1)
{
    return (x1 == x0 ? INFINITY : ((float)y1 - y0) / ((float)x1 - x0));
}

/**
 * Round a non-negative real number to nearest integer, ties are rounded up (e.g. 0.5 -> 1).
 */
static uint16_t round_to_nearest(float f)
{
    assert(f >= 0);
    return (uint16_t)(f + 0.5f);
}

void scan_line(struct vtr_stencil_buf* sb, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    float m = calc_slope(x0, y0, x1, y1);
    int hdir = (x0 < x1 ? 1 : -1);
    int vdir = (y0 < y1 ? 1 : -1);

    if (m == 0.0) {
        do
        {
            render_dot(sb, x0, y0);
            x0 += hdir;
        } while (x0 != x1 + hdir);
    } else if (m == INFINITY) {
        do
        {
            render_dot(sb, x0, y0);
            y0 += vdir;
        } while (y0 != y1 + vdir);
    } else if (m == -1.0 || m == 1.0) {
        do
        {
            render_dot(sb, x0, y0);
            x0 += hdir;
            y0 += vdir;
        } while (x0 != x1 + hdir);
    } else {

        // Generic slope case.
        // A line is y - y0 = m(x - x0) in screen dot space.
        // 
        // We scan either the x or the y coord depending on which one increases faster
        // and plug the scanned value into the line equation to get the other one.
        //
        // If the resulting value has a fractional part it means the line fragment sits in 2 adjacent dot boxes.
        // The fraction distance between the 2 integers tells us which box has the most of the line.
        // We will render the dot that has the largest fragment.
        //
        // If the distance is 0.5 then both boxes have equal fragment sizes, in which case we opt to render both.

        do
        {
            render_dot(sb, x0, y0);

            if (m > -1.0 && m < 1.0) {
                x0 += hdir;
                float y = m * ((float)x0 - x1) + y1;
                y0 = round_to_nearest(y); // 0.5 will round up

                if (y - y0 == -0.5) {
                    render_dot(sb, x0, y0 - 1);
                }
            } else {
                y0 += vdir;
                float x = ((float)y0 - y1) / m + x1;
                x0 = round_to_nearest(x); // 0.5 will round up

                if (x - x0 == -0.5) {
                    render_dot(sb, x0 - 1, y0);
                }
            }
        } while (x0 != x1 + hdir && y0 != y1 + vdir);
    }
}

void vtr_scan_line(struct vtr_canvas* vt, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    scan_line(vt->cur_sb, x0, y0, x1, y1);
}

static void scan_line_map(struct vtr_stencil_buf* sb, const uint8_t* linemap, uint16_t y)
{
    bool in_region = false;

    for (uint16_t x = 0; x < sb->xdots; x++) {
        bool bit = !!(linemap[x >> 3] & (1u << (x & 0x7)));
        if (!in_region && bit) {
            in_region = true;
        } else if (in_region && bit) {
            in_region = false;
        }

        if (in_region || bit) {
            render_dot(sb, x, y);
        }
    }
}

int vtr_trace_poly(struct vtr_canvas* vt, size_t nvertices, const struct vtr_vertex* vertexlist)
{
    assert(vt);
    assert(vertexlist);

    if (nvertices == 0) {
        return 0;
    }

    if (nvertices == 1) {
        render_dot(vt->cur_sb, vertexlist[0].x, vertexlist[0].y);
        return 0;
    }

    if (nvertices == 2) {
        scan_line(vt->cur_sb, vertexlist[0].x, vertexlist[0].y, vertexlist[1].x, vertexlist[1].y);
        return 0;
    }

    assert(nvertices >= 3);

    uint16_t ymax = 0, ymin = UINT16_MAX;
    for (size_t i = 0; i < nvertices; i++) {
        // TODO: should we cull out of bounds vertices instead of failing?
        if (vertexlist[i].x >= vt->xdots || vertexlist[i].y >= vt->ydots) {
            return -1;
        }

        ymin = MIN(ymin, vertexlist[i].y);
        ymax = MAX(ymax, vertexlist[i].y);
    }

    for (uint16_t y = ymin; y != ymax + 1; y++) {
        size_t ncepts = 0;
        memset(vt->linemap, 0, (vt->xdots + 7) >> 3);
        for (size_t i = 0; i < nvertices; i++) {
            struct vtr_vertex a = vertexlist[i];
            struct vtr_vertex b = (i + 1 == nvertices ? vertexlist[0] : vertexlist[i + 1]);

            if (y == a.y && y == b.y) {
                scan_line(vt->cur_sb, a.x, a.y, b.x, b.y);
            } else if ((y == a.y || y == b.y) && (y == ymin || y == ymax)) {
                render_dot(vt->cur_sb, (y == a.y ? a.x : b.x), y);
            } else if ((y >= a.y && y <= b.y) || (y >= b.y && y <= a.y)) {
                uint16_t xcept = round_to_nearest(((float)a.x - b.x) * ((float)y - b.y) / ((float)a.y - b.y) + b.x);
                if (!!(vt->linemap[xcept >> 3] & (1u << (xcept & 0x7)))) {
                    if (y != a.y && y != b.y) {
                        render_dot(vt->cur_sb, xcept, y);
                        vt->linemap[xcept >> 3] &= ~(1u << (xcept & 0x7));
                        ncepts--;
                    }
                }
                else
                {
                    vt->linemap[xcept >> 3] |= (1u << (xcept & 0x7));
                    ncepts++;
                }
            }
        }

        if (ncepts) {
            scan_line_map(vt->cur_sb, vt->linemap, y);
        }
    }

    return 0;
}
