#pragma once

/*
 * vtrenderlib.h - tiny vector rasterizer for ANSI terminals.
 *
 * The library exposes a minimal API for drawing 2D primitives into a
 * terminal emulator using braille characters.  The consumer is expected
 * to manage the terminal file descriptor and drive the rendering loop.
 */

#ifdef __cplusplus
extern "C" {
#endif

struct vtr_canvas;

/*
 * Create a canvas object bound to an already opened TTY file descriptor.
 * Returns NULL on error.
 */
struct vtr_canvas* vtr_canvas_create(int ttyfd);

/*
 * Put the associated terminal into raw mode and switch to the alternate
 * screen buffer.  Must be called before any drawing takes place.
 */
int vtr_reset(struct vtr_canvas* vt);

/*
 * Restore the original terminal attributes and free all resources.
 */
void vtr_close(struct vtr_canvas* vt);

/*
 * Resize support.
 *
 * Terminal resizes are handled asynchronously.  The consumer should
 * watch for SIGWINCH and set the resize pending flag from the signal
 * handler.  The actual resize is performed on the next call to
 * vtr_resize() from the main loop.
 */

void vtr_set_resize_pending(struct vtr_canvas* vt);
bool vtr_is_resize_pending(const struct vtr_canvas* vt);
int vtr_resize(struct vtr_canvas* vt);

/* Canvas dimensions in dots */
uint16_t vtr_xdots(struct vtr_canvas* vt);
uint16_t vtr_ydots(struct vtr_canvas* vt);

/* Clear the entire screen */
int vtr_clear_screen(struct vtr_canvas* vt);

/*
 * Swap buffers when a frame is complete.  All accumulated changes are
 * flushed to the terminal and the back buffer is cleared for the next
 * frame.
 */

int vtr_swap_buffers(struct vtr_canvas* vt);

/*
 * Rasterizer calls.
 */

/* Vertex in dot coordinates */
struct vtr_vertex
{
    int x;
    int y;
};

/* Basic ANSI color palette */
enum vtr_color
{
    VTR_COLOR_DEFAULT,
    VTR_COLOR_BLACK,
    VTR_COLOR_RED,
    VTR_COLOR_GREEN,
    VTR_COLOR_YELLOW,
    VTR_COLOR_BLUE,
    VTR_COLOR_MAGENTA,
    VTR_COLOR_CYAN,
    VTR_COLOR_WHITE,

    VTR_COLOR_TOTAL // always last
};

/**
 * Render a dot at a given VT coordinates.
 */
void vtr_render_dot(struct vtr_canvas* vt, int x, int y);
void vtr_render_dotc(struct vtr_canvas* vt, int x, int y, enum vtr_color fgc);

/**
 * Scan a line give 2 dot coordinates.
 */
void vtr_scan_line(struct vtr_canvas* vt, int x0, int y0, int x1, int y1);
void vtr_scan_linec(struct vtr_canvas* vt, int x0, int y0, int x1, int y1, enum vtr_color fgc);

/**
 * Trace a polygon path given a list of verices.
 * Last vertex will be traced back to the first one and the resulting polygon filled.
 */
int vtr_trace_poly(struct vtr_canvas* vt, size_t nvertices, const struct vtr_vertex* vertexlist);
int vtr_trace_polyc(struct vtr_canvas* vt, size_t nvertices, const struct vtr_vertex* vertexlist, enum vtr_color fgc);

#ifdef __cplusplus
}
#endif
