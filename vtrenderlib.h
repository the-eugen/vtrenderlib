#pragma once

struct vtr_canvas;

struct vtr_canvas* vtr_canvas_create(int ttyfd);
int vtr_reset(struct vtr_canvas* vt);
void vtr_close(struct vtr_canvas* vt);

/*
 * Resize support.
 *
 * The application is responsible for handling SIGWINCH.
 * A typical handler will call vtr_set_resize_pending
 * and the app's main event loop will vtr_resize synchroniously.
 */

void vtr_set_resize_pending(struct vtr_canvas* vt);
bool vtr_is_resize_pending(const struct vtr_canvas* vt);
int vtr_resize(struct vtr_canvas* vt);

uint16_t vtr_xdots(struct vtr_canvas* vt);
uint16_t vtr_ydots(struct vtr_canvas* vt);

int vtr_clear_screen(struct vtr_canvas* vt);

/*
 * Swap buffers is called by the app when the current 2d frame is completed.
 * It collects all the changes accumulates since the last time vtr_swap_buffers was called
 * and applies them to the VT screen contents.
 */

int vtr_swap_buffers(struct vtr_canvas* vt);

/*
 * Rasterizer calls.
 */

struct vtr_vertex
{
    int x;
    int y;
};

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
