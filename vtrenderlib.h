#pragma once

struct vtr_canvas;

struct vtr_canvas* vtr_canvas_create(int ttyfd);
int vtr_reset(struct vtr_canvas* vt);
void vtr_close(struct vtr_canvas* vt);

void vtr_set_resize_pending(struct vtr_canvas* vt);
bool vtr_is_resize_pending(const struct vtr_canvas* vt);
int vtr_resize(struct vtr_canvas* vt);

uint16_t vtr_xdots(struct vtr_canvas* vt);
uint16_t vtr_ydots(struct vtr_canvas* vt);

int vtr_clear_screen(struct vtr_canvas* vt);
int vtr_swap_buffers(struct vtr_canvas* vt);

/*
 * Rasterizer calls.
 */

/**
 * Render a dot at a given VT coordinates.
 */
void vtr_render_dot(struct vtr_canvas* vt, int x, int y);

/**
 * Scan a line give 2 dot coordinates.
 */
void vtr_scan_line(struct vtr_canvas* vt, int x0, int y0, int x1, int y1);

struct vtr_vertex
{
    int x;
    int y;
};

/**
 * Trace a polygon path given a list of verices.
 * Last vertex will be traced back to the first one and the resulting polygon filled.
 */
int vtr_trace_poly(struct vtr_canvas* vt, size_t nvertices, const struct vtr_vertex* vertexlist);
