#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#include <signal.h>
#include <unistd.h>

#include <vtrenderlib.h>

#ifndef NDEBUG
#define DBG
#endif

#ifdef FLT_EPSILON
#undef FLT_EPSILON
#endif
#define FLT_EPSILON 0.001

// Simulation freq
#define VT_HZ 60

// Boid dimensions in dots.
#define VT_BOID_WIDTH   6
#define VT_BOID_LENGTH  9

// Boid linear speed in dots per second.
#define VT_BOID_SPEED        50

// Boid roll angle for banking, in degrees.
// Larger angles produce sharper turns, but should be kept under 90.
#define VT_BOID_BANK_ANGLE   80
_Static_assert(VT_BOID_BANK_ANGLE < 90);

// Wandering configuration.
#define VT_BOID_AVG_HEADING_DELAY_MS        2000
#define VT_BOID_HEADING_DELAY_VARIATION_MS  500
#define VT_BOID_HEADING_CHANGE_LIMIT_DEG    30

// Sensing ranges.
#define VT_BOID_VIEW_RANGE                  80
#define VT_BOID_VIEW_RANGE_SQUARED          (VT_BOID_VIEW_RANGE * VT_BOID_VIEW_RANGE)
#define VT_BOID_REPULSION_RANGE             20
#define VT_BOID_REPULSION_RANGE_SQUARED     (VT_BOID_REPULSION_RANGE * VT_BOID_REPULSION_RANGE)
_Static_assert(VT_BOID_REPULSION_RANGE < VT_BOID_VIEW_RANGE);

#define VT_BOID_TRAIL_SIZE 16

// Precomputed radial force for banking
static float g_boid_radial_force;

// Trail history buffer size.
#define VT_BOID_TRAIL_SIZE                  20

// Options.
static bool g_opt_debug;
static bool g_opt_colors;
static bool g_opt_trails;
static int g_opt_nboids = 64;

struct vec2f
{
    float x;
    float y;
};

struct vt_boid
{
    // position, velocity and normal vectors, normals are unit vectors.
    struct vec2f p;
    struct vec2f v;
    struct vec2f n;

    // heading angles, in radians
    float heading;
    float desired_heading;

    // wandering state
    int heading_change_delay;
    int cur_heading_time;

    // rendering data
    enum vtr_color color;
    struct vec2f trail[VT_BOID_TRAIL_SIZE];
    size_t trail_idx;
    size_t trail_len;
};

static struct vtr_canvas* g_vt;
static struct vt_boid* g_boids;
static size_t g_nboids;

static inline float grad2rad(int grad)
{
    return M_PI * grad / 180;
}

static inline bool are_equalf(float a, float b)
{
    return fabsf(a - b) <= FLT_EPSILON;
}

static inline float roundfe(float v)
{
    return roundf(v / FLT_EPSILON) * FLT_EPSILON;
}

static inline struct vec2f vec2f_make(float x, float y)
{
    return (struct vec2f){x, y};
}

static inline float heading_angle(struct vec2f v)
{
    float hrad = atan2f(v.y, v.x);
    hrad = (hrad < 0 ? hrad + M_PI * 2 : hrad);
    return roundfe(hrad);
}

static inline struct vec2f heading_vec(float heading)
{
    return vec2f_make(roundfe(cosf(heading)), roundfe(sinf(heading)));
}

static inline struct vec2f vec2f_add(struct vec2f a, struct vec2f b)
{
    return vec2f_make(a.x + b.x, a.y + b.y);
}

static inline struct vec2f vec2f_mul(struct vec2f a, float b)
{
    return vec2f_make(a.x * b, a.y * b);
}

static inline struct vec2f vec2f_mul_add(struct vec2f a, struct vec2f b, float scale)
{
    return vec2f_make(a.x + b.x * scale, a.y + b.y * scale);
}

static inline struct vec2f vec2f_sub(struct vec2f a, struct vec2f b)
{
    return vec2f_make(a.x - b.x, a.y - b.y);
}

static inline float vec2f_length(struct vec2f v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

static inline float vec2f_dot(struct vec2f a, struct vec2f b)
{
    return (a.x * b.x + a.y * b.y);
}

static inline struct vec2f vec2f_unit(struct vec2f v)
{
    float m = vec2f_length(v);
    if (m == 0) {
        return v;
    }

    return vec2f_make(v.x / m, v.y / m);
}

static inline struct vec2f vec2f_clamp(struct vec2f v, float max)
{
    // If length-squared is under the limit-squared then the same is true for their sqrts,
    // but we don't have to compute it then.
    float len = v.x * v.x + v.y * v.y;
    if (len <= max * max) {
        return v;
    }

    return vec2f_mul(v, max / sqrt(len));
}

static inline struct vec2f vec2f_normal(struct vec2f v)
{
    return vec2f_unit((struct vec2f){-v.y, v.x});
}

static inline struct vec2f vec2f_rot(struct vec2f v, double rad)
{
    float cs = cosf(rad);
    float sn = sinf(rad);

    return vec2f_make(v.x * cs - v.y * sn, v.x * sn + v.y * cs);
}

static inline float vec2f_dist_squared(struct vec2f a, struct vec2f b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

static inline struct vec2f vec2f_lerp(struct vec2f a, struct vec2f b, float f)
{
    assert(f >= 0 && f <= 1.0);
    return vec2f_make(a.x * (1.0 - f) + b.x * f, a.y * (1.0 - f) + b.y * f);
}

static inline struct vtr_vertex vec2f_project(struct vec2f v)
{
    return (struct vtr_vertex){roundf(v.x), roundf(v.y)};
}

static inline int random_value_in_range(int min, int max)
{
    assert(max > min);
    return min + rand() % (max - min);
}

static inline int random_value_spread(int base, int spread)
{
    assert(spread != 0);
    return base + ((rand() % (spread * 2)) - spread);
}

static void debug_print(uint16_t row, uint16_t col, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buf[128];
    vsnprintf(buf, sizeof(buf), fmt, args);
    vtr_print_text(g_vt, row, col, buf);
    va_end(args);
}

// Implements small random changes to boid's heading after keeping the current heading for some time.
static void wander(struct vt_boid* b, uint32_t dtime)
{
    b->cur_heading_time += dtime;

    if (b->cur_heading_time >= b->heading_change_delay) {
        b->cur_heading_time = 0;
        b->heading_change_delay = random_value_spread(VT_BOID_AVG_HEADING_DELAY_MS, VT_BOID_HEADING_DELAY_VARIATION_MS);
        b->desired_heading = b->heading + grad2rad(random_value_spread(0, VT_BOID_HEADING_CHANGE_LIMIT_DEG));
    }
}

static void draw_debug_vec(struct vec2f origin, struct vec2f vec, int len, enum vtr_color fgc)
{
    struct vtr_vertex start = vec2f_project(origin);
    struct vtr_vertex end = vec2f_project(vec2f_mul_add(origin, vec, len));

    vtr_scan_linec(g_vt, start.x, start.y, end.x, end.y, fgc);
}

// Update simulation, dtime is in millisecs.
static void update(uint32_t dtime)
{
    if (g_opt_debug) {
        static uint64_t total_time = 0;
        total_time += dtime;
        debug_print(0, 0, "t(s) = %.02f\n", (float)total_time / 1000);
    }

    for (size_t i = 0; i < g_nboids; i++) {
        struct vt_boid* b = g_boids + i;

        // The neighbors search below makes the entire update quadratic.
        // It's not super terrible given the low number of boids i anticipate,
        // but i could think about maybe using a proximity lookup via space partitioning.

        size_t total_neighbors = 0;
        struct vec2f alignment = {0, 0};
        struct vec2f cohesion = {0, 0};
        struct vec2f separation = {0, 0};
        for (size_t j = 0; j < g_nboids; j++) {
            if (j == i) {
                continue;
            }

            struct vt_boid* other = g_boids + j;
            float dist_squared = vec2f_dist_squared(b->p, other->p);
            if (dist_squared <= VT_BOID_VIEW_RANGE_SQUARED) {
                total_neighbors += 1;
                alignment = vec2f_add(alignment, other->v);
                cohesion = vec2f_add(cohesion, other->p);

                if (dist_squared <= VT_BOID_REPULSION_RANGE_SQUARED)
                {
                    // If distance rounds to 0 use a tiny distance instead to compute a repulsion vector.
                    struct vec2f repulsion = vec2f_mul(vec2f_sub(b->p, other->p),
                                                       1.0f / (dist_squared == 0 ? 0.001f : dist_squared));
                    separation = vec2f_add(separation, repulsion);
                }
            }
        }

        if (total_neighbors == 0) {
            wander(b, dtime);
        } else {
            alignment = vec2f_unit(alignment);

            cohesion = vec2f_add(cohesion, b->p);
            cohesion = (struct vec2f){cohesion.x / (total_neighbors + 1), cohesion.y / (total_neighbors + 1)};
            cohesion = vec2f_unit(vec2f_sub(cohesion, b->p));

            separation = vec2f_unit(separation);

            // TODO: weighted adds
            struct vec2f heading_vec = {0, 0};
            heading_vec = vec2f_add(heading_vec, alignment);
            heading_vec = vec2f_add(heading_vec, cohesion);
            heading_vec = vec2f_add(heading_vec, separation);

            b->desired_heading = heading_angle(heading_vec);
        }

        // Update heading to be closer to the desired heading over dt.
        float dheading = g_boid_radial_force * dtime / 1000;
        if (fabsf(b->heading - b->desired_heading) <= dheading) {
            b->heading = b->desired_heading;
        } else {
            b->heading += (b->desired_heading > b->heading ? dheading : -dheading);
        }

        b->p.x += VT_BOID_SPEED * cosf(b->heading) * dtime / 1000;
        b->p.y += VT_BOID_SPEED * sinf(b->heading) * dtime / 1000;
        b->v = heading_vec(b->heading);
        b->n = vec2f_normal(b->v);

        // Wrap over screen edges
        if (b->p.x < 0) {
            b->p.x = vtr_xdots(g_vt) + b->p.x;
        } else if (b->p.x >= vtr_xdots(g_vt)) {
            b->p.x = b->p.x - vtr_xdots(g_vt);
        }

        if (b->p.y < 0) {
            b->p.y = vtr_ydots(g_vt) + b->p.y;
        } else if (b->p.y >= vtr_ydots(g_vt)) {
            b->p.y = b->p.y - vtr_ydots(g_vt);
        }
    }
}

static void draw(void)
{
    for (size_t i = 0; i < g_nboids; i++) {
        struct vt_boid* b = g_boids + i;

        struct vtr_vertex buf[] = {
            vec2f_project(vec2f_mul_add(b->p, b->n, -VT_BOID_WIDTH / 2)),
            vec2f_project(vec2f_mul_add(b->p, b->n, VT_BOID_WIDTH / 2)),
            vec2f_project(vec2f_mul_add(b->p, b->v, VT_BOID_LENGTH)),
        };

        vtr_trace_polyc(g_vt, sizeof(buf) / sizeof(*buf), buf, b->color);

        if (g_opt_trails) {
            for (size_t idx = 0; idx < b->trail_len; idx++) {
                // Draw every other trail dot so that it makes a dashed curve
                if (idx & 0x1) {
                    struct vec2f trail_pos = b->trail[b->trail_idx > idx ? b->trail_idx - idx - 1 : b->trail_len + b->trail_idx - idx];
                    struct vtr_vertex trail_dot = vec2f_project(trail_pos);
                    vtr_render_dotc(g_vt, trail_dot.x, trail_dot.y, b->color);
                }
            }
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

static uint64_t clock_monotonic_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t);

    return ((uint64_t)t.tv_sec * 1000000000 + t.tv_nsec) / 1000000;
}

void print_help(const char *progname)
{
    printf("Usage: %s [options]\n", progname);
    printf("\t-n <number>: set a specific number of boids\n");
    printf("\t-d:          draw debug vectors\n");
    printf("\t-c:          use random colors for boids\n");
    printf("\t-t:          draw trails\n");
    printf("\t-h:          display this help\n");
}

int main(int argc, char** argv)
{
    int error;
    int opt;

    while ((opt = getopt(argc, argv, "dn:ch")) != -1) {
        switch (opt) {
        case 'd':
            g_opt_debug = true;
            break;
        case 'n':
            g_opt_nboids = atoi(optarg);
            if (g_opt_nboids <= 0) {
                goto bad_opts;
            }
            break;
        case 'c':
            g_opt_colors = true;
            break;
        case 't':
            g_opt_trails = true;
            break;
        case 'h':
            print_help(argv[0]);
            exit(EXIT_SUCCESS);
        default:
            goto bad_opts;
        }
    }

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

    srand((unsigned int)time(NULL));

    g_nboids = g_opt_nboids;
    g_boids = calloc(sizeof(*g_boids), g_nboids);
    if (!g_boids) {
        exit(ENOMEM);
    }

    static const int colors[] = {
        VTR_COLOR_YELLOW,
        VTR_COLOR_BLUE,
        VTR_COLOR_GREEN,
        VTR_COLOR_RED
    };

    for (size_t i = 0; i < g_nboids; i++) {
        struct vt_boid* b = g_boids + i;
        b->p = (struct vec2f){random_value_in_range(0, vtr_xdots(g_vt) - 1), random_value_in_range(0, vtr_ydots(g_vt) - 1)};
        b->desired_heading = b->heading = grad2rad(random_value_in_range(0, 360));
        b->v = heading_vec(b->heading);
        b->n = vec2f_normal(b->v);
        b->color = (g_opt_colors ? colors[i % (sizeof(colors) / sizeof(*colors))] : VTR_COLOR_DEFAULT);
    }

    // The math below precomputes approximated boid radial force generated by a fixed banking angle
    g_boid_radial_force = 9.81 * tanf(grad2rad(VT_BOID_BANK_ANGLE)) / VT_BOID_SPEED;

    uint64_t tcur, tprev = clock_monotonic_ms();
    uint32_t tdiff;
    while (true) {
        vtr_resize(g_vt);

        tcur = clock_monotonic_ms();
        assert(tcur >= tprev && (tcur - tprev <= UINT32_MAX));
        tdiff = tcur - tprev;
        tprev = tcur;

        update(tdiff);
        draw();

        vtr_swap_buffers(g_vt);

        usleep(1000000 / VT_HZ);
    }

    return 0;

bad_opts:
    fprintf(stderr, "Usage: %s [-d] [-c] [-t] [-n boids-count]\n", argv[0]);
    exit(EXIT_FAILURE);
}
