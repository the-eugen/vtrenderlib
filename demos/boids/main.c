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

#ifdef _LINUX_
#include <signal.h>
#include <unistd.h>
#else
#include <windows.h>
#endif

#include <vtrenderlib.h>

#ifndef NDEBUG
#define DBG
#endif

#ifdef FLT_EPSILON
#undef FLT_EPSILON
#endif
#define FLT_EPSILON 0.001

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

// Simulation freq
#define VT_HZ 60

// Boid dimensions in dots.
#define VT_BOID_WIDTH   7
#define VT_BOID_LENGTH  9

// Boid linear speed in dots per second.
#define VT_BOID_SPEED        60

// Wandering configuration.
#define VT_BOID_AVG_HEADING_DELAY_MS        2000
#define VT_BOID_HEADING_DELAY_VARIATION_MS  500
#define VT_BOID_HEADING_CHANGE_LIMIT_DEG    30

// Sensing ranges.
#define VT_BOID_VIEW_RANGE                  80
#define VT_BOID_VIEW_RANGE_SQUARED          (VT_BOID_VIEW_RANGE * VT_BOID_VIEW_RANGE)
#define VT_BOID_REPULSION_RANGE             15
#define VT_BOID_REPULSION_RANGE_SQUARED     (VT_BOID_REPULSION_RANGE * VT_BOID_REPULSION_RANGE)
_Static_assert(VT_BOID_REPULSION_RANGE < VT_BOID_VIEW_RANGE);

// Steering forces weights.
#define VT_BOID_ALIGNMENT_WEIGHT            1.2f
#define VT_BOID_COHESION_WEIGHT             0.8f
#define VT_BOID_SEPARATION_WEIGHT           1.0f

// Steering force cap.
#define VT_BOID_STEERING_CAP                10.0f

// Acceleration blending factor: dt / (tau + dt), where tau is seconds to blend-in an acceleration update.
#define VT_BOID_BLEND_FACTOR                (((float)VT_HZ / 1000) / (0.2 + ((float)VT_HZ / 1000)))

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
    struct vec2f p;
    struct vec2f v;

    // heading angle, in radians
    float h;

    // Current angular speed
    float w;

    // wandering state
    float wander_angle;
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

static void debug_vec(struct vec2f origin, struct vec2f vec, float scale, enum vtr_color fgc)
{
    struct vtr_vertex start = vec2f_project(origin);
    struct vtr_vertex end = vec2f_project(vec2f_mul_add(origin, vec, scale));

    vtr_scan_linec(g_vt, start.x, start.y, end.x, end.y, fgc);
}

// Apply a steering force (acceleration) vector to a boid to update its speed, position and heading.
static void steer(struct vt_boid* b, struct vec2f fsteer, uint32_t dtime)
{
    // Record current position for trails before we change it.
    b->trail[b->trail_idx] = b->p;
    b->trail_idx = (b->trail_idx + 1) % VT_BOID_TRAIL_SIZE;
    b->trail_len = MIN(b->trail_len + 1, VT_BOID_TRAIL_SIZE);

    // Boid linear speed is fixed so we are only interested in the lateral component of the steering force.
    // Compute that as lateral acceleration, get the angular speed from it and blend it into an accumulator.
    // Then integrate blended angular speed into heading and position changes over dt.
    float dts = (float)dtime / 1000;
    float alat = vec2f_dot(vec2f_clamp(fsteer, VT_BOID_STEERING_CAP), vec2f_make(-sinf(b->h), cos(b->h)));
    b->w = (1 - VT_BOID_BLEND_FACTOR) * b->w + VT_BOID_BLEND_FACTOR * alat;
    b->h = fmodf(b->h + b->w * dts, 2.0f * M_PI);
    b->p.x += VT_BOID_SPEED * cosf(b->h) * dts;
    b->p.y += VT_BOID_SPEED * sinf(b->h) * dts;
    b->v = heading_vec(b->h);
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
        struct vec2f centroid = {0, 0};
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
                centroid = vec2f_add(centroid, other->p);

                if (dist_squared <= VT_BOID_REPULSION_RANGE_SQUARED)
                {
                    // Compute a repulsion vector to be stronger the closer this neighbor is to us.
                    // Add an epsilon value to avoid potential div by 0.
                    struct vec2f repulsion = vec2f_mul(vec2f_sub(b->p, other->p),
                                                       (float)VT_BOID_REPULSION_RANGE / (dist_squared + FLT_EPSILON));
                    separation = vec2f_add(separation, repulsion);
                }
            }
        }

        if (total_neighbors == 0) {
            // Small random changes to boid's heading after keeping the current heading for some time.
            b->cur_heading_time += dtime;
            if (b->cur_heading_time >= b->heading_change_delay) {
                b->cur_heading_time = 0;
                b->heading_change_delay = random_value_spread(VT_BOID_AVG_HEADING_DELAY_MS, VT_BOID_HEADING_DELAY_VARIATION_MS);
                b->wander_angle = grad2rad(random_value_spread(b->h, VT_BOID_HEADING_CHANGE_LIMIT_DEG));
            }

            struct vec2f fwander = heading_vec(b->wander_angle);
            steer(b, fwander, dtime);

            if (g_opt_debug) {
                debug_print(i + 1, 0, "h = %+.02f, w = %+.02f, fwander = %.02f", b->h, b->w, vec2f_length(fwander));
            }
        } else {
            // Reset wander state since we are in a flock now.
            b->cur_heading_time = 0;
            b->heading_change_delay = 0;
            b->wander_angle = 0;

            // Alignment vec points towards average direction of our neighbors while its magnitude
            // is proportional to how much consensus they have with that direction.
            // We'd like to be pulled stronger towards a shared alignment the more consensus it has.
            alignment = vec2f_mul(alignment, VT_BOID_ALIGNMENT_WEIGHT / total_neighbors);
            alignment = vec2f_clamp(alignment, VT_BOID_STEERING_CAP);

            // Cohesion vec points to the center-mass point of local flock.
            // Convert that to a pull towards that centroid which is proportional to distance to the centroid.
            // Maximum possible cohesion magnitude will approach the view distance, so we normalize by that.
            centroid = vec2f_mul(vec2f_add(centroid, b->p), 1.0f / (total_neighbors + 1));
            struct vec2f cohesion = vec2f_sub(centroid, b->p);
            cohesion = vec2f_mul(cohesion, VT_BOID_COHESION_WEIGHT / VT_BOID_VIEW_RANGE);
            cohesion = vec2f_clamp(cohesion, VT_BOID_STEERING_CAP);

            // Separation vec points away from neighbors in repulsion range.
            // The magnitude is iversely proportional to how close the neighbors are (e.g. how urgent it is).
            separation = vec2f_mul(separation, VT_BOID_SEPARATION_WEIGHT);
            separation = vec2f_clamp(separation, VT_BOID_STEERING_CAP);

            struct vec2f fsteer = {0, 0};
            fsteer = vec2f_add(fsteer, alignment);
            fsteer = vec2f_add(fsteer, cohesion);
            fsteer = vec2f_add(fsteer, separation);

            steer(b, fsteer, dtime);

            if (g_opt_debug) {
                debug_print(i + 1, 0, "h = %+.02f, w = %+.02f, falign = %.02f, fcoh = %.02f, fsep = %.02f",
                            b->h, b->w, vec2f_length(alignment), vec2f_length(cohesion), vec2f_length(separation));
                debug_vec(b->p, alignment, 10, VTR_COLOR_BLUE);
                debug_vec(b->p, cohesion, 10, VTR_COLOR_GREEN);
                debug_vec(b->p, separation, 10, VTR_COLOR_RED);
                debug_vec(b->p, fsteer, 10, VTR_COLOR_DEFAULT);
            }
        }


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
        struct vec2f d = vec2f_unit(b->v);
        struct vec2f n = vec2f_normal(d);

        struct vtr_vertex buf[] = {
            vec2f_project(vec2f_mul_add(b->p, n, -VT_BOID_WIDTH / 2)),
            vec2f_project(vec2f_mul_add(b->p, n, VT_BOID_WIDTH / 2)),
            vec2f_project(vec2f_mul_add(b->p, d, VT_BOID_LENGTH)),
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

static uint64_t clock_monotonic_ms(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC_RAW, &t);

    return ((uint64_t)t.tv_sec * 1000000000 + t.tv_nsec) / 1000000;
}

#else

static void usleep(unsigned long long us)
{
    Sleep(us / 1000);
}

static uint64_t clock_monotonic_ms(void)
{
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&counter);

    return (uint64_t)(counter.QuadPart * 1000000000LL / freq.QuadPart) / 1000000;
}

#endif

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

#ifdef _LINUX_
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
    signal(SIGINT, handle_signal);
    signal(SIGWINCH, handle_signal);
#else
    g_opt_colors = true;
    g_opt_nboids = 32;
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
        b->p = vec2f_make(random_value_in_range(0, vtr_xdots(g_vt) - 1), random_value_in_range(0, vtr_ydots(g_vt) - 1));
        b->h = grad2rad(random_value_in_range(0, 360));
        b->v = heading_vec(b->h);
        b->color = (g_opt_colors ? colors[i % (sizeof(colors) / sizeof(*colors))] : VTR_COLOR_DEFAULT);
    }

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
