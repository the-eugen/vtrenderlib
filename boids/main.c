#include <assert.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#include <signal.h>
#include <unistd.h>

#include <vtrenderlib.h>

// Boid dimentions in dots.
#define VT_BOID_WIDTH   6
#define VT_BOID_LENGTH  9

// Boid linear speed in dots per second.
#define VT_BOID_SPEED        50

// Boid roll angle for banking in degrees.
// Larger angles produce sharper turns.
#define VT_BOID_BANK_ANGLE   60

// Wandering configuration.
#define VT_BOID_AVG_HEADING_DELAY_MS        2000
#define VT_BOID_HEADING_DELAY_VARIATION_MS  500
#define VT_BOID_HEADING_CHANGE_LIMIT_DEG    30

// Precomputed radial force for banking
static float g_boid_radial_force;

struct vec2f
{
    float x;
    float y;
};

struct vt_boid
{
    // position, velocity and normal vectors of a boid, normals are unit vectors (len is 1).
    struct vec2f p;
    struct vec2f v;
    struct vec2f n;

    // heading angle, in radians
    float heading;
    float desired_heading;

    // wandering state
    int heading_change_delay;
    int cur_heading_time;
};

static struct vtr_canvas* g_vt;
static struct vt_boid* g_boids;
static size_t g_nboids;

static inline float grad2rad(int grad)
{
    return M_PI * grad / 180;
}

static inline struct vec2f vec2f_mul_add(struct vec2f a, struct vec2f b, int scale)
{
    return (struct vec2f){a.x + b.x * scale, a.y + b.y * scale};
}

static inline struct vec2f vec2f_sub(struct vec2f a, struct vec2f b)
{
    return (struct vec2f){a.x - b.x, a.y - b.y};
}

static inline struct vec2f vec2f_unit(struct vec2f v)
{
    float m = sqrtf(v.x * v.x + v.y * v.y);
    assert(m != 0);

    return (struct vec2f){v.x / m, v.y / m};
}

static inline struct vec2f vec2f_normal(struct vec2f v)
{
    return (struct vec2f){-v.y, v.x};
}

static inline struct vec2f vec2f_rot(struct vec2f v, double rad)
{
    float cs = cosf(rad);
    float sn = sinf(rad);

    return (struct vec2f){
        v.x * cs - v.y * sn,
        v.x * sn + v.y * cs
    };
}

static inline struct vtr_vertex vec2f_project(struct vec2f v)
{
    return (struct vtr_vertex){(int)(v.x + 0.5f), (int)(v.y + 0.5f)};
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


// Implements random wandering for a boid with no neighbours in view range.
// Gets called every frame to apply random gradual changes to boid's heading and follow them.
static void wander(struct vt_boid* b, int dtime)
{
    b->cur_heading_time += dtime;

    // See if we've been using our current target heading for enough time and change it if needed
    if (b->cur_heading_time >= b->heading_change_delay) {
        b->cur_heading_time = 0;
        b->heading_change_delay = random_value_spread(VT_BOID_AVG_HEADING_DELAY_MS, VT_BOID_HEADING_DELAY_VARIATION_MS);
        b->desired_heading = b->heading + grad2rad(random_value_spread(0, VT_BOID_HEADING_CHANGE_LIMIT_DEG));
    }
}

// Update heading to be closer to the desired heading over dt.
static float bank(struct vt_boid* b, int dtime)
{
    if (b->heading == b->desired_heading) {
        return 0.0f;
    }

    float dheading = g_boid_radial_force * dtime / 1000;
    float heading = b->heading;
    if (fabsf(b->heading - b->desired_heading) <= dheading) {
        b->heading = b->desired_heading;
    } else {
        b->heading += (b->desired_heading > b->heading ? dheading : -dheading);
    }

    return b->heading - heading;
}

// Update simulation, dt is in millisecs.
static void update(double dtime)
{
    for (size_t i = 0; i < g_nboids; i++) {
        struct vt_boid* b = g_boids + i;

        // TODO: only wander if no boids are nearby
        wander(b, (int)dtime);
        float dheading = bank(b, (int)dtime);

        b->p.x += VT_BOID_SPEED * cosf(b->heading) * dtime / 1000;
        b->p.y += VT_BOID_SPEED * sinf(b->heading) * dtime / 1000;
        b->v = vec2f_rot(b->v, dheading);
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

        vtr_trace_poly(g_vt, 3, buf);
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

    g_nboids = 32;
    g_boids = calloc(sizeof(*g_boids), g_nboids);
    if (!g_boids) {
        exit(ENOMEM);
    }

    for (size_t i = 0; i < g_nboids; i++) {
        struct vt_boid* b = g_boids + i;
        b->p = (struct vec2f){random_value_in_range(0, vtr_xdots(g_vt) - 1), random_value_in_range(0, vtr_ydots(g_vt) - 1)};
        b->v = (struct vec2f){1.0, 0.0};
        b->n = vec2f_normal(b->v);
    }

    // The math below precomputed approximated boid radial force generated by a fixed banking angle
    g_boid_radial_force = 9.81 * tanf(grad2rad(VT_BOID_BANK_ANGLE)) / VT_BOID_SPEED;

    srand((unsigned int)time(NULL));

    while (true) {
        vtr_resize(g_vt);
        update(1000.0 / 60);
        draw();
        vtr_swap_buffers(g_vt);
        usleep(1000000 / 60);
    }

    return 0;
}
