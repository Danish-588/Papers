#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define FIELD_WIDTH    8.0
#define FIELD_LENGTH  15.0
#define NUM_SENSORS    4
#define CORNER_MARGIN  0.3    // skip hits this close to a corner
#define EPSILON        1e-6

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double x, y; } Position;
typedef struct { const char* name; double dir_off, dx, dy; } Sensor;
typedef struct { int a, b; const char* name; } Pair;

enum Wall { HIT_NONE, HIT_LEFT, HIT_RIGHT, HIT_BOTTOM, HIT_TOP };

typedef struct {
    double t;
    enum Wall wall;
    Position hit;
} HitResult;

typedef struct {
    bool   valid;   // false if unusable (corner or no hit)
    double est_x;  // NaN if only y
    double est_y;  // NaN if only x
} Inversion;

static const Sensor sensors[NUM_SENSORS] = {
    {"Front",  90.0,  0.0,  0.28},
    {"Right",   0.0,  0.18, 0.10},
    {"Back",   270.0,  0.0, -0.28},
    {"Left",   180.0, -0.18, 0.10}
};

static const Pair pairs[NUM_SENSORS] = {
    {0, 1, "Front+Right"},
    {1, 2, "Right+Back"},
    {2, 3, "Back+Left"},
    {3, 0, "Left+Front"}
};

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

// Compute intersection with each wall, return nearest positive t and wall & hit
HitResult find_wall_hit_full(Position p, double angle_deg) {
    double th = deg2rad(angle_deg);
    double dx = cos(th), dy = sin(th);
    HitResult best = { .t = INFINITY, .wall = HIT_NONE, .hit = {NAN, NAN} };

    // left wall x=0 and right wall x=FIELD_WIDTH
    if (fabs(dx) > EPSILON) {
        double t = (0 - p.x)/dx;
        double y = p.y + t*dy;
        if (t>0 && y>=0 && y<=FIELD_LENGTH && t<best.t) {
            best.t = t;
            best.wall = HIT_LEFT;
            best.hit = (Position){0, y};
        }
        t = (FIELD_WIDTH - p.x)/dx;
        y = p.y + t*dy;
        if (t>0 && y>=0 && y<=FIELD_LENGTH && t<best.t) {
            best.t = t;
            best.wall = HIT_RIGHT;
            best.hit = (Position){FIELD_WIDTH, y};
        }
    }
    // bottom y=0 and top y=FIELD_LENGTH
    if (fabs(dy) > EPSILON) {
        double t = (0 - p.y)/dy;
        double x = p.x + t*dx;
        if (t>0 && x>=0 && x<=FIELD_WIDTH && t<best.t) {
            best.t = t;
            best.wall = HIT_BOTTOM;
            best.hit = (Position){x, 0};
        }
        t = (FIELD_LENGTH - p.y)/dy;
        x = p.x + t*dx;
        if (t>0 && x>=0 && x<=FIELD_WIDTH && t<best.t) {
            best.t = t;
            best.wall = HIT_TOP;
            best.hit = (Position){x, FIELD_LENGTH};
        }
    }
    return best;
}

// rotate sensor offset into world frame
static Position sensor_origin(Position rob, double heading_deg, const Sensor* s) {
    double t = deg2rad(heading_deg);
    double wx = s->dx * cos(t) - s->dy * sin(t);
    double wy = s->dx * sin(t) + s->dy * cos(t);
    return (Position){rob.x + wx, rob.y + wy};
}

// analytically invert one sensor
static Inversion invert_sensor(Position rob, double heading_deg,
                               const Sensor* s, double measured_dist) {
    Inversion inv = { .valid = false, .est_x = NAN, .est_y = NAN };
    Position ori = sensor_origin(rob, heading_deg, s);
    HitResult hr = find_wall_hit_full(ori, heading_deg + s->dir_off);
    if (hr.wall == HIT_NONE) return inv;

    bool corner = false;
    if (hr.wall == HIT_LEFT || hr.wall == HIT_RIGHT) {
        if (hr.hit.y < CORNER_MARGIN || hr.hit.y > FIELD_LENGTH - CORNER_MARGIN)
            corner = true;
    } else {
        if (hr.hit.x < CORNER_MARGIN || hr.hit.x > FIELD_WIDTH - CORNER_MARGIN)
            corner = true;
    }
    if (corner) return inv;

    inv.valid = true;
    double ang = deg2rad(heading_deg + s->dir_off);
    switch (hr.wall) {
        case HIT_LEFT:
            inv.est_x = 0 - measured_dist * cos(ang) - (ori.x - rob.x);
            break;
        case HIT_RIGHT:
            inv.est_x = FIELD_WIDTH - measured_dist * cos(ang) - (ori.x - rob.x);
            break;
        case HIT_BOTTOM:
            inv.est_y = 0 - measured_dist * sin(ang) - (ori.y - rob.y);
            break;
        case HIT_TOP:
            inv.est_y = FIELD_LENGTH - measured_dist * sin(ang) - (ori.y - rob.y);
            break;
        default:
            break;
    }
    return inv;
}

int main(void) {
    Position rough = {4.0, 7.5};
    double heading = 20.0;
    double measured[NUM_SENSORS] = {6.0, 4.0, 8.0, 5.0};

    printf("\nAnalytic Pair‑Based Localization with Triangulation\n");
    printf(" rough=(%.2f,%.2f), θ=%.1f°\n\n", rough.x, rough.y, heading);

    Inversion invs[NUM_SENSORS];
    Position origins[NUM_SENSORS];
    double angles[NUM_SENSORS];

    // 1) per-sensor inversion and store origins/angles
    for (int i = 0; i < NUM_SENSORS; i++) {
        const Sensor* s = &sensors[i];
        origins[i] = sensor_origin(rough, heading, s);
        angles[i]  = deg2rad(heading + s->dir_off);
        invs[i]    = invert_sensor(rough, heading, s, measured[i]);
    }

    // 2) pairs
    for (int pi = 0; pi < NUM_SENSORS; pi++) {
        Pair p = pairs[pi];
        Inversion A = invs[p.a], B = invs[p.b];
        printf("%s: ", p.name);

        // if one gives x and one y, use analytic
        if (A.valid && B.valid &&
            (!isnan(A.est_x) ^ !isnan(B.est_x)) &&
            (!isnan(A.est_y) ^ !isnan(B.est_y))) {
            double fx = isnan(A.est_x) ? B.est_x : A.est_x;
            double fy = isnan(A.est_y) ? B.est_y : A.est_y;
            printf("analytic => (%.3f, %.3f)\n", fx, fy);
        }
        // else if both valid, triangulate
        else if (A.valid && B.valid) {
            // line intersection: o1 + t1*v1 = o2 + t2*v2
            Position o1 = origins[p.a], o2 = origins[p.b];
            double v1x = cos(angles[p.a]), v1y = sin(angles[p.a]);
            double v2x = cos(angles[p.b]), v2y = sin(angles[p.b]);
            double denom = v1x * v2y - v1y * v2x;
            if (fabs(denom) < EPSILON) {
                printf("triangulate => parallel rays, skip\n");
            } else {
                double dx = o2.x - o1.x;
                double dy = o2.y - o1.y;
                double t1 = (dx * v2y - dy * v2x) / denom;
                Position tri = { o1.x + t1 * v1x,
                                  o1.y + t1 * v1y };
                printf("triangulate => (%.3f, %.3f)\n", tri.x, tri.y);
            }
        } else {
            printf("skip (invalid)\n");
        }
    }

    return 0;
}
