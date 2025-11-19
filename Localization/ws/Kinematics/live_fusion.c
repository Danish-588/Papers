#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdbool.h>

// FIELD dimensions and search parameters
#define FIELD_WIDTH   8.0
#define FIELD_LENGTH  15.0
#define NUM_SENSORS  4
#define NUM_PAIRS    4
#define SEARCH_MARGIN 0.5
#define COARSE_STEP   0.2
#define FINE_MARGIN   0.2
#define FINE_STEP     0.01

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double x, y; } Position;
typedef struct { const char* name; double dir_off, dx, dy; } Sensor;
typedef struct { int s1, s2; const char* name; } SensorPair;

// Static sensor definitions
static const Sensor sensors[NUM_SENSORS] = {
    {"Front",  90.0,  0.0,  0.28},
    {"Right",   0.0,  0.18, 0.10},
    {"Back",   270.0,  0.0, -0.28},
    {"Left",   180.0, -0.18, 0.10}
};
static const SensorPair sensor_pairs[NUM_PAIRS] = {
    {0, 3, "Front+Left"},  {0, 1, "Front+Right"},
    {2, 3, "Back+Left"},   {2, 1, "Back+Right"}
};

// Monitoring globals
Position rough_pos = {4.0, 7.5};
double heading_deg = 45.0;
double measured[NUM_SENSORS]    = {6.0, 4.0, 8.0, 5.0};
double expected[NUM_SENSORS]    = {0};
double diff[NUM_SENSORS]        = {0};
bool   sensor_valid[NUM_SENSORS] = {0};
bool   valid[NUM_PAIRS]         = {0};
double error_arr[NUM_PAIRS]     = {0};
Position refined[NUM_PAIRS]      = {{0}};
double final_err[NUM_PAIRS]     = {0};
int    best_idx = -1;
Position best_pos = {0, 0};
double best_err = DBL_MAX;
bool   found = false;
unsigned long counter = 0;
volatile bool running = true;

// Utility: degrees to radians
static inline double deg2rad(double d) {
    return d * M_PI / 180.0;
}

// Compute ray intersection with walls
double find_wall_hit(Position p, double angle) {
    double th = deg2rad(angle), dx = cos(th), dy = sin(th), tmin = DBL_MAX;
    if (dx != 0) {
        double t = (0 - p.x) / dx, y = p.y + t * dy;
        if (t > 0 && y >= 0 && y <= FIELD_LENGTH && t < tmin) tmin = t;
        t = (FIELD_WIDTH - p.x) / dx; y = p.y + t * dy;
        if (t > 0 && y >= 0 && y <= FIELD_LENGTH && t < tmin) tmin = t;
    }
    if (dy != 0) {
        double t = (0 - p.y) / dy, x = p.x + t * dx;
        if (t > 0 && x >= 0 && x <= FIELD_WIDTH && t < tmin) tmin = t;
        t = (FIELD_LENGTH - p.y) / dy; x = p.x + t * dx;
        if (t > 0 && x >= 0 && x <= FIELD_WIDTH && t < tmin) tmin = t;
    }
    return tmin;
}

// Predict sensor distance from any position
static inline double predict_sensor_reading_at(Position pos, int idx) {
    Sensor s = sensors[idx];
    double a = deg2rad(heading_deg);
    Position wp = {
        pos.x + s.dx * cos(a) - s.dy * sin(a),
        pos.y + s.dx * sin(a) + s.dy * cos(a)
    };
    return find_wall_hit(wp, heading_deg + s.dir_off);
}

// Helper: use rough_pos for default
static inline double predict_sensor_reading(int idx) {
    return predict_sensor_reading_at(rough_pos, idx);
}

// Compute pair error at rough_pos
static inline double compute_pair_error(int s1, int s2) {
    return fabs(predict_sensor_reading(s1) - measured[s1])
         + fabs(predict_sensor_reading(s2) - measured[s2]);
}

// Compute pair error at arbitrary pos
static inline double compute_pair_error_at(Position pos, int s1, int s2) {
    return fabs(predict_sensor_reading_at(pos, s1) - measured[s1])
             + fabs(predict_sensor_reading_at(pos, s2) - measured[s2]);
}

// Inverse localization for a pair
Position inverse_localize(int s1, int s2, double* out_err) {
    double min_err = DBL_MAX;
    Position best = {0,0}, ref = {0,0};

    double x0 = fmax(0, rough_pos.x - SEARCH_MARGIN);
    double x1 = fmin(FIELD_WIDTH, rough_pos.x + SEARCH_MARGIN);
    double y0 = fmax(0, rough_pos.y - SEARCH_MARGIN);
    double y1 = fmin(FIELD_LENGTH, rough_pos.y + SEARCH_MARGIN);

    // Coarse search
    for (double x = x0; x <= x1; x += COARSE_STEP) {
         for (double y = y0; y <= y1; y += COARSE_STEP) {
            Position p = {x, y};
            double e = compute_pair_error_at(p, s1, s2);
            if (e < min_err) { min_err = e; best = p; }
        }
    }

    // Fine search
    min_err = DBL_MAX;
    double xs = fmax(x0, best.x - FINE_MARGIN);
    double xe = fmin(x1, best.x + FINE_MARGIN);
    double ys = fmax(y0, best.y - FINE_MARGIN);
    double ye = fmin(y1, best.y + FINE_MARGIN);

    for (double x = xs; x <= xe; x += FINE_STEP) {
        for (double y = ys; y <= ye; y += FINE_STEP) {
            Position p = {x, y};
            double e = compute_pair_error_at(p, s1, s2);
            if (e < min_err) { min_err = e; ref = p; }
        }
    }

    if (out_err) *out_err = min_err;
    return ref;
}

// Update all globals
void update_data() {
    // readings, diffs, and individual validity
    for (int i = 0; i < NUM_SENSORS; i++) {
        expected[i] = predict_sensor_reading(i);
        diff[i]     = measured[i] - expected[i];
        double tol  = 0.5;
        sensor_valid[i] = fabs(diff[i]) < tol;
    }

    // Reset best pair
    best_idx = -1;
    best_err = DBL_MAX;
    found    = false;

    // Evaluate pairs
    for (int i = 0; i < NUM_PAIRS; i++) {
        int s1 = sensor_pairs[i].s1;
        int s2 = sensor_pairs[i].s2;
        error_arr[i] = compute_pair_error(s1, s2);
        valid[i]     = sensor_valid[s1] && sensor_valid[s2];
        refined[i]   = inverse_localize(s1, s2, &final_err[i]);

        if (valid[i] && final_err[i] < best_err) {
            best_err = final_err[i];
            best_idx = i;
            best_pos = refined[i];
            found    = true;
        }
    }

    counter++;
}

// Print status
void print_status() {
    printf("\033[2J\033[HUpdate #%lu\n", counter);
    printf("Rough pos: (%.2f,%.2f), Heading: %.1f°\n", rough_pos.x, rough_pos.y, heading_deg);
    printf("Sensor | Meas  Exp   Diff  Valid\n");
    for (int i = 0; i < NUM_SENSORS; i++) {
        printf("%-6s |%6.2f %6.2f %6.2f    %c\n",
               sensors[i].name,
               measured[i], expected[i], diff[i],
               sensor_valid[i] ? 'Y' : 'N');
    }

    printf("Pairs:\n");
    for (int i = 0; i < NUM_PAIRS; i++) {
        printf("%-12s |Err:%6.2f V:%c Ref:(%.2f,%.2f) FE:%6.3f %s\n",
               sensor_pairs[i].name,
               error_arr[i], valid[i] ? 'Y' : 'N',
               refined[i].x, refined[i].y,
               final_err[i], best_idx == i ? "<" : "");
    }

    if (found) {
        printf("Best: %s at (%.2f,%.2f) Err:%.3f\n",
               sensor_pairs[best_idx].name,
               best_pos.x, best_pos.y,
               best_err);
    } else {
        printf("No valid pair found\n");
    }
}

int main() {
    printf("Robot Localization - Live Monitoring\n");
    while (running) {
        update_data();
        print_status();
    }
    return 0;
}
