#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>

#define DEG2RAD(a) ((a) * M_PI / 180.0)
#define ROOM_WIDTH 8.0
#define ROOM_LENGTH 15.0

typedef struct {
    double x, y;
} Point;

typedef struct {
    const char* name;
    double direction_offset; // relative to robot heading
    double dx, dy;           // offset from robot center in robot's frame
} Sensor;

Sensor sensors[4] = {
    {"Front",  90.0,  -0.00386, 0.31221},
    {"Right",   0.0,   0.24136, 0.14136},
    {"Back",   270.0, -0.03279, -0.29372},
    {"Left",   180.0, -0.24577, 0.13383}
};

Point find_wall_hit(double room_w, double room_l, Point pos, double angle_deg, double* dist_out, const char* sensor_name) {
    double theta = DEG2RAD(angle_deg);
    double dx = cos(theta);
    double dy = sin(theta);
    double t_min = DBL_MAX;
    Point hit = {-1, -1};
    const char* wall_name = "None";

    if (dx != 0) {
        for (int i = 0; i < 2; i++) {
            double x_wall = (i == 0) ? 0 : room_w;
            double t = (x_wall - pos.x) / dx;
            double y = pos.y + t * dy;
            if (t > 0 && y >= 0 && y <= room_l && t < t_min) {
                t_min = t;
                hit.x = x_wall;
                hit.y = y;
                wall_name = (i == 0) ? "Left Wall" : "Right Wall";
            }
        }
    }

    if (dy != 0) {
        for (int i = 0; i < 2; i++) {
            double y_wall = (i == 0) ? 0 : room_l;
            double t = (y_wall - pos.y) / dy;
            double x = pos.x + t * dx;
            if (t > 0 && x >= 0 && x <= room_w && t < t_min) {
                t_min = t;
                hit.x = x;
                hit.y = y_wall;
                wall_name = (i == 0) ? "Bottom Wall" : "Top Wall";
            }
        }
    }

    *dist_out = t_min;

    // Print debug information
    printf("Sensor %-6s hits %-12s at (%.2f, %.2f), distance = %.2f\n",
           sensor_name, wall_name, hit.x, hit.y, t_min);

    return hit;
}

int get_sensor_index(const char* name) {
    for (int i = 0; i < 4; i++)
        if (strcmp(sensors[i].name, name) == 0)
            return i;
    return -1;
}

double compute_total_error(Point pos, double theta, int s1, int s2, double desired1, double desired2) {
    double dist1, dist2;

    double angle_rad = DEG2RAD(theta);
    double cos_theta = cos(angle_rad);
    double sin_theta = sin(angle_rad);

    Point sensor1_pos = {
        pos.x + sensors[s1].dx * cos_theta - sensors[s1].dy * sin_theta,
        pos.y + sensors[s1].dx * sin_theta + sensors[s1].dy * cos_theta
    };

    Point sensor2_pos = {
        pos.x + sensors[s2].dx * cos_theta - sensors[s2].dy * sin_theta,
        pos.y + sensors[s2].dx * sin_theta + sensors[s2].dy * cos_theta
    };

    find_wall_hit(ROOM_WIDTH, ROOM_LENGTH, sensor1_pos, theta + sensors[s1].direction_offset, &dist1, sensors[s1].name);
    find_wall_hit(ROOM_WIDTH, ROOM_LENGTH, sensor2_pos, theta + sensors[s2].direction_offset, &dist2, sensors[s2].name);

    return fabs(dist1 - desired1) + fabs(dist2 - desired2);
}

void get_inputs(double readings[4], char sensor1_name[10], char sensor2_name[10], double* heading_deg) {
    printf("Enter sensor readings (Front, Right, Back, Left):\n");
    for (int i = 0; i < 4; i++) {
        printf("%s: ", sensors[i].name);
        scanf("%lf", &readings[i]);
    }

    printf("Enter sensor 1 to use (Front/Right/Back/Left): ");
    scanf("%s", sensor1_name);
    printf("Enter sensor 2 to use (Front/Right/Back/Left): ");
    scanf("%s", sensor2_name);
    printf("Enter robot heading (in degrees): ");
    scanf("%lf", heading_deg);
}

int is_valid_sensor_selection(int s1, int s2) {
    return (s1 != -1 && s2 != -1 && s1 != s2);
}

Point estimate_position(int s1, int s2, double desired1, double desired2, double heading_deg, double tolerance) {
    double best_x = 0, best_y = 0, min_error = DBL_MAX;

    // Coarse search
    for (double x = 0.0; x <= ROOM_WIDTH; x += 0.2) {
        for (double y = 0.0; y <= ROOM_LENGTH; y += 0.2) {
            Point pos = {x, y};
            double error = compute_total_error(pos, heading_deg, s1, s2, desired1, desired2);
            if (error < min_error) {
                min_error = error;
                best_x = x;
                best_y = y;
            }
        }
    }

    // Fine search
    double final_x = 0, final_y = 0;
    min_error = DBL_MAX;
    for (double x = best_x - 0.2; x <= best_x + 0.2; x += 0.01) {
        for (double y = best_y - 0.2; y <= best_y + 0.2; y += 0.01) {
            if (x < 0 || x > ROOM_WIDTH || y < 0 || y > ROOM_LENGTH) continue;
            Point pos = {x, y};
            double error = compute_total_error(pos, heading_deg, s1, s2, desired1, desired2);
            if (error < min_error) {
                min_error = error;
                final_x = x;
                final_y = y;
            }
        }
    }

    if (min_error < tolerance) {
        printf("Estimated Position: x = %.2f, y = %.2f (error = %.4f)\n", final_x, final_y, min_error);
    } else {
        printf("Unable to determine position with acceptable accuracy (min error = %.4f).\n", min_error);
    }

    Point final_pos = {final_x, final_y};
    return final_pos;
}

int main() {
    double readings[4];
    char sensor1_name[10], sensor2_name[10];
    double heading_deg;

    get_inputs(readings, sensor1_name, sensor2_name, &heading_deg);

    int s1 = get_sensor_index(sensor1_name);
    int s2 = get_sensor_index(sensor2_name);

    if (!is_valid_sensor_selection(s1, s2)) {
        printf("Invalid sensor selection.\n");
        return 1;
    }

    double desired1 = readings[s1];
    double desired2 = readings[s2];
    double tolerance = 0.05 * (desired1 + desired2) / 2;

    estimate_position(s1, s2, desired1, desired2, heading_deg, tolerance);

    return 0;
}
