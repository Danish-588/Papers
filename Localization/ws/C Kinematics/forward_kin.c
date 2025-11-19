#include <stdio.h>
#include <math.h>
#define DEG2RAD(angle) ((angle) * M_PI / 180.0)

typedef struct {
    double x, y;
} Point;

typedef struct {
    const char* name;
    double angle_offset;
} Laser;

void find_wall_hit(double room_w, double room_l, Point pos, double theta_deg, const char* label, double angle_deg) {
    double theta = DEG2RAD(angle_deg);
    double dx = cos(theta);
    double dy = sin(theta);

    double t_min = 1e9;
    const char* wall_hit = "none";
    Point hit = {0, 0};

    // LEFT wall (x=0)
    if (dx != 0) {
        double t = (0 - pos.x) / dx;
        double y_hit = pos.y + t * dy;
        if (t > 0 && y_hit >= 0 && y_hit <= room_l && t < t_min) {
            t_min = t;
            wall_hit = "left";
            hit.x = 0;
            hit.y = y_hit;
        }
    }

    // RIGHT wall (x=room_w)
    if (dx != 0) {
        double t = (room_w - pos.x) / dx;
        double y_hit = pos.y + t * dy;
        if (t > 0 && y_hit >= 0 && y_hit <= room_l && t < t_min) {
            t_min = t;
            wall_hit = "right";
            hit.x = room_w;
            hit.y = y_hit;
        }
    }

    // BOTTOM wall (y=0)
    if (dy != 0) {
        double t = (0 - pos.y) / dy;
        double x_hit = pos.x + t * dx;
        if (t > 0 && x_hit >= 0 && x_hit <= room_w && t < t_min) {
            t_min = t;
            wall_hit = "bottom";
            hit.x = x_hit;
            hit.y = 0;
        }
    }

    // TOP wall (y=room_l)
    if (dy != 0) {
        double t = (room_l - pos.y) / dy;
        double x_hit = pos.x + t * dx;
        if (t > 0 && x_hit >= 0 && x_hit <= room_w && t < t_min) {
            t_min = t;
            wall_hit = "top";
            hit.x = x_hit;
            hit.y = room_l;
        }
    }

    if (t_min < 1e9) {
        printf("Laser %-6s hits %-6s wall at (%.3f, %.3f), distance = %.3f\n", label, wall_hit, hit.x, hit.y, t_min);
    } else {
        printf("Laser %-6s hits nothing.\n", label);
    }
}

int main() {
    double room_width = 8.0;
    double room_length = 15.0;

    Point robot = {3, 2};
    double robot_theta = 0.0; // in degrees

    Laser lasers[4] = {
        {"Front", 90},
        {"Right", 0},
        {"Back", 270},
        {"Left", 180}
    };

    for (int i = 0; i < 4; ++i) {
        double laser_angle = robot_theta + lasers[i].angle_offset;
        find_wall_hit(room_width, room_length, robot, robot_theta, lasers[i].name, laser_angle);
    }

    return 0;
}