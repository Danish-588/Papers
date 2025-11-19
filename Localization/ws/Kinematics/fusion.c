#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

// Room dimensions (meters)
#define ROOM_WIDTH 8.0
#define ROOM_LENGTH 15.0

// Sensor structure definition
typedef struct {
    const char* name;
    double direction_offset;  // degrees relative to robot heading
    double dx;                // x offset from robot center
    double dy;                // y offset from robot center
} Sensor;

// Robot position structure
typedef struct {
    double x;
    double y;
} Position;

// Define sensors (relative to robot frame)
static const Sensor sensors[] = {
    {"Front", 90.0,  0.0,  0.28},
    {"Right", 0.0,   0.18,  0.10},
    {"Back",  270.0, 0.0, -0.28},
    {"Left",  180.0, -0.18, 0.10}
};

const int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// Convert degrees to radians
double deg2rad(double angle_deg) {
    return angle_deg * M_PI / 180.0;
}

// Calculate distance to wall hit from a given position and angle
double find_wall_hit(Position pos, double angle_deg, double room_w, double room_l) {
    double theta = deg2rad(angle_deg);
    double dx = cos(theta);
    double dy = sin(theta);
    double t_min = DBL_MAX;
    
    // Vertical walls:
    if (dx != 0) {
        // Left wall (x = 0)
        double t = (0 - pos.x) / dx;
        double y_hit = pos.y + t * dy;
        if (t > 0 && 0 <= y_hit && y_hit <= room_l && t < t_min) {
            t_min = t;
        }
        // Right wall (x = room_w)
        t = (room_w - pos.x) / dx;
        y_hit = pos.y + t * dy;
        if (t > 0 && 0 <= y_hit && y_hit <= room_l && t < t_min) {
            t_min = t;
        }
    }

    // Horizontal walls:
    if (dy != 0) {
        // Bottom wall (y = 0)
        double t = (0 - pos.y) / dy;
        double x_hit = pos.x + t * dx;
        if (t > 0 && 0 <= x_hit && x_hit <= room_w && t < t_min) {
            t_min = t;
        }
        // Top wall (y = room_l)
        t = (room_l - pos.y) / dy;
        x_hit = pos.x + t * dx;
        if (t > 0 && 0 <= x_hit && x_hit <= room_w && t < t_min) {
            t_min = t;
        }
    }

    return t_min;
}

// Predict sensor reading for a given position, heading, and sensor
double predict_sensor_reading(Position pos, double heading_deg, int sensor_index, Position* sensor_world_pos, double* beam_angle) {
    const Sensor sensor = sensors[sensor_index];
    double angle_rad = deg2rad(heading_deg);
    Position sensor_world;
    
    // Calculate sensor position in world coordinates
    sensor_world.x = pos.x + sensor.dx * cos(angle_rad) - sensor.dy * sin(angle_rad);
    sensor_world.y = pos.y + sensor.dx * sin(angle_rad) + sensor.dy * cos(angle_rad);
    
    // Calculate beam angle
    double beam_angle_deg = heading_deg + sensor.direction_offset;
    
    // Find distance to nearest wall
    double distance = find_wall_hit(sensor_world, beam_angle_deg, ROOM_WIDTH, ROOM_LENGTH);
    
    // Return sensor world position and beam angle if requested
    if (sensor_world_pos != NULL) {
        *sensor_world_pos = sensor_world;
    }
    if (beam_angle != NULL) {
        *beam_angle = beam_angle_deg;
    }
    
    return distance;
}

// Calculate error between predicted and measured sensor readings for a pair of sensors
double compute_pair_error(Position pos, double heading_deg, int sensor1_index, int sensor2_index, double measured1, double measured2) {
    double pred1 = predict_sensor_reading(pos, heading_deg, sensor1_index, NULL, NULL);
    double pred2 = predict_sensor_reading(pos, heading_deg, sensor2_index, NULL, NULL);
    double error = fabs(pred1 - measured1) + fabs(pred2 - measured2);
    return error;
}

// Inverse localization: estimate position based on two sensor readings
Position inverse_localize(double heading_deg, int sensor1_index, int sensor2_index, double measured1, double measured2, Position rough_pos, double* final_error) {
    // Search in a window ±0.5 m around the rough estimate
    double search_margin = 0.5;
    double x_min = fmax(0, rough_pos.x - search_margin);
    double x_max = fmin(ROOM_WIDTH, rough_pos.x + search_margin);
    double y_min = fmax(0, rough_pos.y - search_margin);
    double y_max = fmin(ROOM_LENGTH, rough_pos.y + search_margin);
    double coarse_step = 0.2;

    // Coarse search
    double best_x = 0, best_y = 0;
    double min_error = DBL_MAX;
    Position pos;
    
    for (double x = x_min; x <= x_max; x += coarse_step) {
        for (double y = y_min; y <= y_max; y += coarse_step) {
            pos.x = x;
            pos.y = y;
            double error = compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2);
            if (error < min_error) {
                min_error = error;
                best_x = x;
                best_y = y;
            }
        }
    }

    // Fine search around best coarse match
    double fine_margin = 0.2;
    double fine_step = 0.01;
    double refined_x = best_x, refined_y = best_y;
    min_error = DBL_MAX;
    
    double x_start = fmax(x_min, best_x - fine_margin);
    double x_end = fmin(x_max, best_x + fine_margin);
    double y_start = fmax(y_min, best_y - fine_margin);
    double y_end = fmin(y_max, best_y + fine_margin);
    
    for (double x = x_start; x <= x_end; x += fine_step) {
        for (double y = y_start; y <= y_end; y += fine_step) {
            pos.x = x;
            pos.y = y;
            double error = compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2);
            if (error < min_error) {
                min_error = error;
                refined_x = x;
                refined_y = y;
            }
        }
    }

    // Set final error if pointer provided
    if (final_error != NULL) {
        *final_error = min_error;
    }

    // Return refined position
    Position refined_pos = {refined_x, refined_y};
    return refined_pos;
}

// Function to print expected sensor readings for a given position and heading
void print_expected_readings(Position pos, double heading_deg) {
    printf("Expected sensor readings for position (%.2f, %.2f) at heading %.2f degrees:\n", 
           pos.x, pos.y, heading_deg);
           
    for (int i = 0; i < NUM_SENSORS; i++) {
        double distance = predict_sensor_reading(pos, heading_deg, i, NULL, NULL);
        printf("  %s: %.2f m\n", sensors[i].name, distance);
    }
}

// Test the localization with a sample scenario
void test_localization() {
    // Define a "true" position and heading
    Position true_pos = {6.9, 4.2};
    double true_heading = 30.0;
    
    // Calculate expected sensor readings at this position
    printf("== TRUE POSITION ==\n");
    print_expected_readings(true_pos, true_heading);
    
    // Get sensor readings (in a real system, these would come from physical sensors)
    double measured_front = predict_sensor_reading(true_pos, true_heading, 0, NULL, NULL);
    double measured_left = predict_sensor_reading(true_pos, true_heading, 3, NULL, NULL);
    
    // Add some noise to simulate real-world measurements
    measured_front += 0.2;  // 20cm error
    measured_left -= 0.15;  // 15cm error
    
    printf("\n== MEASURED VALUES (with simulated noise) ==\n");
    printf("  Front: %.2f m\n", measured_front);
    printf("  Left: %.2f m\n", measured_left);
    
    // Define a rough estimate of position (in a real system, this might come from odometry)
    Position rough_pos = {7, 4};  // Slightly off from true position
    
    printf("\n== ROUGH POSITION ESTIMATE ==\n");
    printf("  Position: (%.2f, %.2f)\n", rough_pos.x, rough_pos.y);
    print_expected_readings(rough_pos, true_heading);
    
    // Run inverse localization
    double error;
    Position refined_pos = inverse_localize(true_heading, 0, 3, measured_front, measured_left, rough_pos, &error);
    
    printf("\n== LOCALIZATION RESULT ==\n");
    printf("  Refined position: (%.2f, %.2f)\n", refined_pos.x, refined_pos.y);
    printf("  Error: %.3f m\n", error);
    printf("  Distance from true position: %.3f m\n", 
           sqrt(pow(refined_pos.x - true_pos.x, 2) + pow(refined_pos.y - true_pos.y, 2)));
    
    // Print expected readings at refined position
    printf("\n== EXPECTED READINGS AT REFINED POSITION ==\n");
    print_expected_readings(refined_pos, true_heading);
}

int main() {
    printf("Robot Localization Test\n");
    printf("======================\n\n");
    
    test_localization();
    
    return 0;
}