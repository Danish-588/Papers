#include <stdio.h>
#include <math.h>
#include <float.h>
#include <stdbool.h>

#define TO_RAD(deg) ((deg) * M_PI / 180.0)

#define NUM_SENSORS  4
#define NUM_PAIRS    4
#define WIDTH 8.0
#define LENGTH 15.0

#define BORDER_MARGIN 0.6   
#define CORNER_MARGIN 1.0 
#define FINAL_MARGIN  0.1

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum 
{
    VALID    = 0,
    BORDER   = 1,
    CORNER   = 2,
    MAXVAL   = 3,
    BLOCKED  = 4,
}
SensorStatus;

typedef enum
{
    FRONT = 0,
    RIGHT = 1,
    BACK  = 2,
    LEFT  = 3
}
Direction;

enum 
{
    FRONT_LEFT = 0,
    FRONT_RIGHT = 1,
    BACK_LEFT = 2,
    BACK_RIGHT = 3
}
PairIndex;

SensorStatus sensor_status[4] = {BLOCKED, BLOCKED, BLOCKED, BLOCKED};

typedef struct { double x, y, theta; } Pose;
volatile struct { double x, y; Direction wall; } sensor_hits[NUM_SENSORS];
typedef struct { int s1, s2; const char* name; } SensorPair;

volatile Pose sensor_tf[NUM_SENSORS] = 
{
    {-0.00386,  0.312219, 90.0 },
    {0.24136 ,  0.14136 , 0.0  },
    {-0.03279, -0.29372 , 270.0},
    {-0.24577,  0.13383 , 180.0},
};

volatile Pose rough_pose = {0, 0, 0};
volatile Pose best_pose  = {0, 0, 0};
volatile double angle = 0.0;

volatile Pose expected_sensor[NUM_SENSORS]         = {0};
volatile Pose refined_sensor[NUM_SENSORS]         = {0};
volatile Pose refined[NUM_SENSORS]        = {0};

double expected[NUM_SENSORS]     = {0};
double measured[NUM_SENSORS]     = {0};

double diff[NUM_SENSORS]         = {0};
bool   valid_pairs[NUM_PAIRS]    = {0};

void robot_to_sensor();
void predict_wall_hit();
void is_sensor_valid();
void is_pair_valid();
void calculate_sensor_pose();
void sensor_to_robot();
void find_best_pose();

void robot_to_sensor() 
{
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        expected_sensor[i].x      = rough_pose.x + (sensor_tf[i].x * cos(angle)) - (sensor_tf[i].y * sin(angle));
        expected_sensor[i].y      = rough_pose.y + (sensor_tf[i].x * sin(angle)) + (sensor_tf[i].y * cos(angle));
        expected_sensor[i].theta  = rough_pose.theta + sensor_tf[i].theta;
    }
}

void predict_wall_hit()
{
    for (Direction i = 0; i < NUM_SENSORS; i++)
    {
        double theta = TO_RAD(expected_sensor[i].theta);
        double dx    = cos(theta);
        double dy    = sin(theta);
        
        double t_min = DBL_MAX;
        double t, x_hit, y_hit;
        
        // Vertical walls
        if (fabs(dx) > 1e-9)
        {
            // Left wall (x = 0)
            t     = (0.0   - expected_sensor[i].x) / dx;
            y_hit = expected_sensor[i].y + t * dy;
            if (t > 0.0 && y_hit >= 0.0 && y_hit <= LENGTH && t < t_min)
            {
                t_min                     = t;
                sensor_hits[i].x          = 0.0;
                sensor_hits[i].y          = y_hit;
                sensor_hits[i].wall       = LEFT; 
            }
            
            // Right wall (x = WIDTH)
            t     = (WIDTH - expected_sensor[i].x) / dx;
            y_hit = expected_sensor[i].y + t * dy;
            if (t > 0.0 && y_hit >= 0.0 && y_hit <= LENGTH && t < t_min)
            {
                t_min                     = t;
                sensor_hits[i].x          = WIDTH;
                sensor_hits[i].y          = y_hit;
                sensor_hits[i].wall       = RIGHT; 
            }
        }
        
        // Horizontal walls
        if (fabs(dy) > 1e-9)
        {
            // Bottom wall (y = 0)
            t     = (0.0   - expected_sensor[i].y) / dy;
            x_hit = expected_sensor[i].x + t * dx;
            if (t > 0.0 && x_hit >= 0.0 && x_hit <= WIDTH && t < t_min)
            {
                t_min                     = t;
                sensor_hits[i].x          = x_hit;
                sensor_hits[i].y          = 0.0;
                sensor_hits[i].wall       = BACK; 
            }
            
            // Top wall (y = LENGTH)
            t     = (LENGTH - expected_sensor[i].y) / dy;
            x_hit = expected_sensor[i].x + t * dx;
            if (t > 0.0 && x_hit >= 0.0 && x_hit <= WIDTH && t < t_min)
            {
                t_min                     = t;
                sensor_hits[i].x          = x_hit;
                sensor_hits[i].y          = LENGTH;
                sensor_hits[i].wall       = FRONT;
            }
        }
        expected[i]           = t_min;
        diff[i]              = measured[i] - expected[i];
    }
}

void is_sensor_valid()
{
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        // Check for BORDER
        if (rough_pose.x < (0 + BORDER_MARGIN) || rough_pose.x > WIDTH - BORDER_MARGIN || 
        rough_pose.y < (0 + BORDER_MARGIN) || rough_pose.y > LENGTH - BORDER_MARGIN)
        {
            sensor_status[i] = BORDER;
        }
        
        // Check if near MAX VALUE
        else if (measured[i] > 10.0) 
        {
            sensor_status[i] = MAXVAL;
        }
        
        // Check for CORNER
        else if ((sensor_hits[i].x < CORNER_MARGIN || sensor_hits[i].x > WIDTH - CORNER_MARGIN) &&
        (sensor_hits[i].y < CORNER_MARGIN || sensor_hits[i].y > LENGTH - CORNER_MARGIN))
        {
            sensor_status[i] = CORNER;
        }
        
        // Check for BLOCKED
        else if (fabs(diff[i]) > 0.5) 
        {
            sensor_status[i] = BLOCKED;
        } 
        
        // If none of the above, set to VALID
        else 
        {
            sensor_status[i] = VALID;
        }
    }
}

void is_pair_valid()
{
    valid_pairs[FRONT_LEFT]  = (sensor_status[FRONT] == VALID && sensor_status[LEFT]  == VALID);
    valid_pairs[FRONT_RIGHT] = (sensor_status[FRONT] == VALID && sensor_status[RIGHT] == VALID);
    valid_pairs[BACK_LEFT]   = (sensor_status[BACK]  == VALID && sensor_status[LEFT]  == VALID);
    valid_pairs[BACK_RIGHT]  = (sensor_status[BACK]  == VALID && sensor_status[RIGHT] == VALID);
}

void calculate_sensor_pose()
{
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensor_status[i] != VALID)
        continue;
        
        double theta = TO_RAD(expected_sensor[i].theta);
        
        switch (sensor_hits[i].wall)
        {
            case LEFT:
            refined_sensor[i].x = 0.0   - measured[i] * cos(theta);
            break;
            
            case RIGHT:
            refined_sensor[i].x = WIDTH - measured[i] * cos(theta);
            break;
            
            case BACK:   
            refined_sensor[i].y = 0.0   - measured[i] * sin(theta);
            break;
            
            case FRONT: 
            refined_sensor[i].y = LENGTH - measured[i] * sin(theta);
            break;
    
            default:
            continue;
        }
        refined_sensor[i].theta = expected_sensor[i].theta;
    }
}
    
void sensor_to_robot() 
{
    for (int i = 0; i < NUM_SENSORS; i++) 
    {
        refined[i].x      = refined_sensor[i].x - (sensor_tf[i].x * cos(angle)) + (sensor_tf[i].y * sin(angle));
        refined[i].y      = refined_sensor[i].y - (sensor_tf[i].x * sin(angle)) - (sensor_tf[i].y * cos(angle));
        refined[i].theta  = refined_sensor[i].theta - sensor_tf[i].theta;
    }
}

void find_best_pose()
{
    double best_x = rough_pose.x;
    double best_y = rough_pose.y;
    double min_dx  = DBL_MAX;
    double min_dy  = DBL_MAX;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (sensor_status[i] != VALID)
            continue;

        double cx = refined[i].x;
        double cy = refined[i].y;

        double dx = fabs(cx - rough_pose.x);
        double dy = fabs(cy - rough_pose.y);

        if (dx < min_dx)
        {
            min_dx = dx;
            best_x = cx;
        }
        if (dy < min_dy)
        {
            min_dy = dy;
            best_y = cy;
        }
    }

    if (fabs(min_dx) < FINAL_MARGIN)
        best_pose.x = best_x;
    else 
        best_pose.x = rough_pose.x;

    if (fabs(min_dy) < FINAL_MARGIN)
        best_pose.y = best_y;
    else 
        best_pose.y = rough_pose.y;

    best_pose.theta = rough_pose.theta;
}

int main() 
{
    rough_pose.x = 4.0;
    rough_pose.y = 4.0;
    rough_pose.theta = 169.0;
    angle = TO_RAD(rough_pose.theta);

    measured[FRONT] = 3.7; 
    measured[RIGHT] = 3.8; 
    measured[BACK]  = 10.9;
    measured[LEFT]  = 3.8; 

    robot_to_sensor();
    predict_wall_hit();
    is_sensor_valid();
    // is_pair_valid();
    calculate_sensor_pose();
    sensor_to_robot();
    find_best_pose();

    return 0;
}
