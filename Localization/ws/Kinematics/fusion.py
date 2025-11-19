import math
import sys

# Constants for room dimensions (meters)
ROOM_WIDTH = 8.0
ROOM_LENGTH = 15.0

# Convert degrees to radians
def deg2rad(a):
    return a * math.pi / 180.0

# Define sensor configurations as a list of dictionaries.
# Each sensor has:
# - name: used for identification
# - direction_offset: the angle (in degrees) relative to the robot heading at which the sensor measures distance
# - dx, dy: the sensor's position relative to the robot center (in the robot’s coordinate frame)
sensors = [
    {"name": "Front", "direction_offset": 90,  "dx": 0.0,  "dy": 0.8},
    {"name": "Right", "direction_offset": 0,   "dx": 0.8,  "dy": 0.0},
    {"name": "Back",  "direction_offset": 270, "dx": 0.0,  "dy": -0.8},
    {"name": "Left",  "direction_offset": 180, "dx": -0.8, "dy": 0.0}
]

# A helper function to compute the wall hit distance from a given starting point and angle
def find_wall_hit(pos, angle_deg, room_w=ROOM_WIDTH, room_l=ROOM_LENGTH):
    theta = deg2rad(angle_deg)
    dx = math.cos(theta)
    dy = math.sin(theta)
    t_min = float('inf')
    
    # Check intersection with vertical walls (x = 0 and x = room_w)
    if dx != 0:
        # Left wall: x = 0
        t = (0 - pos["x"]) / dx
        y_hit = pos["y"] + t * dy
        if t > 0 and 0 <= y_hit <= room_l and t < t_min:
            t_min = t

        # Right wall: x = room_w
        t = (room_w - pos["x"]) / dx
        y_hit = pos["y"] + t * dy
        if t > 0 and 0 <= y_hit <= room_l and t < t_min:
            t_min = t

    # Check intersection with horizontal walls (y = 0 and y = room_l)
    if dy != 0:
        # Bottom wall: y = 0
        t = (0 - pos["y"]) / dy
        x_hit = pos["x"] + t * dx
        if t > 0 and 0 <= x_hit <= room_w and t < t_min:
            t_min = t

        # Top wall: y = room_l
        t = (room_l - pos["y"]) / dy
        x_hit = pos["x"] + t * dx
        if t > 0 and 0 <= x_hit <= room_w and t < t_min:
            t_min = t

    return t_min

# Given a robot position (pos with keys x and y) and heading (in degrees), predict the sensor reading
# for a sensor specified by its index in the sensors list.
def predict_sensor_reading(pos, heading_deg, sensor_index):
    sensor = sensors[sensor_index]
    # Convert heading to radians
    angle_rad = deg2rad(heading_deg)
    # Rotate the sensor's offset (dx, dy) by the robot's heading.
    # (Assume the sensor offset is expressed in the robot's coordinate frame.)
    sensor_world = {
        "x": pos["x"] + sensor["dx"] * math.cos(angle_rad) - sensor["dy"] * math.sin(angle_rad),
        "y": pos["y"] + sensor["dx"] * math.sin(angle_rad) + sensor["dy"] * math.cos(angle_rad)
    }
    # Sensor beam direction = robot's heading + sensor's direction offset
    beam_angle = heading_deg + sensor["direction_offset"]
    distance = find_wall_hit(sensor_world, beam_angle)
    return distance

# Compute the total error (absolute difference) between predicted and measured sensor readings for a given sensor pair.
def compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2):
    pred1 = predict_sensor_reading(pos, heading_deg, sensor1_index)
    pred2 = predict_sensor_reading(pos, heading_deg, sensor2_index)
    error = abs(pred1 - measured1) + abs(pred2 - measured2)
    return error

# Inverse localization: given sensor readings for a selected sensor pair,
# perform a coarse and then a fine grid search around the rough position to find a refined estimate.
def inverse_localize(heading_deg, sensor1_index, sensor2_index, measured1, measured2, rough_pos):
    # Define search bounds around the rough estimate.
    # Since rough_pos is within ±0.5 m, we search in a window of ±0.5 m.
    search_margin = 0.5
    x_min = max(0, rough_pos["x"] - search_margin)
    x_max = min(ROOM_WIDTH, rough_pos["x"] + search_margin)
    y_min = max(0, rough_pos["y"] - search_margin)
    y_max = min(ROOM_LENGTH, rough_pos["y"] + search_margin)

    # Phase 1: Coarse search with 0.2 m step
    coarse_step = 0.2
    best_x, best_y = None, None
    min_error = float('inf')
    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            pos = {"x": x, "y": y}
            error = compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2)
            if error < min_error:
                min_error = error
                best_x, best_y = x, y
            y += coarse_step
        x += coarse_step

    # Phase 2: Fine search around the best coarse position (±0.2 m) with 0.01 m step
    fine_margin = 0.2
    fine_step = 0.01
    refined_x, refined_y = best_x, best_y
    min_error = float('inf')
    x_start = max(x_min, best_x - fine_margin)
    x_end = min(x_max, best_x + fine_margin)
    y_start = max(y_min, best_y - fine_margin)
    y_end = min(y_max, best_y + fine_margin)
    x = x_start
    while x <= x_end:
        y = y_start
        while y <= y_end:
            pos = {"x": x, "y": y}
            error = compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2)
            if error < min_error:
                min_error = error
                refined_x, refined_y = x, y
            y += fine_step
        x += fine_step

    return refined_x, refined_y, min_error

def main():
    # Ask for measured sensor readings (assumed to be highly accurate in open space)
    measured_readings = {}
    for sensor in sensors:
        try:
            reading = float(input(f"Enter sensor reading for {sensor['name']}: "))
        except ValueError:
            sys.exit("Invalid input.")
        measured_readings[sensor["name"]] = reading

    # Ask for rough position estimate (from other source, ±0.5 m accuracy)
    try:
        rough_x = float(input("Enter rough x coordinate (m): "))
        rough_y = float(input("Enter rough y coordinate (m): "))
    except ValueError:
        sys.exit("Invalid input.")
    rough_pos = {"x": rough_x, "y": rough_y}

    # Ask for robot heading in degrees
    try:
        heading_deg = float(input("Enter robot heading (in degrees): "))
    except ValueError:
        sys.exit("Invalid input.")

    # Define sensor pairs for validation.
    # Each pair is specified as (sensor_index, sensor_index)
    # Using indices: 0 = Front, 1 = Right, 2 = Back, 3 = Left.
    sensor_pairs = [
        (0, 3),  # Front + Left
        (0, 1),  # Front + Right
        (2, 3),  # Back + Left
        (2, 1)   # Back + Right
    ]

    pair_names = {
        (0, 3): "Front+Left",
        (0, 1): "Front+Right",
        (2, 3): "Back+Left",
        (2, 1): "Back+Right"
    }

    # Define a dynamic tolerance (e.g. 5% of each measured reading)
    best_combo_error = float('inf')
    best_pair = None
    for pair in sensor_pairs:
        s1, s2 = pair
        # Retrieve the measured distances for each sensor
        m1 = measured_readings[sensors[s1]["name"]]
        m2 = measured_readings[sensors[s2]["name"]]
        error = compute_pair_error(rough_pos, heading_deg, s1, s2, m1, m2)
        # Here we check whether each sensor's error is within 5% of the measured reading.
        pred1 = predict_sensor_reading(rough_pos, heading_deg, s1)
        pred2 = predict_sensor_reading(rough_pos, heading_deg, s2)
        tol1 = 0.05 * m1
        tol2 = 0.05 * m2
        error1 = abs(pred1 - m1)
        error2 = abs(pred2 - m2)
        print(f"Sensor pair {pair_names[pair]} has errors: {error1:.3f} and {error2:.3f} (Total: {error:.3f})")
        if error1 < tol1 and error2 < tol2 and error < best_combo_error:
            best_combo_error = error
            best_pair = pair

    if best_pair is None:
        print("No sensor pair appears to be unobstructed (within tolerance).")
        return

    print(f"Selected sensor pair: {pair_names[best_pair]}")
    s1, s2 = best_pair
    m1 = measured_readings[sensors[s1]["name"]]
    m2 = measured_readings[sensors[s2]["name"]]

    # Run inverse localization to compute the refined position.
    refined_x, refined_y, final_error = inverse_localize(heading_deg, s1, s2, m1, m2, rough_pos)

    # Report the refined coordinates if the final error is acceptably low.
    tolerance = 0.05 * ((m1 + m2) / 2)  # example: 5% of the average measured reading
    if final_error < tolerance:
        print(f"Estimated Position: x = {refined_x:.2f}, y = {refined_y:.2f} (final error = {final_error:.3f})")
    else:
        print(f"No accurate position found using the selected sensor pair (min error = {final_error:.3f}).")

if __name__ == "__main__":
    main()
