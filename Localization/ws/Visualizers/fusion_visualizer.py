import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# Room dimensions (meters)
ROOM_WIDTH = 8.0
ROOM_LENGTH = 15.0

# Sensor definitions (relative to robot frame)
sensors = [
    {"name": "Front", "direction_offset": 90,  "dx": -0.27995,  "dy":  0.18097},
    {"name": "Right", "direction_offset": 0,   "dx":  0.30218,  "dy":  0.13796},
    {"name": "Back",  "direction_offset": 270, "dx":  0.11581,  "dy": -0.33108},
    {"name": "Left",  "direction_offset": 180, "dx": -0.32057,  "dy":  0.09104}
]

def deg2rad(a):
    return a * math.pi / 180.0

def find_wall_hit(pos, angle_deg, room_w=ROOM_WIDTH, room_l=ROOM_LENGTH):
    theta = deg2rad(angle_deg)
    dx = math.cos(theta)
    dy = math.sin(theta)
    t_min = float('inf')
    
    # Vertical walls:
    if dx != 0:
        # Left wall (x = 0)
        t = (0 - pos["x"]) / dx
        y_hit = pos["y"] + t * dy
        if t > 0 and 0 <= y_hit <= room_l and t < t_min:
            t_min = t
        # Right wall (x = room_w)
        t = (room_w - pos["x"]) / dx
        y_hit = pos["y"] + t * dy
        if t > 0 and 0 <= y_hit <= room_l and t < t_min:
            t_min = t

    # Horizontal walls:
    if dy != 0:
        # Bottom wall (y = 0)
        t = (0 - pos["y"]) / dy
        x_hit = pos["x"] + t * dx
        if t > 0 and 0 <= x_hit <= room_w and t < t_min:
            t_min = t
        # Top wall (y = room_l)
        t = (room_l - pos["y"]) / dy
        x_hit = pos["x"] + t * dx
        if t > 0 and 0 <= x_hit <= room_w and t < t_min:
            t_min = t

    return t_min

def predict_sensor_reading(pos, heading_deg, sensor_index):
    sensor = sensors[sensor_index]
    angle_rad = deg2rad(heading_deg)
    sensor_world = {
        "x": pos["x"] + sensor["dx"] * math.cos(angle_rad) - sensor["dy"] * math.sin(angle_rad),
        "y": pos["y"] + sensor["dx"] * math.sin(angle_rad) + sensor["dy"] * math.cos(angle_rad)
    }
    beam_angle = heading_deg + sensor["direction_offset"]
    distance = find_wall_hit(sensor_world, beam_angle)
    return distance, sensor_world, beam_angle

def compute_pair_error(pos, heading_deg, sensor1_index, sensor2_index, measured1, measured2):
    pred1, _, _ = predict_sensor_reading(pos, heading_deg, sensor1_index)
    pred2, _, _ = predict_sensor_reading(pos, heading_deg, sensor2_index)
    error = abs(pred1 - measured1) + abs(pred2 - measured2)
    return error

def inverse_localize(heading_deg, sensor1_index, sensor2_index, measured1, measured2, rough_pos):
    # Search in a window ±0.5 m around the rough estimate
    search_margin = 0.5
    x_min = max(0, rough_pos["x"] - search_margin)
    x_max = min(ROOM_WIDTH, rough_pos["x"] + search_margin)
    y_min = max(0, rough_pos["y"] - search_margin)
    y_max = min(ROOM_LENGTH, rough_pos["y"] + search_margin)
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

    # Fine search around best coarse match
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

# Set up the interactive plot and sliders
fig, ax = plt.subplots(figsize=(12, 10))
plt.subplots_adjust(left=0.25, bottom=0.35, right=0.95, top=0.9)

# Draw room boundaries
room_rect = plt.Rectangle((0, 0), ROOM_WIDTH, ROOM_LENGTH, fill=False, color='black', linewidth=2)
ax.add_patch(room_rect)
ax.set_xlim(-1, ROOM_WIDTH + 1)
ax.set_ylim(-1, ROOM_LENGTH + 1)
ax.set_aspect('equal')
ax.set_title("Robot Localization Visualizer", fontsize=14)

# Initial parameters for sliders:
init_sensor_front = 6.0
init_sensor_right = 4.0
init_sensor_back = 8.0
init_sensor_left = 5.0
init_rough_x = 4.0
init_rough_y = 7.5
init_heading = 45.0

# Create text panel at the top of the figure, outside of the main axes
# For positioning: use figure coordinates (0,0 is bottom-left, 1,1 is top-right)
panel_top = 0.98
panel_spacing = 0.04
panel_left_col = 0.1  # Left column start
panel_center_col = 0.4  # Center column start
panel_right_col = 0.7  # Right column start

# Create a box background for the information panel
panel_bg = plt.Rectangle((0.05, panel_top-0.22), 0.9, 0.2, 
                         transform=fig.transFigure, facecolor='lightyellow', 
                         alpha=0.3, zorder=1)
fig.patches.append(panel_bg)

# Display texts for sensor pair and refined result
text_title = fig.text(0.5, panel_top, "Robot Localization Information", 
                      fontsize=12, ha='center', fontweight='bold')

# Left column - Refined position info
panel_top_offset = 0.04
text_sensor_pair = fig.text(panel_left_col, panel_top-panel_top_offset, 
                           "", fontsize=11, color="blue")
text_refined = fig.text(panel_left_col, panel_top-panel_top_offset-panel_spacing, 
                       "", fontsize=11, color="red")

# Center column - Expected readings
expected_readings_title = fig.text(panel_center_col, panel_top-panel_top_offset, 
                                  "Expected Readings:", fontsize=11, fontweight='bold')
expected_front = fig.text(panel_center_col, panel_top-panel_top_offset-panel_spacing*1, 
                         "", fontsize=11)
expected_right = fig.text(panel_center_col, panel_top-panel_top_offset-panel_spacing*2, 
                         "", fontsize=11)
expected_back = fig.text(panel_center_col, panel_top-panel_top_offset-panel_spacing*3, 
                        "", fontsize=11)
expected_left = fig.text(panel_center_col, panel_top-panel_top_offset-panel_spacing*4, 
                        "", fontsize=11)

# Right column - Differences
diff_title = fig.text(panel_right_col, panel_top-panel_top_offset, 
                     "Difference (Measured-Expected):", fontsize=11, fontweight='bold')
diff_front = fig.text(panel_right_col, panel_top-panel_top_offset-panel_spacing*1, 
                     "", fontsize=11)
diff_right = fig.text(panel_right_col, panel_top-panel_top_offset-panel_spacing*2, 
                     "", fontsize=11)
diff_back = fig.text(panel_right_col, panel_top-panel_top_offset-panel_spacing*3, 
                    "", fontsize=11)
diff_left = fig.text(panel_right_col, panel_top-panel_top_offset-panel_spacing*4, 
                    "", fontsize=11)

# Markers for rough and refined positions (pass as sequences)
rough_marker, = ax.plot([init_rough_x], [init_rough_y], 'bo', markersize=8, label="Rough Position")
refined_marker, = ax.plot([], [], 'ro', markersize=8, label="Refined Position")

# Lines to represent sensor rays (one line per sensor from the sensor's location to wall hit)
sensor_lines = {}
for sensor in sensors:
    line, = ax.plot([], [], label=f"{sensor['name']} ray", linestyle='--')
    sensor_lines[sensor["name"]] = line

# Create slider axes
slider_ax_color = 'lightgoldenrodyellow'
ax_sensor_front = plt.axes([0.25, 0.25, 0.65, 0.03], facecolor=slider_ax_color)
ax_sensor_right = plt.axes([0.25, 0.21, 0.65, 0.03], facecolor=slider_ax_color)
ax_sensor_back = plt.axes([0.25, 0.17, 0.65, 0.03], facecolor=slider_ax_color)
ax_sensor_left = plt.axes([0.25, 0.13, 0.65, 0.03], facecolor=slider_ax_color)
ax_rough_x = plt.axes([0.25, 0.09, 0.65, 0.03], facecolor=slider_ax_color)
ax_rough_y = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=slider_ax_color)
ax_heading = plt.axes([0.25, 0.01, 0.65, 0.03], facecolor=slider_ax_color)

slider_sensor_front = Slider(ax_sensor_front, 'Front', 0.5, 15.0, valinit=init_sensor_front)
slider_sensor_right = Slider(ax_sensor_right, 'Right', 0.5, 15.0, valinit=init_sensor_right)
slider_sensor_back = Slider(ax_sensor_back, 'Back', 0.5, 15.0, valinit=init_sensor_back)
slider_sensor_left = Slider(ax_sensor_left, 'Left', 0.5, 15.0, valinit=init_sensor_left)
slider_rough_x = Slider(ax_rough_x, 'Rough X', 0, ROOM_WIDTH, valinit=init_rough_x)
slider_rough_y = Slider(ax_rough_y, 'Rough Y', 0, ROOM_LENGTH, valinit=init_rough_y)
slider_heading = Slider(ax_heading, 'Heading', 0, 360, valinit=init_heading)

# Define candidate sensor pairs (by index): (Front+Left, Front+Right, Back+Left, Back+Right)
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

def update(val):
    # Get slider values
    measured_front = slider_sensor_front.val
    measured_right = slider_sensor_right.val
    measured_back = slider_sensor_back.val
    measured_left = slider_sensor_left.val
    rough_x = slider_rough_x.val
    rough_y = slider_rough_y.val
    heading_deg = slider_heading.val

    measured_readings = {
        "Front": measured_front,
        "Right": measured_right,
        "Back": measured_back,
        "Left": measured_left
    }
    rough_pos = {"x": rough_x, "y": rough_y}

    # Update rough position marker (pass values as lists)
    rough_marker.set_data([rough_x], [rough_y])
    
    # Calculate expected sensor readings based on rough position
    expected_readings = {}
    
    # For each sensor compute predicted sensor reading from the rough position (and update sensor ray lines)
    for idx, sensor in enumerate(sensors):
        pred, sensor_world, beam_angle = predict_sensor_reading(rough_pos, heading_deg, idx)
        expected_readings[sensor["name"]] = pred
        
        # Compute line end: from sensor_world, line extending to wall (for visualization)
        theta = deg2rad(beam_angle)
        dx = math.cos(theta) * pred
        dy = math.sin(theta) * pred
        sensor_lines[sensor["name"]].set_data([sensor_world["x"], sensor_world["x"] + dx],
                                             [sensor_world["y"], sensor_world["y"] + dy])
    
    # Update expected readings text
    expected_front.set_text(f"Front: {expected_readings['Front']:.2f} m")
    expected_right.set_text(f"Right: {expected_readings['Right']:.2f} m")
    expected_back.set_text(f"Back: {expected_readings['Back']:.2f} m")
    expected_left.set_text(f"Left: {expected_readings['Left']:.2f} m")
    
    # Calculate and display differences
    diff_front.set_text(f"Front: {measured_front - expected_readings['Front']:.2f} m")
    diff_right.set_text(f"Right: {measured_right - expected_readings['Right']:.2f} m")
    diff_back.set_text(f"Back: {measured_back - expected_readings['Back']:.2f} m")
    diff_left.set_text(f"Left: {measured_left - expected_readings['Left']:.2f} m")
    
    # Set colors based on difference magnitude
    for sensor_name, measured_val in measured_readings.items():
        expected_val = expected_readings[sensor_name]
        diff = abs(measured_val - expected_val)
        
        # Color coding: green if difference is small, yellow if moderate, red if large
        if diff < 0.5:
            color = 'green'
        elif diff < 1.0:
            color = 'darkorange'
        else:
            color = 'red'
            
        # Set text colors
        if sensor_name == "Front":
            diff_front.set_color(color)
        elif sensor_name == "Right":
            diff_right.set_color(color)
        elif sensor_name == "Back":
            diff_back.set_color(color)
        elif sensor_name == "Left":
            diff_left.set_color(color)

    # Determine the best sensor pair
    best_combo_error = float('inf')
    best_pair = None
    for pair in sensor_pairs:
        s1, s2 = pair
        m1 = measured_readings[sensors[s1]["name"]]
        m2 = measured_readings[sensors[s2]["name"]]
        error = compute_pair_error(rough_pos, heading_deg, s1, s2, m1, m2)
        # Check tolerance: here tolerance = 5% of measured reading
        tol1 = 0.05 * m1
        tol2 = 0.05 * m2
        pred1, _, _ = predict_sensor_reading(rough_pos, heading_deg, s1)
        pred2, _, _ = predict_sensor_reading(rough_pos, heading_deg, s2)
        error1 = abs(pred1 - m1)
        error2 = abs(pred2 - m2)
        if error1 < tol1 and error2 < tol2 and error < best_combo_error:
            best_combo_error = error
            best_pair = pair

    if best_pair is None:
        text_sensor_pair.set_text("No valid sensor pair found")
        refined_marker.set_data([], [])
        text_refined.set_text("")
    else:
        text_sensor_pair.set_text(f"Selected sensor pair: {pair_names[best_pair]}")
        s1, s2 = best_pair
        m1 = measured_readings[sensors[s1]["name"]]
        m2 = measured_readings[sensors[s2]["name"]]
        # Run inverse localization to compute the refined position.
        refined_x, refined_y, final_error = inverse_localize(heading_deg, s1, s2, m1, m2, rough_pos)
        # Update refined marker if error is acceptable (here tolerance based on average reading)
        tolerance = 0.05 * ((m1 + m2) / 2)
        if final_error < tolerance:
            refined_marker.set_data([refined_x], [refined_y])
            text_refined.set_text(f"Refined Position: ({refined_x:.2f}, {refined_y:.2f}) (err={final_error:.3f})")
        else:
            refined_marker.set_data([], [])
            text_refined.set_text("No accurate refined position found")
    
    fig.canvas.draw_idle()

# Set slider update callbacks
slider_sensor_front.on_changed(update)
slider_sensor_right.on_changed(update)
slider_sensor_back.on_changed(update)
slider_sensor_left.on_changed(update)
slider_rough_x.on_changed(update)
slider_rough_y.on_changed(update)
slider_heading.on_changed(update)

# Initial update call
update(None)

# Reset button to restore defaults
resetax = plt.axes([0.025, 0.9, 0.1, 0.04])
button = Button(resetax, 'Reset', color=slider_ax_color, hovercolor='0.975')
def reset(event):
    slider_sensor_front.reset()
    slider_sensor_right.reset()
    slider_sensor_back.reset()
    slider_sensor_left.reset()
    slider_rough_x.reset()
    slider_rough_y.reset()
    slider_heading.reset()
button.on_clicked(reset)

ax.legend(loc="upper right")
plt.show()