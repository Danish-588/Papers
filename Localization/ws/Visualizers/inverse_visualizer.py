import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, RadioButtons

room_width = 8.0
room_length = 15.0

# Sensor direction offsets (in degrees relative to the robot's heading)
laser_dirs = {
    "Front": 90,
    "Right": 0,
    "Back": 270,
    "Left": 180
}

# Sensor positions relative to the robot center (in meters)
sensor_offsets = {
    "Front": (0.0, 0.2),
    "Right": (0.2, 0.0),
    "Back": (0.0, -0.2),
    "Left": (-0.2, 0.0)
}

def deg2rad(angle):
    return angle * np.pi / 180.0

def find_wall_hit(room_w, room_l, pos, angle_deg):
    theta = deg2rad(angle_deg)
    dx = np.cos(theta)
    dy = np.sin(theta)

    t_min = float('inf')
    hit = [None, None]

    if dx != 0:
        for x_wall in [0, room_w]:
            t = (x_wall - pos[0]) / dx
            y_hit = pos[1] + t * dy
            if t > 0 and 0 <= y_hit <= room_l and t < t_min:
                t_min = t
                hit = [x_wall, y_hit]

    if dy != 0:
        for y_wall in [0, room_l]:
            t = (y_wall - pos[1]) / dy
            x_hit = pos[0] + t * dx
            if t > 0 and 0 <= x_hit <= room_w and t < t_min:
                t_min = t
                hit = [x_hit, y_wall]

    return hit, t_min

# Initial parameters
init_d_front = 6.0
init_d_right = 9.0
init_d_back = 4.0
init_d_left = 2.5
init_theta = 100.0

fig, ax = plt.subplots()
plt.subplots_adjust(left=0.25, bottom=0.45)
text_annotation = ax.text(0, room_length + 0.5, '', fontsize=10)
ax.set_xlim(0, room_width)
ax.set_ylim(0, room_length)
ax.set_aspect('equal')
ax.grid(True)

# Sliders
ax_front = plt.axes([0.25, 0.35, 0.65, 0.03])
ax_right = plt.axes([0.25, 0.30, 0.65, 0.03])
ax_back  = plt.axes([0.25, 0.25, 0.65, 0.03])
ax_left  = plt.axes([0.25, 0.20, 0.65, 0.03])
ax_theta = plt.axes([0.25, 0.10, 0.65, 0.03])

slider_front = Slider(ax_front, 'Front dist', 0.5, 15.0, valinit=init_d_front)
slider_right = Slider(ax_right, 'Right dist', 0.5, 15.0, valinit=init_d_right)
slider_back  = Slider(ax_back,  'Back dist',  0.5, 15.0, valinit=init_d_back)
slider_left  = Slider(ax_left,  'Left dist',  0.5, 15.0, valinit=init_d_left)
slider_theta = Slider(ax_theta, 'Theta (°)', 0, 360, valinit=init_theta)

# Radio buttons
ax_radio1 = plt.axes([0.025, 0.65, 0.2, 0.15])
ax_radio2 = plt.axes([0.025, 0.45, 0.2, 0.15])
sensors = ["Front", "Right", "Back", "Left"]
radio1 = RadioButtons(ax_radio1, sensors, active=0)
radio2 = RadioButtons(ax_radio2, sensors, active=1)

def update(val):
    selected1 = radio1.value_selected
    selected2 = radio2.value_selected
    theta = slider_theta.val

    sensor_values = {
        "Front": slider_front.val,
        "Right": slider_right.val,
        "Back": slider_back.val,
        "Left": slider_left.val
    }
    d1 = sensor_values[selected1]
    d2 = sensor_values[selected2]

    ax.cla()
    ax.set_xlim(0, room_width)
    ax.set_ylim(0, room_length)
    ax.grid(True)
    ax.set_aspect('equal')

    # Re-draw room
    ax.plot([0, room_width, room_width, 0, 0],
            [0, 0, room_length, room_length, 0], 'k-')

    tolerance = 0.1
    best_pose = None

    for x in np.linspace(0.5, room_width - 0.5, 200):
        for y in np.linspace(0.5, room_length - 0.5, 200):
            match = True
            for sensor_name, expected in [(selected1, d1), (selected2, d2)]:
                angle = theta + laser_dirs[sensor_name]
                dx_r, dy_r = sensor_offsets[sensor_name]
                theta_rad = deg2rad(theta)
                dx_w = dx_r * np.cos(theta_rad) - dy_r * np.sin(theta_rad)
                dy_w = dx_r * np.sin(theta_rad) + dy_r * np.cos(theta_rad)
                sensor_pos = (x + dx_w, y + dy_w)

                _, measured = find_wall_hit(room_width, room_length, sensor_pos, angle)
                if abs(measured - expected) > tolerance:
                    match = False
                    break
            if match:
                best_pose = (x, y)
                break
        if best_pose:
            break

    if best_pose:
        x, y = best_pose
        ax.scatter(x, y, color='red', s=50)
        ax.add_patch(plt.Circle((x, y), 0.1, color='red', fill=False))
        ax.text(0.5, room_length + 0.3, f"Estimated Pose: x={x:.2f}, y={y:.2f}, θ={theta:.1f}°", fontsize=10)

        for sensor_name in [selected1, selected2]:
            angle = theta + laser_dirs[sensor_name]
            dx_r, dy_r = sensor_offsets[sensor_name]
            theta_rad = deg2rad(theta)
            dx_w = dx_r * np.cos(theta_rad) - dy_r * np.sin(theta_rad)
            dy_w = dx_r * np.sin(theta_rad) + dy_r * np.cos(theta_rad)
            sensor_pos = (x + dx_w, y + dy_w)

            hit_point, _ = find_wall_hit(room_width, room_length, sensor_pos, angle)
            if hit_point:
                ax.plot([sensor_pos[0], hit_point[0]], [sensor_pos[1], hit_point[1]], 'b--')
                ax.plot(sensor_pos[0], sensor_pos[1], 'go')  # Sensor location

    fig.canvas.draw_idle()

# Connect sliders and buttons to update
slider_front.on_changed(update)
slider_right.on_changed(update)
slider_back.on_changed(update)
slider_left.on_changed(update)
slider_theta.on_changed(update)
radio1.on_clicked(update)
radio2.on_clicked(update)

update(None)  # Initial draw
plt.show()