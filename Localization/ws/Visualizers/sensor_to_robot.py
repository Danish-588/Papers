import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button

# Sensor frame definitions (x, y, theta offset)
sensor_tf = np.array([
    [-0.26059, 0.19435, 90.0],    # F
    [ 0.30215, 0.13147, 0.0 ],    # R
    [ 0.13674, -0.31880, 270.0],  # B
    [-0.32057, 0.12588, 180.0]    # L
])
sensor_labels = ['F', 'R', 'B', 'L']

# Fast trig functions
def faster_cos(deg): return np.cos(np.radians(deg))
def faster_sin(deg): return np.sin(np.radians(deg))

# Robot to sensor transform
def compute_sensor_positions(x, y, theta):
    result = []
    for tf in sensor_tf:
        sx = x + (tf[0] * faster_cos(theta)) - (tf[1] * faster_sin(theta))
        sy = y + (tf[0] * faster_sin(theta)) + (tf[1] * faster_cos(theta))
        stheta = theta + tf[2]
        result.append((sx, sy, stheta))
    return np.array(result)

# Sensor to robot inverse transform
def compute_robot_positions_from_sensors(sensor_positions, theta):
    result = []
    for i, tf in enumerate(sensor_tf):
        x = sensor_positions[i][0] - (tf[0] * faster_cos(theta)) + (tf[1] * faster_sin(theta))
        y = sensor_positions[i][1] - (tf[0] * faster_sin(theta)) - (tf[1] * faster_cos(theta))
        result.append((x, y, theta))
    return np.array(result)

# Initial robot pose
init_pose = [0.0, 0.0, 0.0]
refined_sensor_positions = compute_sensor_positions(*init_pose)

# Matplotlib setup
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.35)
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_aspect('equal')
ax.set_title("Robot ↔ Sensor Transform Visualizer")
ax.grid(True)

# Plot elements
robot_dot, = ax.plot([], [], 'bo', label='Robot Pose')
sensor_dots, = ax.plot([], [], 'ro', label='Sensor Poses')
refined_dots, = ax.plot([], [], 'go', label='Back-Calculated Robot Pose (from sensors)')
text_labels = [ax.text(0, 0, '', fontsize=12, ha='center', va='bottom') for _ in range(4)]
ax.legend()

# Sliders for pose
ax_x = plt.axes([0.15, 0.22, 0.65, 0.03])
ax_y = plt.axes([0.15, 0.18, 0.65, 0.03])
ax_theta = plt.axes([0.15, 0.14, 0.65, 0.03])
slider_x = Slider(ax_x, 'X', -1.0, 1.0, valinit=init_pose[0])
slider_y = Slider(ax_y, 'Y', -1.0, 1.0, valinit=init_pose[1])
slider_theta = Slider(ax_theta, 'Theta', 0.0, 360.0, valinit=init_pose[2])

# Button to toggle view
ax_button = plt.axes([0.4, 0.05, 0.2, 0.05])
toggle_button = Button(ax_button, 'Toggle View')

show_inverse = False

def update(val):
    global refined_sensor_positions, show_inverse
    x = slider_x.val
    y = slider_y.val
    theta = slider_theta.val

    # Forward transform: robot → sensors
    refined_sensor_positions = compute_sensor_positions(x, y, theta)
    sensor_dots.set_data(refined_sensor_positions[:, 0], refined_sensor_positions[:, 1])
    robot_dot.set_data([x], [y])

    # Labels: show letter + x, y
    for i, txt in enumerate(text_labels):
        sx, sy = refined_sensor_positions[i, 0], refined_sensor_positions[i, 1]
        txt.set_position((sx, sy))
        txt.set_text(f'{sensor_labels[i]} ({sx:.2f}, {sy:.2f})')

    # Inverse transform: sensors → robot
    if show_inverse:
        refined_robot_positions = compute_robot_positions_from_sensors(refined_sensor_positions, theta)
        refined_dots.set_data(refined_robot_positions[:, 0], refined_robot_positions[:, 1])
    else:
        refined_dots.set_data([], [])

    fig.canvas.draw_idle()


def toggle(event):
    global show_inverse
    show_inverse = not show_inverse
    update(None)

toggle_button.on_clicked(toggle)
slider_x.on_changed(update)
slider_y.on_changed(update)
slider_theta.on_changed(update)

# Initial plot
update(None)
plt.show()
