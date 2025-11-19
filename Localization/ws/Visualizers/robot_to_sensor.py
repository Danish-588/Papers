import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# Define sensor transform and labels
sensor_tf = np.array([
    [-0.27995, 0.18097, 90.0],    # Front
    [ 0.30218, 0.13796, 0.0 ],    # Right
    [ 0.11581, -0.33108, 270.0],  # Back
    [-0.32057, 0.09104, 180.0]    # Left
])
sensor_labels = ['F', 'R', 'B', 'L']

def faster_cos(deg): return np.cos(np.radians(deg))
def faster_sin(deg): return np.sin(np.radians(deg))

def compute_sensor_positions(x, y, theta):
    result = []
    for tf in sensor_tf:
        sx = x + (tf[0] * faster_cos(theta)) - (tf[1] * faster_sin(theta))
        sy = y + (tf[0] * faster_sin(theta)) + (tf[1] * faster_cos(theta))
        stheta = theta + tf[2]
        result.append((sx, sy, stheta))
    return np.array(result)

# Plotting setup
fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.25)
ax.set_xlim(0, 8)
ax.set_ylim(0, 15)
ax.set_aspect('equal')
ax.grid(True)
ax.set_title("Robot Pose and Sensor Positions")

# Plot elements
robot_dot, = ax.plot([], [], 'bo', label='Robot')
sensor_dots, = ax.plot([], [], 'ro', label='Sensors')
text_labels = [ax.text(0, 0, '', fontsize=12, ha='center', va='bottom') for _ in range(4)]
ax.legend()

# Sliders
ax_x = plt.axes([0.15, 0.1, 0.65, 0.03])
ax_y = plt.axes([0.15, 0.06, 0.65, 0.03])
ax_theta = plt.axes([0.15, 0.02, 0.65, 0.03])

slider_x = Slider(ax_x, 'X', 0.0, 8.0, valinit=0.0)
slider_y = Slider(ax_y, 'Y', 0.0, 15.0, valinit=0.0)
slider_theta = Slider(ax_theta, 'Theta', 0.0, 360.0, valinit=0.0)

def update(val):
    x = slider_x.val
    y = slider_y.val
    theta = slider_theta.val
    sensors = compute_sensor_positions(x, y, theta)

    sensor_dots.set_data(sensors[:, 0], sensors[:, 1])
    robot_dot.set_data([x], [y])
    
    for i, txt in enumerate(text_labels):
        txt.set_position((sensors[i, 0], sensors[i, 1]))
        txt.set_text(sensor_labels[i])

    fig.canvas.draw_idle()

slider_x.on_changed(update)
slider_y.on_changed(update)
slider_theta.on_changed(update)

# Initial call
update(None)
plt.show()
