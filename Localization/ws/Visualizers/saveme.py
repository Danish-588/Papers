import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.widgets import Slider
import math

class RobotLocalizer:
    def __init__(self):
        # Field dimensions
        self.WIDTH = 8.0
        self.LENGTH = 15.0
        
        # Margins
        self.BORDER_MARGIN = 0.4
        self.CORNER_MARGIN = 0.8
        self.BLOCKED_MARGIN = 0.1
        self.FIELD_MARGIN = -0.1
        self.FINAL_MARGIN = 0.08
        
        # Sensor transforms (relative to robot center)
        self.sensor_tf = [
            {'x': -0.27995, 'y': 0.18097, 'theta': 90.0},   # FRONT
            {'x': 0.30218,  'y': 0.13796, 'theta': 0.0},    # RIGHT
            {'x': 0.11581,  'y': -0.33108, 'theta': 270.0}, # BACK
            {'x': -0.32057, 'y': 0.09104, 'theta': 180.0},  # LEFT
        ]
        
        # Sensor calibration parameters
        self.sensor_params = [
            {'offset': 13,  'scale': 2223.1274, 'base': 0.0402},  # front
            {'offset': 37,  'scale': 2216.1616, 'base': 0.0432},  # right
            {'offset': 325, 'scale': 2228.7458, 'base': 0.0903},  # back
            {'offset': 27,  'scale': 2220.7327, 'base': 0.046},   # left
        ]
        
        # Real robot state (ground truth)
        self.real_x = 4.0
        self.real_y = 7.0
        self.real_theta = 0.0
        
        # Predicted robot state
        self.xy_x = 4.0
        self.xy_y = 7.0
        self.angle = 0.0
        
        # Offset for localization
        self.offset = {'x': 0.0, 'y': 0.0}
        
        # Combined pose (predicted + offset)
        self.combined_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        
        # Sensor data
        self.expected_sensor = [{'x': 0, 'y': 0, 'theta': 0} for _ in range(4)]
        self.refined_sensor = [{'x': 0, 'y': 0, 'theta': 0} for _ in range(4)]
        self.refined = [{'x': 0, 'y': 0, 'theta': 0} for _ in range(4)]
        self.sensor_hits = [{'x': 0, 'y': 0, 'wall': 0} for _ in range(4)]
        
        self.expected = [0.0] * 4
        self.measured = [0.0] * 4
        self.diff = [0.0] * 4
        self.sensor_status = ['BLOCKED'] * 4
        
        # Sensor derived pose
        self.sensor_derived_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.dead_x = False
        self.dead_y = False
        
        # Direction constants
        self.FRONT, self.RIGHT, self.BACK, self.LEFT = 0, 1, 2, 3
        
    def faster_cos(self, angle_deg):
        return math.cos(math.radians(angle_deg))
    
    def faster_sin(self, angle_deg):
        return math.sin(math.radians(angle_deg))
    
    def simulate_sensor_readings(self):
        """Simulate sensor readings based on real robot position"""
        for i in range(4):
            # Calculate real sensor position
            real_sensor_x = self.real_x + (self.sensor_tf[i]['x'] * self.faster_cos(self.real_theta)) - (self.sensor_tf[i]['y'] * self.faster_sin(self.real_theta))
            real_sensor_y = self.real_y + (self.sensor_tf[i]['x'] * self.faster_sin(self.real_theta)) + (self.sensor_tf[i]['y'] * self.faster_cos(self.real_theta))
            real_sensor_theta = self.real_theta + self.sensor_tf[i]['theta']
            
            # Calculate distance to nearest wall
            dx = self.faster_cos(real_sensor_theta)
            dy = self.faster_sin(real_sensor_theta)
            
            t_min = 100.0
            
            # Check vertical walls
            if abs(dx) > 1e-9:
                # Left wall (x = 0)
                t = (0.0 - real_sensor_x) / dx
                y_hit = real_sensor_y + t * dy
                if t > 0.0 and 0.0 <= y_hit <= self.LENGTH and t < t_min:
                    t_min = t
                
                # Right wall (x = WIDTH)
                t = (self.WIDTH - real_sensor_x) / dx
                y_hit = real_sensor_y + t * dy
                if t > 0.0 and 0.0 <= y_hit <= self.LENGTH and t < t_min:
                    t_min = t
            
            # Check horizontal walls
            if abs(dy) > 1e-9:
                # Bottom wall (y = 0)
                t = (0.0 - real_sensor_y) / dy
                x_hit = real_sensor_x + t * dx
                if t > 0.0 and 0.0 <= x_hit <= self.WIDTH and t < t_min:
                    t_min = t
                
                # Top wall (y = LENGTH)
                t = (self.LENGTH - real_sensor_y) / dy
                x_hit = real_sensor_x + t * dx
                if t > 0.0 and 0.0 <= x_hit <= self.WIDTH and t < t_min:
                    t_min = t
            
            # Add some noise to simulate real sensor
            noise = np.random.normal(0, 0.02)
            self.measured[i] = max(0.01, t_min + noise)
    
    def robot_to_sensor(self):
        for i in range(4):
            self.expected_sensor[i]['x'] = self.combined_pose['x'] + (self.sensor_tf[i]['x'] * self.faster_cos(self.angle)) - (self.sensor_tf[i]['y'] * self.faster_sin(self.angle))
            self.expected_sensor[i]['y'] = self.combined_pose['y'] + (self.sensor_tf[i]['x'] * self.faster_sin(self.angle)) + (self.sensor_tf[i]['y'] * self.faster_cos(self.angle))
            self.expected_sensor[i]['theta'] = self.combined_pose['theta'] + self.sensor_tf[i]['theta']
    
    def predict_wall_hit(self):
        for i in range(4):
            theta = self.expected_sensor[i]['theta']
            dx = self.faster_cos(theta)
            dy = self.faster_sin(theta)
            
            t_min = 100.0
            
            # Vertical walls
            if abs(dx) > 1e-9:
                # Left wall (x = 0)
                t = (0.0 - self.expected_sensor[i]['x']) / dx
                y_hit = self.expected_sensor[i]['y'] + t * dy
                if t > 0.0 and 0.0 <= y_hit <= self.LENGTH and t < t_min:
                    t_min = t
                    self.sensor_hits[i] = {'x': 0.0, 'y': y_hit, 'wall': self.LEFT}
                
                # Right wall (x = WIDTH)
                t = (self.WIDTH - self.expected_sensor[i]['x']) / dx
                y_hit = self.expected_sensor[i]['y'] + t * dy
                if t > 0.0 and 0.0 <= y_hit <= self.LENGTH and t < t_min:
                    t_min = t
                    self.sensor_hits[i] = {'x': self.WIDTH, 'y': y_hit, 'wall': self.RIGHT}
            
            # Horizontal walls
            if abs(dy) > 1e-9:
                # Bottom wall (y = 0)
                t = (0.0 - self.expected_sensor[i]['y']) / dy
                x_hit = self.expected_sensor[i]['x'] + t * dx
                if t > 0.0 and 0.0 <= x_hit <= self.WIDTH and t < t_min:
                    t_min = t
                    self.sensor_hits[i] = {'x': x_hit, 'y': 0.0, 'wall': self.BACK}
                
                # Top wall (y = LENGTH)
                t = (self.LENGTH - self.expected_sensor[i]['y']) / dy
                x_hit = self.expected_sensor[i]['x'] + t * dx
                if t > 0.0 and 0.0 <= x_hit <= self.WIDTH and t < t_min:
                    t_min = t
                    self.sensor_hits[i] = {'x': x_hit, 'y': self.LENGTH, 'wall': self.FRONT}
            
            self.expected[i] = t_min
            self.diff[i] = self.expected[i] - self.measured[i]
    
    def is_sensor_valid(self):
        for i in range(4):
            # Check for BORDER
            if (self.combined_pose['x'] < (0 + self.BORDER_MARGIN) or 
                self.combined_pose['x'] > self.WIDTH - self.BORDER_MARGIN or
                self.combined_pose['y'] < (0 + self.BORDER_MARGIN) or 
                self.combined_pose['y'] > self.LENGTH - self.BORDER_MARGIN):
                self.sensor_status[i] = 'BORDER'
            # Check if near MAX VALUE
            elif self.measured[i] > 10.0:
                self.sensor_status[i] = 'MAXVAL'
            # Check for CORNER
            elif ((self.sensor_hits[i]['x'] < self.CORNER_MARGIN or 
                   self.sensor_hits[i]['x'] > self.WIDTH - self.CORNER_MARGIN) and
                  (self.sensor_hits[i]['y'] < self.CORNER_MARGIN or 
                   self.sensor_hits[i]['y'] > self.LENGTH - self.CORNER_MARGIN)):
                self.sensor_status[i] = 'CORNER'
            # Check for BLOCKED
            elif self.diff[i] > self.BLOCKED_MARGIN:
                self.sensor_status[i] = 'BLOCKED'
            # Check if OUTSIDE the field
            elif self.diff[i] < self.FIELD_MARGIN:
                self.sensor_status[i] = 'OUTSIDE'
            # If none of the above, set to VALID
            else:
                self.sensor_status[i] = 'VALID'
    
    def calculate_sensor_pose(self):
        for i in range(4):
            theta = self.expected_sensor[i]['theta']
            wall = self.sensor_hits[i]['wall']
            
            if wall == self.LEFT:
                self.refined_sensor[i]['x'] = 0.0 - self.measured[i] * self.faster_cos(theta)
                self.refined_sensor[i]['y'] = self.expected_sensor[i]['y']
            elif wall == self.RIGHT:
                self.refined_sensor[i]['x'] = self.WIDTH - self.measured[i] * self.faster_cos(theta)
                self.refined_sensor[i]['y'] = self.expected_sensor[i]['y']
            elif wall == self.BACK:
                self.refined_sensor[i]['y'] = 0.0 - self.measured[i] * self.faster_sin(theta)
                self.refined_sensor[i]['x'] = self.expected_sensor[i]['x']
            elif wall == self.FRONT:
                self.refined_sensor[i]['y'] = self.LENGTH - self.measured[i] * self.faster_sin(theta)
                self.refined_sensor[i]['x'] = self.expected_sensor[i]['x']
            
            self.refined_sensor[i]['theta'] = self.expected_sensor[i]['theta']
    
    def sensor_to_robot(self):
        for i in range(4):
            self.refined[i]['x'] = (self.refined_sensor[i]['x'] - 
                                  (self.sensor_tf[i]['x'] * self.faster_cos(self.angle)) + 
                                  (self.sensor_tf[i]['y'] * self.faster_sin(self.angle)))
            self.refined[i]['y'] = (self.refined_sensor[i]['y'] - 
                                  (self.sensor_tf[i]['x'] * self.faster_sin(self.angle)) - 
                                  (self.sensor_tf[i]['y'] * self.faster_cos(self.angle)))
            self.refined[i]['theta'] = self.combined_pose['theta']
    
    def check_opposites(self):
        self.dead_x = False
        self.dead_y = False
        
        # FRONT & BACK pair
        w1 = self.sensor_hits[self.FRONT]['wall']
        w2 = self.sensor_hits[self.BACK]['wall']
        
        # Both hit horizontal walls → derive Y
        if (w1 in [self.FRONT, self.BACK]) and (w2 in [self.FRONT, self.BACK]):
            y1 = self.refined[self.FRONT]['y']
            y2 = self.refined[self.BACK]['y']
            if abs(y1 - y2) < self.FINAL_MARGIN:
                self.sensor_derived_pose['y'] = 0.5 * (y1 + y2)
                self.dead_y = True
        # Both hit vertical walls → derive X
        elif (w1 in [self.LEFT, self.RIGHT]) and (w2 in [self.LEFT, self.RIGHT]):
            x1 = self.refined[self.FRONT]['x']
            x2 = self.refined[self.BACK]['x']
            if abs(x1 - x2) < self.FINAL_MARGIN:
                self.sensor_derived_pose['x'] = 0.5 * (x1 + x2)
                self.dead_x = True
        
        # LEFT & RIGHT pair
        w1 = self.sensor_hits[self.LEFT]['wall']
        w2 = self.sensor_hits[self.RIGHT]['wall']
        
        # Both hit vertical walls → derive X
        if (w1 in [self.LEFT, self.RIGHT]) and (w2 in [self.LEFT, self.RIGHT]):
            x1 = self.refined[self.LEFT]['x']
            x2 = self.refined[self.RIGHT]['x']
            if abs(x1 - x2) < self.FINAL_MARGIN:
                self.sensor_derived_pose['x'] = 0.5 * (x1 + x2)
                self.dead_x = True
        # Both hit horizontal walls → derive Y
        elif (w1 in [self.FRONT, self.BACK]) and (w2 in [self.FRONT, self.BACK]):
            y1 = self.refined[self.LEFT]['y']
            y2 = self.refined[self.RIGHT]['y']
            if abs(y1 - y2) < self.FINAL_MARGIN:
                self.sensor_derived_pose['y'] = 0.5 * (y1 + y2)
                self.dead_y = True
    
    def deadreck(self):
        if self.dead_x:
            self.offset['x'] = self.sensor_derived_pose['x'] - self.xy_x
        if self.dead_y:
            self.offset['y'] = self.sensor_derived_pose['y'] - self.xy_y
    
    def localize(self):
        # Update combined pose
        self.combined_pose['x'] = self.xy_x + self.offset['x']
        self.combined_pose['y'] = self.xy_y + self.offset['y']
        self.combined_pose['theta'] = self.angle
        
        # Simulate sensor readings from real position
        self.simulate_sensor_readings()
        
        # Run localization algorithm
        self.robot_to_sensor()
        self.predict_wall_hit()
        self.is_sensor_valid()
        self.calculate_sensor_pose()
        self.sensor_to_robot()
        self.check_opposites()
        self.deadreck()
    
    def draw_robot(self, ax, x, y, theta, color, label, alpha=0.7, linewidth=2):
        # Robot body (rectangle)
        robot_width = 0.3
        robot_height = 0.4
        
        # Create robot rectangle with thicker outline for better visibility
        robot_rect = patches.Rectangle((-robot_width/2, -robot_height/2), 
                                     robot_width, robot_height, 
                                     facecolor=color, alpha=alpha, 
                                     edgecolor='black', linewidth=linewidth)
        
        # Transform rectangle
        t = plt.matplotlib.transforms.Affine2D().rotate_deg(theta) + \
            plt.matplotlib.transforms.Affine2D().translate(x, y) + ax.transData
        robot_rect.set_transform(t)
        ax.add_patch(robot_rect)
        
        # Direction arrow
        dx = 0.3 * self.faster_cos(theta)
        dy = 0.3 * self.faster_sin(theta)
        ax.arrow(x, y, dx, dy, head_width=0.08, head_length=0.1, 
                fc=color, ec='black', alpha=alpha, linewidth=1)
        
        # Draw sensors only for real robot to avoid clutter
        if 'Real' in label:
            sensor_colors = ['red', 'green', 'blue', 'orange']
            sensor_names = ['F', 'R', 'B', 'L']
            
            for i, tf in enumerate(self.sensor_tf):
                sensor_x = x + (tf['x'] * self.faster_cos(theta)) - (tf['y'] * self.faster_sin(theta))
                sensor_y = y + (tf['x'] * self.faster_sin(theta)) + (tf['y'] * self.faster_cos(theta))
                ax.plot(sensor_x, sensor_y, 'o', color=sensor_colors[i], markersize=6)
                ax.text(sensor_x + 0.1, sensor_y + 0.1, sensor_names[i], 
                       fontsize=8, color=sensor_colors[i])
        
        # Label with coordinates
        coord_text = f"{label}\n({x:.2f}, {y:.2f}, {theta:.1f}°)"
        ax.text(x, y - 0.8, coord_text, ha='center', fontsize=9, color=color, 
               weight='bold', bbox=dict(boxstyle="round,pad=0.2", facecolor='white', alpha=0.8))
    
    def visualize(self):
        fig, ax = plt.subplots(figsize=(16, 10))
        
        # Create sliders
        ax_real_x = plt.axes([0.08, 0.02, 0.15, 0.03])
        ax_real_y = plt.axes([0.25, 0.02, 0.15, 0.03])
        ax_real_theta = plt.axes([0.42, 0.02, 0.15, 0.03])
        
        slider_real_x = Slider(ax_real_x, 'Real X', 0.5, self.WIDTH-0.5, valinit=self.real_x)
        slider_real_y = Slider(ax_real_y, 'Real Y', 0.5, self.LENGTH-0.5, valinit=self.real_y)
        slider_real_theta = Slider(ax_real_theta, 'Real θ', -180, 180, valinit=self.real_theta)
        
        def update_plot():
            ax.clear()
            
            # Draw field
            field_rect = patches.Rectangle((0, 0), self.WIDTH, self.LENGTH, 
                                         linewidth=2, edgecolor='black', facecolor='lightgreen', alpha=0.3)
            ax.add_patch(field_rect)
            
            # Run localization
            self.localize()
            
            # Draw real robot with thicker border
            self.draw_robot(ax, self.real_x, self.real_y, self.real_theta, 'red', 'Real Robot', alpha=0.5, linewidth=3)
            
            # Draw predicted robot with different style for better visibility
            self.draw_robot(ax, self.combined_pose['x'], self.combined_pose['y'], 
                          self.combined_pose['theta'], 'blue', 'Predicted Robot', alpha=0.8, linewidth=2)
            
            # Draw sensor rays and measurements
            sensor_colors = ['red', 'green', 'blue', 'orange']
            sensor_names = ['FRONT', 'RIGHT', 'BACK', 'LEFT']
            
            for i in range(4):
                if self.sensor_status[i] == 'VALID':
                    # Draw expected ray
                    sensor_x = self.expected_sensor[i]['x']
                    sensor_y = self.expected_sensor[i]['y']
                    hit_x = self.sensor_hits[i]['x']
                    hit_y = self.sensor_hits[i]['y']
                    
                    ax.plot([sensor_x, hit_x], [sensor_y, hit_y], 
                           '--', color=sensor_colors[i], alpha=0.5, linewidth=1)
                    
                    # Draw measured distance
                    measured_end_x = sensor_x + self.measured[i] * self.faster_cos(self.expected_sensor[i]['theta'])
                    measured_end_y = sensor_y + self.measured[i] * self.faster_sin(self.expected_sensor[i]['theta'])
                    ax.plot([sensor_x, measured_end_x], [sensor_y, measured_end_y], 
                           '-', color=sensor_colors[i], alpha=0.8, linewidth=2)
            
            # Show sensor status and coordinates
            status_text = f"=== COORDINATES ===\n"
            status_text += f"Real Robot:      ({self.real_x:.3f}, {self.real_y:.3f}, {self.real_theta:.1f}°)\n"
            status_text += f"XY Module:       ({self.xy_x:.3f}, {self.xy_y:.3f}, {self.angle:.1f}°)\n"
            status_text += f"Combined Pose:   ({self.combined_pose['x']:.3f}, {self.combined_pose['y']:.3f}, {self.combined_pose['theta']:.1f}°)\n"
            status_text += f"Sensor Derived:  ({self.sensor_derived_pose['x']:.3f}, {self.sensor_derived_pose['y']:.3f})\n"
            status_text += f"Offset:          ({self.offset['x']:.3f}, {self.offset['y']:.3f})\n\n"
            
            status_text += f"=== LOCALIZATION ERROR ===\n"
            error_x = self.real_x - self.combined_pose['x']
            error_y = self.real_y - self.combined_pose['y']
            error_total = np.sqrt(error_x**2 + error_y**2)
            status_text += f"Position Error:  ({error_x:.3f}, {error_y:.3f})\n"
            status_text += f"Total Error:     {error_total:.3f} units\n\n"
            
            status_text += f"=== SENSOR STATUS ===\n"
            for i, name in enumerate(sensor_names):
                status_text += f"{name:5}: {self.sensor_status[i]:8} | Meas: {self.measured[i]:.3f} | Exp: {self.expected[i]:.3f} | Diff: {self.diff[i]:.3f}\n"
            
            status_text += f"\n=== DEAD RECKONING ===\n"
            status_text += f"Dead X: {self.dead_x}  |  Dead Y: {self.dead_y}\n"
            
            ax.text(self.WIDTH + 0.2, self.LENGTH - 0.5, status_text, fontsize=8, 
                   verticalalignment='top', fontfamily='monospace',
                   bbox=dict(boxstyle="round,pad=0.5", facecolor='lightgray', alpha=0.9))
            
            ax.set_xlim(-1, self.WIDTH + 6)
            ax.set_ylim(-1, self.LENGTH + 1)
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.set_title('Robot Localization Visualizer - Real vs Predicted Position', fontsize=14, weight='bold')
        
        def update_real_x(val):
            self.real_x = slider_real_x.val
            update_plot()
            plt.draw()
        
        def update_real_y(val):
            self.real_y = slider_real_y.val
            update_plot()
            plt.draw()
        
        def update_real_theta(val):
            self.real_theta = slider_real_theta.val
            update_plot()
            plt.draw()
        
        slider_real_x.on_changed(update_real_x)
        slider_real_y.on_changed(update_real_y)
        slider_real_theta.on_changed(update_real_theta)
        
        update_plot()
        plt.tight_layout()
        plt.show()

# Create and run the visualizer
if __name__ == "__main__":
    localizer = RobotLocalizer()
    localizer.visualize()