import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
from shapely.geometry import LineString

class SandBox:
    def __init__(self, robot, pid_regulator):
        # Initialize SandBox with robot and PID regulator
        self.robot = robot
        self.pid_regulator = pid_regulator
        self.path_index = 0
        
        # Initialize matplotlib figure and axes
        self.fig = None
        self.ax = None
        self.robot_patch = None
        self.sensor_line = None
        self.canvas_agg = None
        self.ani = None
        
        # Generate egg-shaped path
        self.egg_path = self.generate_egg_shape()
        self.path_line = LineString(self.egg_path)
        
        # Add robot as an observer
        self.robot.add_observer(self)
        
        # Set initial speed and position
        self.target_speed = 1.0
        self.initial_position = self.egg_path[0]
        self.animation_running = False

    def generate_egg_shape(self, a=0.5, b=0.65, num_points=1000, scale_factor=0.75):
        # Generate points for an egg-shaped path
        theta = np.linspace(0, 2 * np.pi, num_points)
        x = a * (1 - 0.6 * np.cos(theta)) * np.cos(theta) * scale_factor
        y = b * np.sin(theta) * scale_factor
        return list(zip(x, y))

    def draw_shape(self, canvas):
        # Draw the egg-shaped path and set up the matplotlib figure
        x, y = zip(*self.egg_path)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.plot(x, y, 'k-', linewidth=2)
        self.ax.fill(x, y, edgecolor='black', fill=False)

        # Set axis limits with padding
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)
        padding = 0.1
        x_range = x_max - x_min
        y_range = y_max - y_min
        self.ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
        self.ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('off')

        # Set initial robot position and orientation
        self.robot.x, self.robot.y = self.initial_position
        next_position = self.egg_path[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.robot.angle = np.arctan2(dy, dx)

        # Create robot patch
        self.robot_patch = Rectangle(
            (0, -self.robot.height / 2),
            self.robot.width,
            self.robot.height,
            color="red",
            zorder=10
        )
        self.ax.add_patch(self.robot_patch)

        # Initialize sensor line
        sensor_x = [-self.robot.sensor.width / 2, self.robot.sensor.width / 2]
        sensor_y = [self.robot.sensor.distance, self.robot.sensor.distance]
        self.sensor_line, = self.ax.plot(sensor_x, sensor_y, 'b-', linewidth=2, zorder=11)

        self.update_robot()

        # Set up canvas for tkinter
        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)

        self.start_animation()

    def start_animation(self):
        def update(frame):
            if not self.animation_running:
                return self.robot_patch, self.sensor_line

            # Update robot position and orientation
            current_speed = self.robot.update_speed(self.target_speed)
            self.path_index = (self.path_index + current_speed) % len(self.egg_path)
            new_position = self.egg_path[int(self.path_index)]
            self.robot.x, self.robot.y = new_position

            next_index = (int(self.path_index) + 1) % len(self.egg_path)
            next_position = self.egg_path[next_index]
            dx = next_position[0] - new_position[0]
            dy = next_position[1] - new_position[1]
            self.robot.angle = np.arctan2(dy, dx) + np.pi/2  # Add 90 degrees to the angle

            self.robot.sensor.update_position(self.robot)

            # Find closest intersection and update PID regulator
            line_position = self.robot.sensor.find_closest_intersection(self.path_line)
            if line_position:
                normalized_position = (line_position.x - self.robot.x) / (self.robot.sensor.width / 2)
                self.pid_regulator.sensor.last_seen = normalized_position

            self.update_robot()
            return self.robot_patch, self.sensor_line

        self.animation_running = True
        self.ani = FuncAnimation(
            self.fig, update, frames=None, interval=50, blit=True, repeat=True,
            cache_frame_data=False
        )
        self.canvas_agg.draw()

    def update_robot(self):
        if self.robot_patch and self.sensor_line:
            # Update robot patch
            self.robot_patch.set_width(self.robot.width)
            self.robot_patch.set_height(self.robot.height)
            
            robot_t = Affine2D().rotate(self.robot.angle).translate(self.robot.x, self.robot.y)
            self.robot_patch.set_transform(robot_t + self.ax.transData)
            
            self.robot_patch.set_xy((-self.robot.width / 2, -self.robot.height / 2))
            
            # Update sensor line
            self.robot.sensor.update_position(self.robot)
            sensor_coords = np.array(self.robot.sensor.sensor_line.coords)
            self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def set_speed(self, speed):
        # Set target speed
        self.target_speed = speed / 100.0
        print(f"Target speed set to {self.target_speed}")

    def reset_position(self):
        # Reset robot position and orientation
        self.path_index = 0
        self.robot.x, self.robot.y = self.initial_position
        next_position = self.egg_path[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.robot.angle = np.arctan2(dy, dx)
        self.robot.current_speed = 0.0
        self.update_robot()
        print("Robot position reset")

    def on_close(self):
        # Stop animation and close matplotlib figure
        self.animation_running = False
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None