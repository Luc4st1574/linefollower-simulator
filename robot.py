import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math

class Robot:
    def __init__(self, sensor, path_drawer):
        # Robot's initial parameters
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.width = 0.05
        self.height = 0.05
        self.wheel_gauge = 0.05
        self.observers = []
        self.acceleration = 0.1
        self.current_speed = 0.0
        self.sensor = sensor
        self.left_speed = 0
        self.right_speed = 0
        self.path_drawer = path_drawer
        self.path_index = 0
        self.fig = None
        self.ax = None
        self.robot_patch = None
        self.sensor_line = None
        self.canvas_agg = None
        self.ani = None
        self.target_speed = 1.0
        self.initial_position = self.path_drawer.get_path()[0]
        self.animation_running = False
        self.background = None
        self.initial_robot = None

    # === Setup methods ===
    def set_wheel_gauge(self, gauge):
        #Sets the wheel gauge (width of the robot).
        self.wheel_gauge = max(0.01, min(0.2, gauge))
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        #Registers an observer for robot updates.
        self.observers.append(observer)

    def notify_observers(self):
        #Notifies all observers when the robot's state changes.
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        #Resets the robot to its initial position and orientation.
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx)
        self.current_speed = 0.0
        self.path_index = 0
        self.notify_observers()
        print("Robot position reset")

        # Update the robot patch position
        robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
        self.robot_patch.set_transform(robot_t + self.ax.transData)

        # Update sensor position
        self.sensor.update_position(self)
        sensor_coords = np.array(self.sensor.sensor_line.coords)
        self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])

        # Redraw the canvas
        self.fig.canvas.draw()

    # === Control methods ===
    def set_acceleration(self, acceleration):
        #Sets the robot's acceleration.
        self.acceleration = acceleration
        print(f"Acceleration set to {self.acceleration}")

    def set_motor_speeds(self, left, right):
        #Sets the speed of the robot's left and right motors.
        self.left_speed = left
        self.right_speed = right

    def set_speed(self, speed):
        #Sets the target speed for the robot.
        self.target_speed = speed / 100.0
        print(f"Target speed set to {self.target_speed}")

    def update_speed(self, target_speed):
        #Gradually changes the robot's current speed towards the target speed.
        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed + self.acceleration, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed - self.acceleration, target_speed)
        return self.current_speed

    # === Update methods ===
    def update_position(self, x, y, angle):
        #Updates the robot's position and orientation.
        self.x = x
        self.y = y
        self.angle = angle + math.pi / 2
        self.notify_observers()

    def update_robot(self):
        if self.robot_patch and self.sensor_line:
            self.robot_patch.set_width(self.width)
            self.robot_patch.set_height(self.height)
            robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
            self.robot_patch.set_transform(robot_t + self.ax.transData)
            self.robot_patch.set_xy((-self.width / 2, -self.height / 2))
            self.sensor.update_position(self)
            sensor_coords = np.array(self.sensor.sensor_line.coords)
            self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])
            self.check_bounds()
            self.fig.canvas.draw_idle() 
            self.fig.canvas.flush_events()

    def check_bounds(self):
        #Ensures the robot stays within the bounds of the plot.
        x_min, x_max = self.ax.get_xlim()
        y_min, y_max = self.ax.get_ylim()
        padding = 0.1

        if self.x < x_min + padding:
            self.ax.set_xlim(self.x - padding, x_max + (x_min - self.x + padding))
        elif self.x > x_max - padding:
            self.ax.set_xlim(x_min - (self.x - x_max + padding), self.x + padding)

        if self.y < y_min + padding:
            self.ax.set_ylim(self.y - padding, y_max + (y_min - self.y + padding))
        elif self.y > y_max - padding:
            self.ax.set_ylim(y_min - (self.y - y_max + padding), self.y + padding)

    # === Drawing and Animation ===
    def draw_shape(self, canvas):
        #Initializes the robot's drawing and starts the animation.
        x, y = zip(*self.path_drawer.get_path())
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.plot(x, y, 'k-', linewidth=2)
        self.ax.fill(x, y, edgecolor='black', fill=False)
        self.set_limits(x, y)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('off')

        # Initial robot orientation and position
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx)

        # Draw robot as a red rectangle
        self.robot_patch = Rectangle((0, -self.height / 2), self.width, self.height, color="red", zorder=10)
        self.ax.add_patch(self.robot_patch)

        # Draw sensor as a blue line
        sensor_x = [-self.sensor.width / 2, self.sensor.width / 2]
        sensor_y = [self.sensor.distance, self.sensor.distance]
        self.sensor_line, = self.ax.plot(sensor_x, sensor_y, 'b-', linewidth=2, zorder=11)

        self.update_robot()

        # Embed canvas into the Tkinter GUI
        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)

        # Cache the background without the initial robot
        self.robot_patch.set_visible(False)
        self.sensor_line.set_visible(False)
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self.robot_patch.set_visible(True)
        self.sensor_line.set_visible(True)

        # Start the animation loop
        self.start_animation()

    def set_limits(self, x, y):
        #Sets plot limits with padding.
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)
        padding = 0.2
        x_range = x_max - x_min
        y_range = y_max - y_min
        self.ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
        self.ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)

    def start_animation(self):
        def update(frame):
            if not self.animation_running:
                return self.robot_patch, self.sensor_line

            # Update speed and position only if needed
            current_speed = self.update_speed(self.target_speed)
            if current_speed > 0:
                # Update robot's path position
                path_len = len(self.path_drawer.get_path())
                self.path_index = (self.path_index + current_speed) % path_len
                new_position = self.path_drawer.get_path()[int(self.path_index)]
                self.x, self.y = new_position

                # Update angle for smooth movement
                next_index = (int(self.path_index) + 1) % path_len
                next_position = self.path_drawer.get_path()[next_index]
                dx = next_position[0] - new_position[0]
                dy = next_position[1] - new_position[1]
                self.angle = np.arctan2(dy, dx) + np.pi / 2

                # Restore the background
                self.fig.canvas.restore_region(self.background)

                # Update robot position and orientation
                robot_t = Affine2D().rotate(self.angle).translate(self.x, self.y)
                self.robot_patch.set_transform(robot_t + self.ax.transData)

                # Update sensor position
                self.sensor.update_position(self)
                sensor_coords = np.array(self.sensor.sensor_line.coords)
                self.sensor_line.set_data(sensor_coords[:, 0], sensor_coords[:, 1])

                # Draw only the updated patches
                self.ax.draw_artist(self.robot_patch)
                self.ax.draw_artist(self.sensor_line)

                # Redraw the canvas
                self.fig.canvas.blit(self.ax.bbox)
                self.fig.canvas.flush_events()

            return self.robot_patch, self.sensor_line

        self.animation_running = True
        self.ani = FuncAnimation(self.fig, update, frames=None, interval=50, blit=True, repeat=True, cache_frame_data=False)
        self.canvas_agg.draw()
    
    
    def on_close(self):
        #Handles the closure of the robot animation.
        self.animation_running = False
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None
