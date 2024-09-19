import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from shapely.geometry import LineString, Point
import math
import threading
import time

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

    # === Setup methods ===
    def set_wheel_gauge(self, gauge):
        """Sets the wheel gauge (width of the robot)."""
        self.wheel_gauge = max(0.01, min(0.2, gauge))
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        """Registers an observer for robot updates."""
        self.observers.append(observer)

    def notify_observers(self):
        """Notifies all observers when the robot's state changes."""
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        """Resets the robot to its initial position and orientation."""
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx)
        self.current_speed = 0.0
        self.path_index = 0
        self.notify_observers()
        print("Robot position reset")

    # === Control methods ===
    def set_acceleration(self, acceleration):
        """Sets the robot's acceleration."""
        self.acceleration = acceleration
        print(f"Acceleration set to {self.acceleration}")

    def set_motor_speeds(self, left, right):
        """Sets the speed of the robot's left and right motors."""
        self.left_speed = left
        self.right_speed = right

    def set_speed(self, speed):
        """Sets the target speed for the robot."""
        self.target_speed = speed / 100.0
        print(f"Target speed set to {self.target_speed}")

    def update_speed(self, target_speed):
        """Gradually changes the robot's current speed towards the target speed."""
        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed + self.acceleration, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed - self.acceleration, target_speed)
        return self.current_speed

    # === Update methods ===
    def update_position(self, x, y, angle):
        """Updates the robot's position and orientation."""
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
            self.fig.canvas.draw_idle()  # Use draw_idle instead of draw
            self.fig.canvas.flush_events()

    def check_bounds(self):
        """Ensures the robot stays within the bounds of the plot."""
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
        """Initializes the robot's drawing and starts the animation."""
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

        # Start the animation loop
        self.start_animation()

    def set_limits(self, x, y):
        """Sets plot limits with padding."""
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

            # Update speed and position
            current_speed = self.update_speed(self.target_speed)
            self.path_index = (self.path_index + current_speed) % len(self.path_drawer.get_path())
            new_position = self.path_drawer.get_path()[int(self.path_index)]
            self.x, self.y = new_position

            # Calculate new angle
            next_index = (int(self.path_index) + 1) % len(self.path_drawer.get_path())
            next_position = self.path_drawer.get_path()[next_index]
            dx = next_position[0] - new_position[0]
            dy = next_position[1] - new_position[1]
            self.angle = np.arctan2(dy, dx) + np.pi / 2

            # Update sensor and steering
            self.sensor.update_position(self)
            line_position = self.sensor.get_line_position(self.path_drawer.get_path_line())
            if abs(line_position) > 0.8:
                turn_factor = 0.2 * np.sign(line_position)
                self.angle += turn_factor

            self.update_robot()
            return self.robot_patch, self.sensor_line

        self.animation_running = True
        self.ani = FuncAnimation(self.fig, update, frames=None, interval=50, blit=True, repeat=True, cache_frame_data=False)
        self.canvas_agg.draw()

    def on_close(self):
        """Handles the closure of the robot animation."""
        self.animation_running = False
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None

class Sensor:
    def __init__(self):
        """Initializes the sensor with default geometry and position."""
        self.sensor_line = LineString()  # The line representing the sensor range.
        self.set_geometry(0.05, 0.03)  # Default sensor geometry (width, distance from robot).
        self.last_seen = 1  # The last position seen on the path, default to 1.

    def set_geometry(self, width, distance):
        """
        Sets the geometry of the sensor.

        Args:
            width (float): The width of the sensor line.
            distance (float): The distance from the robot to the sensor line.
        """
        self.width = width
        self.distance = max(distance, 0.01)  # Ensure a minimum distance for the sensor.
        self.update_sensor_line()  # Update sensor line after geometry changes.

    def set_sensor_position(self, distance):
        """
        Sets the sensor's distance from the robot.

        Args:
            distance (float): New distance from the robot.
        """
        self.set_geometry(self.width, distance / 100.0)
        print("Sensor position set to", self.distance)

    def set_sensor_width(self, width):
        """
        Sets the sensor's width.

        Args:
            width (float): New width for the sensor line.
        """
        self.set_geometry(width / 100.0, self.distance)
        print("Sensor width set to", self.width)

    def update_sensor_line(self):
        """Updates the sensor line based on current width and position."""
        x1 = -self.width / 2  # Left edge of the sensor line.
        x2 = self.width / 2  # Right edge of the sensor line.
        y = 0  # Sensor line is horizontal.
        self.sensor_line = LineString([(x1, y), (x2, y)])  # Create a line representing the sensor.

    def update_position(self, robot):
        """
        Updates the sensor's position based on the robot's position and orientation.
        """
        self.update_sensor_line()  # Ensure the sensor line is up-to-date.

        # Calculate sensor's position relative to the robot.
        sensor_x = 0  # Sensor is centered horizontally.
        sensor_y = robot.height / 2 + self.distance  # Sensor is placed at the front of the robot.

        # Apply rotation and translation to the sensor line based on the robot's position.
        t = Affine2D().rotate(robot.angle).translate(
            robot.x + sensor_x * math.cos(robot.angle) + sensor_y * math.sin(robot.angle),
            robot.y + sensor_x * math.sin(robot.angle) - sensor_y * math.cos(robot.angle)
        )
        self.sensor_line = LineString(t.transform(self.sensor_line.coords))  # Transform the sensor line.

    def find_closest_intersection(self, path):
        """
        Finds the closest intersection between the sensor line and the path.
        """
        intersections = self.sensor_line.intersection(path)

        if intersections.is_empty:
            return None  # No intersection found.
        elif isinstance(intersections, Point):
            return intersections  # A single intersection point.
        elif isinstance(intersections, LineString):
            return Point(intersections.coords[0])  # First point on the intersection line.
        else:
            # Multiple intersection points, return the closest one to the sensor's origin.
            return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])))

    def get_line_position(self, path):
        """
        Determines the sensor's position relative to the path.
        """
        intersection = self.find_closest_intersection(path)

        if intersection:
            # Calculate the position of the intersection along the sensor line.
            sensor_start = Point(self.sensor_line.coords[0])
            sensor_end = Point(self.sensor_line.coords[-1])
            sensor_length = sensor_start.distance(sensor_end)
            position = sensor_start.distance(intersection) / sensor_length

            # Convert the position to a range of [-1, 1].
            normalized_position = (position * 2) - 1
            self.last_seen = normalized_position  # Update the last seen position.
            return normalized_position
        else:
            return self.last_seen  # If no intersection, return the last known position.


class PIDregulator:
    def __init__(self, robot, sensor):
        """Initializes the PID regulator with default PID values and robot/sensor."""
        self.p = 0.0  # Proportional gain.
        self.i = 0.0  # Integral gain.
        self.d = 0.0  # Derivative gain.
        self.speed = 0.0  # Base speed of the robot.
        self.dt = 20  # Time interval for updates in milliseconds.
        self.last_error = 0.0  # Error from the previous cycle.
        self.sum_line_positions = 0.0  # Cumulative sum of errors (for integral term).
        self.robot = robot  # Reference to the robot being controlled.
        self.sensor = sensor  # Reference to the sensor reading path position.
        self.running = False  # Flag for whether the regulator is running.
        self.thread = None  # Thread for running the PID control loop.

    def set_frecuency(self, freq):
        """
        Sets the update frequency of the PID loop.
        """
        freq = max(1, min(int(freq), 50))  # Clamp frequency between 1 and 50 Hz.
        self.dt = 1000 / freq  # Update the time interval.
        print("Frequency:", freq)

    def run(self):
        while self.running:
            position = self.sensor.last_seen

            if abs(position) > 1:
                if position < 0:
                    self.robot.set_motor_speeds(0, self.speed)
                else:
                    self.robot.set_motor_speeds(self.speed, 0)
            else:
                error = 0.0 - position
                self.sum_line_positions += (error - self.last_error) * self.dt / 1000
                u = self.p * error + self.i * self.sum_line_positions + self.d * (error - self.last_error) / (self.dt / 1000)
                u *= self.speed
                self.last_error = error

                if u < 0:
                    vl = max(self.speed + u, 0)
                    vr = self.speed
                else:
                    vl = self.speed
                    vr = max(self.speed - u, 0)

                self.robot.set_motor_speeds(vl, vr)

            time.sleep(max(self.dt / 1000, 0.05))  # Ensure a minimum sleep time

    def start(self):
        """Starts the PID control loop in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def stop(self):
        """Stops the PID control loop."""
        self.running = False
        if self.thread:
            self.thread.join()

    def set_p(self, p):
        """Sets the proportional gain (P)."""
        self.p = p
        print("P:", self.p)

    def set_i(self, i):
        """Sets the integral gain (I)."""
        self.i = i
        print("I:", self.i)

    def set_d(self, d):
        """Sets the derivative gain (D)."""
        self.d = d
        print("D:", self.d)

    def set_speed(self, speed):
        """Sets the base speed of the robot."""
        self.speed = speed
        print("Speed:", self.speed)
