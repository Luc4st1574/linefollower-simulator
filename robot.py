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
        self.left_speed = 0
        self.right_speed = 0
        self.pid_adjustment = 0
        self.target_x = 0
        self.target_y = 0
        self.target_angle = 0
        self.smoothing_factor = 0.1

    def set_wheel_gauge(self, gauge):
        self.wheel_gauge = max(0.01, min(0.2, gauge))
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        self.observers.append(observer)

    def notify_observers(self):
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx)
        self.current_speed = 0.0
        self.path_index = 0
        self.notify_observers()
        print("Robot position reset")

    def update_position(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle + math.pi/2
        self.notify_observers()

    def set_acceleration(self, acceleration):
        self.acceleration = acceleration
        print(f"Acceleration set to {self.acceleration}")

    def update_speed(self, target_speed):
        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed + self.acceleration, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed - self.acceleration, target_speed)
        return self.current_speed

    def set_motor_speeds(self, left, right):
        self.left_speed = left
        self.right_speed = right

    def draw_shape(self, canvas):
        x, y = zip(*self.path_drawer.get_path())
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.plot(x, y, 'k-', linewidth=2)
        self.ax.fill(x, y, edgecolor='black', fill=False)
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)
        padding = 0.2  # Increased padding
        x_range = x_max - x_min
        y_range = y_max - y_min
        self.ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
        self.ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('off')
        self.x, self.y = self.initial_position
        next_position = self.path_drawer.get_path()[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.angle = np.arctan2(dy, dx)
        self.robot_patch = Rectangle(
            (0, -self.height / 2),
            self.width,
            self.height,
            color="red",
            zorder=10
        )
        self.ax.add_patch(self.robot_patch)
        sensor_x = [-self.sensor.width / 2, self.sensor.width / 2]
        sensor_y = [self.sensor.distance, self.sensor.distance]
        self.sensor_line, = self.ax.plot(sensor_x, sensor_y, 'b-', linewidth=2, zorder=11)
        self.update_robot()
        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)
        self.start_animation()

    def start_animation(self):
        def update(frame):
            if not self.animation_running:
                return self.robot_patch, self.sensor_line

            current_speed = self.update_speed(self.target_speed)
            self.path_index = (self.path_index + current_speed) % len(self.path_drawer.get_path())
            new_position = self.path_drawer.get_path()[int(self.path_index)]
            self.target_x, self.target_y = new_position
            next_index = (int(self.path_index) + 1) % len(self.path_drawer.get_path())
            next_position = self.path_drawer.get_path()[next_index]
            dx = next_position[0] - new_position[0]
            dy = next_position[1] - new_position[1]
            self.target_angle = np.arctan2(dy, dx) + np.pi/2 + self.pid_adjustment * 0.01

            # Smooth movement towards target position and angle
            self.x += (self.target_x - self.x) * self.smoothing_factor
            self.y += (self.target_y - self.y) * self.smoothing_factor
            angle_diff = (self.target_angle - self.angle + np.pi) % (2 * np.pi) - np.pi
            self.angle += angle_diff * self.smoothing_factor

            self.sensor.update_position(self)
            self.update_robot()
            return self.robot_patch, self.sensor_line

        self.animation_running = True
        self.ani = FuncAnimation(
            self.fig, update, frames=None, interval=20, blit=True, repeat=True,
            cache_frame_data=False
        )
        self.canvas_agg.draw()

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
            
            # Check if robot is out of bounds and adjust plot limits if necessary
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
            
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def set_speed(self, speed):
        self.target_speed = speed / 100.0
        print(f"Target speed set to {self.target_speed}")
        
    def set_pid_adjustment(self, adjustment):
        self.pid_adjustment = adjustment

    def on_close(self):
        self.animation_running = False
        if self.ani is not None:
            self.ani.event_source.stop()
            self.ani = None
        if self.fig is not None:
            plt.close(self.fig)
            self.fig = None

class Sensor:
    def __init__(self):
        self.sensor_line = LineString()
        self.set_geometry(0.05, 0.03)
        self.last_seen = 1

    def set_geometry(self, width, distance):
        self.width = width
        self.distance = max(distance, 0.01)
        self.update_sensor_line()

    def set_sensor_position(self, distance):
        self.set_geometry(self.width, distance / 100.0)
        print("Sensor position set to", self.distance)

    def set_sensor_width(self, width):
        self.set_geometry(width / 100.0, self.distance)
        print("Sensor width set to", self.width)

    def update_sensor_line(self):
        x1 = -self.width / 2
        x2 = self.width / 2
        y = 0
        self.sensor_line = LineString([(x1, y), (x2, y)])

    def update_position(self, robot):
        self.update_sensor_line()
        sensor_x = 0
        sensor_y = robot.height / 2 + self.distance
        t = Affine2D().rotate(robot.angle).translate(
            robot.x + sensor_x * math.cos(robot.angle) + sensor_y * math.sin(robot.angle),
            robot.y + sensor_x * math.sin(robot.angle) - sensor_y * math.cos(robot.angle)
        )
        self.sensor_line = LineString(t.transform(self.sensor_line.coords))

    def find_closest_intersection(self, path):
        intersections = self.sensor_line.intersection(path)
        if intersections.is_empty:
            return None
        elif isinstance(intersections, Point):
            return intersections
        elif isinstance(intersections, LineString):
            return Point(intersections.coords[0])
        else:
            return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])))

    def get_line_position(self, path):
        intersection = self.find_closest_intersection(path)
        if intersection:
            sensor_start = Point(self.sensor_line.coords[0])
            sensor_end = Point(self.sensor_line.coords[-1])
            sensor_length = sensor_start.distance(sensor_end)
            position = sensor_start.distance(intersection) / sensor_length
            normalized_position = (position * 2) - 1  # Convert to range [-1, 1]
            self.last_seen = normalized_position
            return normalized_position
        else:
            return self.last_seen  # Return the last known position if no intersection is found

class PIDregulator:
    def __init__(self, robot, sensor):
        self.p = 0.1
        self.i = 0.01
        self.d = 0.05
        self.speed = 1.0
        self.dt = 50
        self.last_error = 0.0
        self.sum_line_positions = 0.0
        self.robot = robot
        self.sensor = sensor
        self.running = False
        self.thread = None

    def run(self):
        while self.running:
            position = self.sensor.get_line_position(self.robot.path_drawer.get_path_line())
            error = 0.0 - position
            self.sum_line_positions += error * self.dt / 1000
            
            u = self.p * error + self.i * self.sum_line_positions + self.d * (error - self.last_error) / (self.dt / 1000)
            u = u * self.speed
            
            self.last_error = error
            
            # Set the PID adjustment to the robot
            self.robot.set_pid_adjustment(u)
            
            time.sleep(self.dt / 1000)

    def set_frecuency(self, freq):
        freq = max(1, min(int(freq), 50))
        self.dt = 1000 / freq
        print("Frequency:", freq)

    def set_p(self, p):
        self.p = float(p)
        print("P:", self.p)

    def set_i(self, i):
        self.i = float(i)
        print("I:", self.i)

    def set_d(self, d):
        self.d = float(d)
        print("D:", self.d)

    def set_speed(self, speed):
        self.speed = float(speed) / 100.0  # Convert to 0-1 range
        print("Speed:", self.speed)
        
    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()