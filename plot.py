import tkinter as tk
import math
import numpy as np
import threading
import time
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, box, Point
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class MotorController:
    def __init__(self):
        self.left_speed = 0
        self.right_speed = 0
        self.desired_left_speed = 0
        self.desired_right_speed = 0
        self.acceleration = 0
    
    def update(self):
        self.left_speed = self._compute_speed(self.left_speed, self.desired_left_speed)
        self.right_speed = self._compute_speed(self.right_speed, self.desired_right_speed)
    
    def _compute_speed(self, current, target):
        if abs(current - target) <= self.acceleration:
            return target
        elif current < target:
            return current + self.acceleration
        else:
            return current - self.acceleration
    
    def set_speed(self, left, right):
        self.desired_left_speed = left
        self.desired_right_speed = right
    
    def set_acceleration(self, accel):
        self.acceleration = accel


class Sensor:
    def __init__(self, path):
        self.path = path
        self.sensor_line = LineString()
        self.set_geometry(30, 20)

    def set_geometry(self, width, location):
        self.width = width
        self.location = location
        self.hypotenuse = math.sqrt((width / 2) ** 2 + location ** 2)
        self.angle = math.atan((width / 2) / location)

    def update_position(self, robot):
        a = robot.angle - self.angle
        x1 = robot.x + self.hypotenuse * math.cos(a)
        y1 = robot.y + self.hypotenuse * math.sin(a)

        a = robot.angle + self.angle
        x2 = robot.x + self.hypotenuse * math.cos(a)
        y2 = robot.y + self.hypotenuse * math.sin(a)

        self.sensor_line = LineString([(x1, y1), (x2, y2)])


class Robot:
    def __init__(self, path):
        self.path = path
        self.index = 0
        self.position = self.path[self.index]
        self.angle = 0.0
        self.x, self.y = 0.0, 0.0
        self.width, self.height = 50, 30
        self.wheel_gauge = 60
        self.motor_ctrl = MotorController()
        self.sensor = Sensor(path)
        self.bounds = box(-self.width / 2, -self.height / 2, self.width / 2, self.height / 2)
        self.reset_position()

    def move(self):
        self.motor_ctrl.update()
        self.angle += (self.motor_ctrl.left_speed - self.motor_ctrl.right_speed) / self.wheel_gauge
        distance = (self.motor_ctrl.left_speed + self.motor_ctrl.right_speed) / 2
        self.x += distance * math.cos(self.angle)
        self.y += distance * math.sin(self.angle)
        self.sensor.update_position(self)
        self.update_shape()

    def reset_position(self):
        self.x, self.y, self.angle = 0.0, 0.0, 0.0
        print("Robot position reset")

    def update_shape(self):
        self.shape = rotate(self.bounds, math.degrees(self.angle), origin='centroid')
        self.shape = translate(self.shape, xoff=self.x, yoff=self.y)
        self.shape = self.shape.union(self.sensor.sensor_line)

    def update_position(self):
        if self.index < len(self.path) - 1:
            self.index += 1
        self.position = self.path[self.index]


class PIDregulator:
    def __init__(self, motor_ctrl, sensor):
        self.p, self.i, self.d = 0, 0, 0
        self.speed = 0
        self.freq = 1
        self.motor_ctrl = motor_ctrl
        self.sensor = sensor
        self.timer = threading.Thread(target=self.run)
        self.running = False

    def start(self):
        self.running = True
        self.timer.start()

    def run(self):
        while self.running:
            position = self.sensor.get_line_position()
            if abs(position) > 1:
                if position < 0:
                    self.motor_ctrl.set_speed(0, self.speed)
                else:
                    self.motor_ctrl.set_speed(self.speed, 0)
            else:
                error = 0.0 - position
                u = error * self.p + self.i * (error - self.d)
                self.motor_ctrl.set_speed(self.speed - u, self.speed + u)
            time.sleep(1 / self.freq)


class SandBox:
    def __init__(self, robot):
        self.control_points = [
            [0.2, 0.5], [0.4, 0.8], [0.7, 0.9], [0.9, 0.6],
            [0.8, 0.3], [0.5, 0.2], [0.2, 0.4], [0.2, 0.5]
        ]
        self.robot = robot

    def bezier_curve(self, t, control_points):
        n = len(control_points) - 1
        point = np.zeros(2)
        for i, P in enumerate(control_points):
            bernstein_poly = math.comb(n, i) * (t**i) * ((1 - t)**(n - i))
            point += bernstein_poly * np.array(P)
        return point

    def generate_bezier_path(self, control_points):
        return [self.bezier_curve(t, control_points) for t in np.linspace(0, 1, 100)]

    def draw_shape(self, canvas):
        bezier_path = self.generate_bezier_path(self.control_points)
        x, y = zip(*bezier_path)

        fig, ax = plt.subplots(figsize=(6, 6))
        ax.plot(x, y, 'k-')
        ax.fill(x, y, edgecolor='black', fill=False)
        ax.set_aspect('equal', adjustable='box')
        ax.axis('off')

        robot_patch = Rectangle(
            (self.robot.position[0] - self.robot.width / 2, self.robot.position[1] - self.robot.height / 2),
            self.robot.width, self.robot.height, color="red"
        )
        ax.add_patch(robot_patch)

        def update(frame):
            self.robot.update_position()
            robot_patch.set_xy(
                (self.robot.position[0] - self.robot.width / 2, self.robot.position[1] - self.robot.height / 2)
            )
            return robot_patch,

        ani = FuncAnimation(fig, update, frames=len(bezier_path), interval=50, blit=True, repeat=True)
        canvas_agg = FigureCanvasTkAgg(fig, master=canvas)
        canvas_agg.draw()
        canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)


class ControlPanel(tk.Tk):
    def __init__(self, path):
        super().__init__()
        self.title("Control Panel")

        self.robot = Robot(path)
        self.pid = PIDregulator(self.robot.motor_ctrl, self.robot.sensor)
        self.canvas_width = 600
        self.canvas_height = 300

        self.robot_canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.sandbox = SandBox(self.robot)
        self.sandbox.draw_shape(self.robot_canvas)

        self.setup_controls()

    def setup_controls(self):
        control_frame = tk.Frame(self)
        control_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        pid_frame = tk.LabelFrame(control_frame, text="PID Controller")
        pid_frame.grid(row=0, column=0)

        tk.Label(pid_frame, text="P:").grid(row=0, column=0)
        self.p_slider = tk.Scale(pid_frame, from_=0, to_=10, orient=tk.HORIZONTAL, command=lambda p: self.pid.set_p(float(p)))
        self.p_slider.grid(row=0, column=1)

        tk.Label(pid_frame, text="Speed:").grid(row=1, column=0)
        self.speed_slider = tk.Scale(pid_frame, from_=0, to_=100, orient=tk.HORIZONTAL, command=lambda s: self.pid.set_speed(float(s)))
        self.speed_slider.grid(row=1, column=1)


if __name__ == "__main__":
    path = [[0.2, 0.5], [0.4, 0.8], [0.7, 0.9], [0.9, 0.6], [0.8, 0.3], [0.5, 0.2], [0.2, 0.4], [0.2, 0.5]]
    app = ControlPanel(path)
    app.mainloop()