import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

class SandBox:
    def __init__(self, robot):
        self.control_points = [
            [0.2, 0.5],
            [0.4, 0.8],
            [0.7, 0.9],
            [0.9, 0.6],
            [0.8, 0.3],
            [0.5, 0.2],
            [0.2, 0.4],
            [0.2, 0.5]
        ]
        self.robot = robot
        self.robot.add_observer(self)
        self.bezier_path = self.generate_bezier_path(self.control_points)
        self.path_index = 0
        self.fig = None
        self.ax = None
        self.robot_patch = None
        self.canvas_agg = None
        self.ani = None

    def bezier_curve(self, t, control_points):
        n = len(control_points) - 1
        point = np.zeros(2)
        for i, P in enumerate(control_points):
            bernstein_poly = math.comb(n, i) * (t**i) * ((1 - t)**(n - i))
            point += bernstein_poly * np.array(P)
        return point

    def generate_bezier_path(self, control_points, num_points=100):
        return [self.bezier_curve(t, control_points) for t in np.linspace(0, 1, num_points)]

    def draw_shape(self, canvas):
        x, y = zip(*self.bezier_path)

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.plot(x, y, 'k-')
        self.ax.fill(x, y, edgecolor='black', fill=False)

        # Calculate the bounds of the path
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)

        # Add some padding to the bounds
        padding = 0.2
        x_range = x_max - x_min
        y_range = y_max - y_min
        self.ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
        self.ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('on')

        # Initialize robot position
        initial_position = self.bezier_path[0]
        self.robot.x, self.robot.y = initial_position

        self.robot_patch = Rectangle(
            (self.robot.x - self.robot.width / 2, self.robot.y - self.robot.height / 2),
            self.robot.width,
            self.robot.height,
            color="red"
        )
        self.ax.add_patch(self.robot_patch)

        def update(frame):
            if self.path_index < len(self.bezier_path) - 1:
                self.path_index += 1
            else:
                self.path_index = 0

            new_position = self.bezier_path[self.path_index]
            self.robot.x, self.robot.y = new_position

            if self.path_index < len(self.bezier_path) - 1:
                next_position = self.bezier_path[self.path_index + 1]
                dx = next_position[0] - new_position[0]
                dy = next_position[1] - new_position[1]
                self.robot.angle = math.atan2(dy, dx)

            self.update_robot_size()
            return self.robot_patch,

        self.ani = FuncAnimation(
            self.fig, update, frames=len(self.bezier_path), interval=50, blit=True, repeat=True
        )

        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)

    def update_robot_size(self):
        if self.robot_patch:
            self.robot_patch.set_width(self.robot.width)
            self.robot_patch.set_height(self.robot.height)
            self.robot_patch.set_xy((self.robot.x - self.robot.width / 2, self.robot.y - self.robot.height / 2))
            self.robot_patch.set_angle(math.degrees(self.robot.angle))
            if self.canvas_agg:
                self.canvas_agg.draw_idle()

    def start_animation(self):
        if self.ani is not None:
            self.ani.event_source.start()

    def stop_animation(self):
        if self.ani is not None:
            self.ani.event_source.stop()