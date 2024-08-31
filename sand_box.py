import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
from matplotlib.animation import FuncAnimation

class SandBox:
    def __init__(self, robot):
        self.robot = robot
        self.path_index = 0
        self.fig = None
        self.ax = None
        self.robot_patch = None
        self.canvas_agg = None
        self.ani = None
        self.egg_path = self.generate_egg_shape()
        self.robot.add_observer(self)  # Add SandBox as an observer to the robot
        self.speed = 1.0  # Default speed
        self.initial_position = self.egg_path[0]  # Store the initial position

    def generate_egg_shape(self, a=0.5, b=0.65, num_points=1000, scale_factor=0.75):
        theta = np.linspace(0, 2 * np.pi, num_points)
        x = a * (1 - 0.6 * np.cos(theta)) * np.cos(theta) * scale_factor
        y = b * np.sin(theta) * scale_factor
        return list(zip(x, y))

    def draw_shape(self, canvas):
        x, y = zip(*self.egg_path)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.plot(x, y, 'k-', linewidth=2)
        self.ax.fill(x, y, edgecolor='black', fill=False)

        # Calculate the bounds of the path
        x_min, x_max = min(x), max(x)
        y_min, y_max = min(y), max(y)

        # Add some padding to the bounds
        padding = 0.1
        x_range = x_max - x_min
        y_range = y_max - y_min
        self.ax.set_xlim(x_min - padding * x_range, x_max + padding * x_range)
        self.ax.set_ylim(y_min - padding * y_range, y_max + padding * y_range)

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.axis('off')

        # Initialize robot position
        self.robot.x, self.robot.y = self.initial_position

        # Initialize the robot's angle to face the path
        next_position = self.egg_path[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.robot.angle = np.arctan2(dy, dx)

        # Create the robot patch with the correct orientation
        self.robot_patch = Rectangle(
            (0, -self.robot.height / 2),  # Center the rectangle on its width
            self.robot.width,
            self.robot.height,
            color="red",
            zorder=10  # Ensure the robot is drawn on top
        )
        self.update_robot()  # This will set the correct position and rotation

        self.ax.add_patch(self.robot_patch)

        self.canvas_agg = FigureCanvasTkAgg(self.fig, master=canvas)
        self.canvas_agg.draw()
        self.canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)

        self.start_animation()

    def start_animation(self):
        def update(frame):
            self.path_index = (self.path_index + self.speed) % len(self.egg_path)
            new_position = self.egg_path[int(self.path_index)]
            self.robot.x, self.robot.y = new_position

            next_index = (int(self.path_index) + 1) % len(self.egg_path)
            next_position = self.egg_path[next_index]
            dx = next_position[0] - new_position[0]
            dy = next_position[1] - new_position[1]
            self.robot.angle = np.arctan2(dy, dx)

            self.update_robot()
            return self.robot_patch,

        self.ani = FuncAnimation(
            self.fig, update, frames=len(self.egg_path), interval=50, blit=True, repeat=True
        )
        self.canvas_agg.draw()

    def update_robot(self):
        if self.robot_patch:
            # Update the width and height of the patch
            self.robot_patch.set_width(self.robot.width)
            self.robot_patch.set_height(self.robot.height)
            
            # Create a new transform
            t = Affine2D().rotate(self.robot.angle + np.pi/2).translate(self.robot.x, self.robot.y)
            self.robot_patch.set_transform(t + self.ax.transData)
            
            # Update the patch's xy position to keep it centered on its width
            self.robot_patch.set_xy((-self.robot.width / 2, -self.robot.height / 2))
            
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def update_robot_width(self):
        if self.robot_patch:
            self.update_robot()

    def set_speed(self, speed):
        self.speed = speed / 10.0  # Adjust the scaling factor as needed
        print(f"Speed set to {self.speed}")

    def reset_position(self):
        self.path_index = 0
        self.robot.x, self.robot.y = self.initial_position
        next_position = self.egg_path[1]
        dx = next_position[0] - self.initial_position[0]
        dy = next_position[1] - self.initial_position[1]
        self.robot.angle = np.arctan2(dy, dx)
        self.update_robot()
        print("Robot position reset")

    def on_close(self, event):
        if self.ani is not None:
            self.ani.event_source.stop()
        plt.close(self.fig)