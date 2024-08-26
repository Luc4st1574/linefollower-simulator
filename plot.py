import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
import math


def bezier_curve(t, control_points):
    """Compute a point on a Bezier curve given the control points and a parameter t."""
    n = len(control_points) - 1
    point = np.zeros(2)
    for i, P in enumerate(control_points):
        bernstein_poly = math.comb(n, i) * (t**i) * ((1 - t) ** (n - i))
        point += bernstein_poly * np.array(P)
    return point


def generate_bezier_path(control_points, num_points=100):
    """Generate a path along the Bezier curve with a specified number of points."""
    return [bezier_curve(t, control_points) for t in np.linspace(0, 1, num_points)]


# Define control points for the Bezier curve (forming a closed loop)
control_points = [
            [0.2, 0.5],
            [0.4, 0.8],
            [0.7, 0.9],
            [0.9, 0.6],
            [0.8, 0.3],
            [0.5, 0.2],
            [0.2, 0.4],
            [0.2, 0.5]  # Closing the shape by repeating the first point
        ]

# Generate the path
bezier_path = generate_bezier_path(control_points, num_points=1000)


class LineFollowerRobot:
    def __init__(self, path):
        self.path = path
        self.index = 0
        self.position = self.path[self.index]

    def update_position(self):
        if self.index < len(self.path) - 1:
            self.index += 1
        self.position = self.path[self.index]


# Initialize the robot
robot = LineFollowerRobot(bezier_path)

# Set up the plot
fig, ax = plt.subplots()
ax.set_aspect("equal")
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

# Plot the Bezier curve path
bezier_x, bezier_y = zip(*bezier_path)
ax.plot(bezier_x, bezier_y, color="black", linewidth=2)

# Initialize the robot's graphical representation
robot_patch = Circle(robot.position, 0.2, color="red")
ax.add_patch(robot_patch)


# Update function for animation
def update(frame):
    robot.update_position()
    robot_patch.set_center(robot.position)
    return (robot_patch,)


# Create animation
ani = FuncAnimation(
    fig, update, frames=len(bezier_path), interval=20, blit=True, repeat=True
)

# Show the plot
plt.show()
