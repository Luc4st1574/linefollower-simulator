import numpy as np
from shapely.geometry import LineString

class PathDrawer:
    def __init__(self):
        self.egg_path = self.generate_egg_shape()
        self.path_line = LineString(self.egg_path)

    def generate_egg_shape(self, a=0.5, b=0.65, num_points=1000, scale_factor=0.75):
        theta = np.linspace(0, 2 * np.pi, num_points)
        x = a * (1 - 0.6 * np.cos(theta)) * np.cos(theta) * scale_factor
        y = b * np.sin(theta) * scale_factor
        return list(zip(x, y))

    def get_path(self):
        return self.egg_path

    def get_path_line(self):
        return self.path_line