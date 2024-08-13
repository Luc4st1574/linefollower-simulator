import numpy as np
from scipy.optimize import fsolve
from shapely.geometry import LineString, Point
from math import atan, cos, sin

EPSILON = 1E-5

class BezierIntersection:
    @staticmethod
    def get_intersection(path, line):
        intersections = []
        
        coords = list(path.coords)
        last_point = coords[0]
        
        for i in range(1, len(coords)):
            segment = coords[i]
            if len(segment) == 6:  # Cubic Bézier curve segment
                curve = [
                    last_point,
                    (segment[0], segment[1]),
                    (segment[2], segment[3]),
                    (segment[4], segment[5])
                ]
                intersections.extend(BezierIntersection.get_intersections_curve(curve, line))
                last_point = (segment[4], segment[5])
            else:
                raise NotImplementedError("Only cubic Bézier curve intersections are implemented")
        return intersections
    
    @staticmethod
    
    def get_intersections_curve(curve, line):
        y = np.array([p[1] for p in curve])
        x = np.array([p[0] for p in curve])

        if abs(line.coords[0][0] - line.coords[1][0]) <= EPSILON:
            # Switch x and y and offset
            x, y = y, x
            y -= line.coords[0][0]
        else:
            # Calculate the slope and intercept of the line
            b = (line.coords[0][1] - line.coords[1][1]) / (line.coords[0][0] - line.coords[1][0])
            c = line.coords[0][1] - b * line.coords[0][0]

            angle = -atan(b)

            # Calculate translation vector for the line
            vect_x = -(b * c) / (b * b + 1)
            vect_y = c / (b * b + 1)

            for i in range(len(y)):
                # Do control point translation
                x[i] -= vect_x
                y[i] -= vect_y

                # Rotate (transformation) y coordinates only
                y[i] = x[i] * sin(angle) + y[i] * cos(angle)

        coefs = np.array([
            -y[0] + 3*y[1] - 3*y[2] + y[3],
            3*y[0] - 6*y[1] + 3*y[2],
            -3*y[0] + 3*y[1],
            y[0]
        ])

        roots = fsolve(lambda t: np.polyval(coefs, t), [0, 0.33, 0.67, 1])

        intersections = []
        for root in roots:
            if 0 <= root <= 1:
                point = BezierIntersection.get_point_on_curve(curve, root)
                if BezierIntersection.point_lies_on_line(point, line):
                    intersections.append(point)
        return intersections
    
    @staticmethod
    def get_point_on_curve(curve, t):
        x0, x1, x2, x3 = [p[0] for p in curve]
        y0, y1, y2, y3 = [p[1] for p in curve]

        x = (1 - t)**3 * x0 + 3 * (1 - t)**2 * t * x1 + 3 * (1 - t) * t**2 * x2 + t**3 * x3
        y = (1 - t)**3 * y0 + 3 * (1 - t)**2 * t * y1 + 3 * (1 - t) * t**2 * y2 + t**3 * y3

        return Point(x, y)
    
    @staticmethod
    def point_lies_on_line(point, line):
        return line.distance(point) <= EPSILON