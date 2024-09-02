import math
from shapely.geometry import LineString, Point
from shapely.ops import split
from matplotlib.transforms import Affine2D

class Robot:
    def __init__(self, sensor):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.width = 0.05 
        self.height = 0.05  # Fixed height
        self.wheel_gauge = 0.05 
        self.observers = []
        self.acceleration = 0.1  # Default acceleration
        self.current_speed = 0.0  # Current speed of the robot
        self.sensor = sensor  # Store the sensor as an attribute
        
    def set_wheel_gauge(self, gauge):
        self.wheel_gauge = max(0.01, min(0.2, gauge))  # Limit the gauge between 0.01 and 0.2
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        self.observers.append(observer)

    def notify_observers(self):
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.current_speed = 0.0
        self.notify_observers()

    def update_position(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle + math.pi/2  # Add 90 degrees to the angle
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
    
class Sensor:
    def __init__(self):
        self.sensor_line = LineString()
        self.set_geometry(0.05, 0.02)  # Default width and distance
        self.last_seen = 1
        
    def set_geometry(self, width, distance):
        self.width = width
        self.distance = max(distance, 0.02)  # Ensure minimum distance of 0.02
        self.update_sensor_line()

    def set_sensor_position(self, distance):
        self.set_geometry(self.width, distance / 100.0)  # Convert slider value to appropriate scale
        print("Sensor position set to", self.distance)

    def set_sensor_width(self, width):
        self.set_geometry(width / 100.0, self.distance)  # Convert slider value to appropriate scale
        print("Sensor width set to", self.width)

    def update_sensor_line(self):
        y1 = -self.width / 2
        y2 = self.width / 2
        x = self.distance
        self.sensor_line = LineString([(x, y1), (x, y2)])

    def update_position(self, robot):
        self.update_sensor_line()
        # Rotate the sensor line to align with the robot's direction and position it at the front
        t = Affine2D().rotate(robot.angle).translate(
            robot.x + robot.width / 2 * math.cos(robot.angle),
            robot.y + robot.width / 2 * math.sin(robot.angle)
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
        else:  # MultiPoint or GeometryCollection
            return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])))

class MotorController:
    def __init__(self):
        self.left_speed = 0
        self.desired_left_speed = 0
        self.right_speed = 0
        self.desired_right_speed = 0
        self.acceleration = 0
    
    def update_speed(self):
        self.left_speed = self.compute_speed(self.left_speed, self.desired_left_speed)
        self.right_speed = self.compute_speed(self.right_speed, self.desired_right_speed)
        
    def compute_speed(self, speed, desired):
        if abs(speed - desired) <= self.acceleration:
            return desired
        elif speed < desired:
            return speed + self.acceleration
        else:
            return speed - self.acceleration
    
    def set_speed(self, left, right):
        self.desired_left_speed = left
        self.desired_right_speed = right
    
    def get_left_speed(self):
        return self.left_speed
    
    def get_right_speed(self):
        return self.right_speed
    
    def set_acceleration(self, acceleration):
        self.acceleration = acceleration
        print("Acceleration set to", acceleration)