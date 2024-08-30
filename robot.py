import math
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Point, box
from matplotlib.patches import Rectangle


class Robot:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.width = 0.05
        self.height = 0.03  # Fixed height
        self.wheel_gauge = 0.05
        self.observers = []

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
        self.notify_observers()

    def update_position(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle
        self.notify_observers()
class Sensor:
    def __init__(self):
        self.sensor_line = LineString()
        self.set_geometry(30, 20)  # Default width and location
        self.last_seen = 1
        
    def set_geometry(self, width, location):
        self.width = width
        self.location = location
        self.hypotenuse = math.sqrt((width / 2)**2 + location**2)
        self.angle = math.atan((width / 2) / location)

    def set_sensor_position(self, location):
        self.set_geometry(self.width, location)
        print("Sensor position set to", location)

    def set_sensor_width(self, width):
        self.set_geometry(width, self.location)
        print("Sensor width set to", width)

    def update_position(self, robot):
        a = robot.angle - self.angle
        x1 = robot.x + self.hypotenuse * math.cos(a)
        y1 = robot.y + self.hypotenuse * math.sin(a)
        
        a = robot.angle + self.angle
        
        x2 = robot.x + self.hypotenuse * math.cos(a)
        y2 = robot.y + self.hypotenuse * math.sin(a)
        
        self.sensor_line = LineString([(x1, y1), (x2, y2)])
    
    def find_closest_intersection(self, path):
        intersections = path.intersection(self.sensor_line)
        return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])), default=None)

    
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
        
        
        