import math
from shapely.affinity import rotate, translate
from shapely.geometry import LineString, Point, box
import threading
import time


class Robot:
    
    def __init__(self, path, x, y, angle):
        
        #Rotation of the robot, in radians
        self.angle = angle
        #X coordinate of the robot centre of rotation
        self.x = x
        #Y coordinate of the robot centre of rotation
        self.y = y
        
        #Default rotation of the robot, in radians
        self.default_angle = 0.0
        self.default_x = 0.0
        self.default_y = 0.0
        
        #Motor controller of the robot
        self.motor_ctrl = MotorController()
        self.robot_width = 20.0
        self.wheel_gauge = 0.0
        self.robot_height = 0.0
        
        #Sensor of the robot
        self.line_sensor = Sensor(path)
        self.bounds = box(-self.robot_width / 2, -self.robot_height / 2, self.robot_width / 2, self.robot_height / 2)
        
        self.set_wheel_gauge(40)
        self.reset_position()
        
    
    def reset_position(self):
        self.x = self.default_x
        self.y = self.default_y
        self.angle = self.default_angle
        print("Robot position reset", self.x, self.y, self.angle)
        
    def set_wheel_gauge(self, gauge):
        self.wheel_gauge = gauge
        self.robot_height = gauge
        self.bounds = box(-self.robot_width / 2, -self.robot_height / 2, self.robot_width / 2, self.robot_height / 2)
        print("Wheel gauge set to", gauge)
    
    def move(self):
        self.motor_ctrl.update()
        self.angle += (self.motor_ctrl.get_left_speed() - self.motor_ctrl.get_right_speed()) / self.wheel_gauge
        distance = (self.motor_ctrl.get_left_speed() + self.motor_ctrl.get_right_speed()) / 2
        self.x += distance * math.cos(self.angle)
        self.y += distance * math.sin(self.angle)
        
        self.line_sensor.update_position(self)
        self.update_shape()
    
    def update_shape(self):
        self.shape = self.bounds
        self.shape = rotate(self.shape, math.degrees(self.angle), origin='centroid')
        self.shape = translate(self.shape, xoff=self.x, yoff=self.y)
        self.shape = self.shape.union(self.line_sensor.sensor_line)
    
    def calculate_new_position(self, distance, angle):
        new_x = self.x + distance * math.cos(angle)
        new_y = self.y + distance * math.sin(angle)
        return new_x, new_y
    


class Sensor:
    def __init__(self, path):
        self.sensor_line = LineString()
        self.path = path
        self.set_geometry(30, 20)  # Default width and location
        self.last_seen = 1
        
    def set_geometry(self, width, location):
        #Sets the sensor geometry with the given width and location.
        self.width = width
        self.location = location
        self.hypotenuse = math.sqrt((width / 2)**2 + location**2)
        self.angle = math.atan((width / 2) / location)

    def set_sensor_position(self, location):
        #Sets the position of the sensor relative to the robot.
        self.set_geometry(self.width, location)
        print("Sensor position set to", location)

    def set_sensor_width(self, width):
        #Sets the width of the sensor's detection area.
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
    
    def find_closest_intersection(self):
        intersections = self.path.intersection(self.sensor_line)
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