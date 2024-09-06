import math
from shapely.geometry import LineString, Point
from matplotlib.transforms import Affine2D

class Robot:
    def __init__(self, sensor):
        # Initialize robot's position and orientation
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        
        # Set robot's physical properties
        self.width = 0.05 
        self.height = 0.05  # Fixed height
        self.wheel_gauge = 0.05 
        
        # Initialize list for observers (for updating UI)
        self.observers = []
        
        # Set default acceleration and current speed
        self.acceleration = 0.1
        self.current_speed = 0.0
        
        # Store the sensor as an attribute
        self.sensor = sensor
        
    def set_wheel_gauge(self, gauge):
        # Set wheel gauge with limits
        self.wheel_gauge = max(0.01, min(0.2, gauge))  # Limit the gauge between 0.01 and 0.2
        self.width = self.wheel_gauge
        self.notify_observers()

    def add_observer(self, observer):
        # Add an observer to the list
        self.observers.append(observer)

    def notify_observers(self):
        # Notify all observers to update
        for observer in self.observers:
            observer.update_robot()

    def reset_position(self):
        # Reset robot to initial position and state
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.current_speed = 0.0
        self.notify_observers()

    def update_position(self, x, y, angle):
        # Update robot's position and orientation
        self.x = x
        self.y = y
        self.angle = angle + math.pi/2  # Add 90 degrees to the angle
        self.notify_observers()

    def set_acceleration(self, acceleration):
        # Set the robot's acceleration
        self.acceleration = acceleration
        print(f"Acceleration set to {self.acceleration}")

    def update_speed(self, target_speed):
        # Update the robot's speed based on acceleration and target speed
        if self.current_speed < target_speed:
            self.current_speed = min(self.current_speed + self.acceleration, target_speed)
        elif self.current_speed > target_speed:
            self.current_speed = max(self.current_speed - self.acceleration, target_speed)
        return self.current_speed
    
class Sensor:
    def __init__(self):
        # Initialize sensor line
        self.sensor_line = LineString()
        self.set_geometry(0.05, 0.03)  # Default width, default distance
        self.last_seen = 1
        
    def set_geometry(self, width, distance):
        # Set sensor geometry
        self.width = width
        self.distance = max(distance, 0.01)  # Ensure minimum distance of 0.01
        self.update_sensor_line()

    def set_sensor_position(self, distance):
        # Set sensor position (distance from robot center)
        self.set_geometry(self.width, distance / 100.0)  # Convert slider value to appropriate scale
        print("Sensor position set to", self.distance)

    def set_sensor_width(self, width):
        # Set sensor width
        self.set_geometry(width / 100.0, self.distance)  # Convert slider value to appropriate scale
        print("Sensor width set to", self.width)

    def update_sensor_line(self):
        # Update the sensor line geometry
        x1 = -self.width / 2
        x2 = self.width / 2
        y = 0  # The sensor line is always at y=0 in its local coordinate system
        self.sensor_line = LineString([(x1, y), (x2, y)])

    def update_position(self, robot):
        # Update sensor position based on robot's position and orientation
        self.update_sensor_line()
        
        # Calculate the position of the sensor relative to the robot's center
        # Place the sensor on top of the robot
        sensor_x = 0
        sensor_y = robot.height / 2 + self.distance
        
        # Create a transformation that rotates and translates the sensor
        t = Affine2D().rotate(robot.angle).translate(
            robot.x + sensor_x * math.cos(robot.angle) + sensor_y * math.sin(robot.angle),
            robot.y + sensor_x * math.sin(robot.angle) - sensor_y * math.cos(robot.angle)
        )
        
        self.sensor_line = LineString(t.transform(self.sensor_line.coords))

    def find_closest_intersection(self, path):
        # Find the closest intersection between the sensor line and the path
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
        # Initialize motor speeds and acceleration
        self.left_speed = 0
        self.desired_left_speed = 0
        self.right_speed = 0
        self.desired_right_speed = 0
        self.acceleration = 0
    
    def update_speed(self):
        # Update both motor speeds
        self.left_speed = self.compute_speed(self.left_speed, self.desired_left_speed)
        self.right_speed = self.compute_speed(self.right_speed, self.desired_right_speed)
        
    def compute_speed(self, speed, desired):
        # Compute new speed based on current speed, desired speed, and acceleration
        if abs(speed - desired) <= self.acceleration:
            return desired
        elif speed < desired:
            return speed + self.acceleration
        else:
            return speed - self.acceleration
    
    def set_speed(self, left, right):
        # Set desired speeds for both motors
        self.desired_left_speed = left
        self.desired_right_speed = right
    
    def get_left_speed(self):
        # Get current left motor speed
        return self.left_speed
    
    def get_right_speed(self):
        # Get current right motor speed
        return self.right_speed
    
    def set_acceleration(self, acceleration):
        # Set motor acceleration
        self.acceleration = acceleration
        print("Acceleration set to", acceleration)