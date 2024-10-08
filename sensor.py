import math
from matplotlib.transforms import Affine2D
from shapely import LineString, Point


class Sensor:
    def __init__(self):
        #Initializes the sensor with default geometry and position
        self.sensor_line = LineString() 
        self.set_geometry(0.05, 0.03)
        self.last_seen = 1

    def set_geometry(self, width, distance):
        #Sets the geometry of the sensor.
        self.width = width
        self.distance = max(distance, 0.01)
        self.update_sensor_line()

    def set_sensor_position(self, distance):
        #Sets the sensor's distance from the robot
        self.set_geometry(self.width, distance / 100.0)
        print("Sensor position set to", self.distance)

    def set_sensor_width(self, width):
        #Sets the sensor's width.
        self.set_geometry(width / 100.0, self.distance)
        print("Sensor width set to", self.width)

    def update_sensor_line(self):
        #Updates the sensor line based on current width and position.
        x1 = -self.width / 2  
        x2 = self.width / 2  
        y = 0  
        self.sensor_line = LineString([(x1, y), (x2, y)])  # Create a line representing the sensor.

    def update_position(self, robot):

        #Updates the sensor's position based on the robot's position and orientation.

        self.update_sensor_line()  # Ensure the sensor line is up-to-date.

        # Calculate sensor's position relative to the robot.
        sensor_x = 0  
        sensor_y = robot.height / 2 + self.distance  

        # Apply rotation and translation to the sensor line based on the robot's position.
        t = Affine2D().rotate(robot.angle).translate(
            robot.x + sensor_x * math.cos(robot.angle) + sensor_y * math.sin(robot.angle),
            robot.y + sensor_x * math.sin(robot.angle) - sensor_y * math.cos(robot.angle)
        )
        self.sensor_line = LineString(t.transform(self.sensor_line.coords))  # Transform the sensor line.

    def find_closest_intersection(self, path):
        #Finds the closest intersection between the sensor line and the path
        intersections = self.sensor_line.intersection(path)

        if intersections.is_empty:
            return None  # No intersection found.
        elif isinstance(intersections, Point):
            return intersections  # A single intersection point.
        elif isinstance(intersections, LineString):
            return Point(intersections.coords[0])  # First point on the intersection line.
        else:
            # Multiple intersection points, return the closest one to the sensor's origin.
            return min(intersections.geoms, key=lambda p: Point(p).distance(Point(self.sensor_line.coords[0])))

    def get_line_position(self, path):
        #Determines the sensor's position relative to the path.
        intersection = self.find_closest_intersection(path)

        if intersection:
            # Calculate the position of the intersection along the sensor line.
            sensor_start = Point(self.sensor_line.coords[0])
            sensor_end = Point(self.sensor_line.coords[-1])
            sensor_length = sensor_start.distance(sensor_end)
            position = sensor_start.distance(intersection) / sensor_length
            normalized_position = (position * 2) - 1
            self.last_seen = normalized_position  
            return normalized_position
        else:
            return self.last_seen  # If no intersection, return the last known position.
