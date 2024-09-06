import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from robot import Robot, MotorController, Sensor
from pid_regulator import PIDregulator
from sand_box import SandBox

class ControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Line Follower Robot Simulator")

        # Initialize robot components
        self.motor_ctrl = MotorController()
        self.sensor = Sensor()
        self.robot = Robot(self.sensor)
        self.pid = PIDregulator(self.motor_ctrl, self.sensor)
        
        # Set up the main canvas
        self.canvas_width = 800
        self.canvas_height = 800
        self.robot_canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        # Create a frame for controls
        controls_frame = tk.Frame(self)
        controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Initialize the sandbox and draw the initial shape
        self.sandbox = SandBox(self.robot, self.pid)
        self.sandbox.draw_shape(self.robot_canvas)

        # Set up PID controller frame
        pid_frame = ttk.LabelFrame(controls_frame, text="PID Controller", padding=(10, 10))
        pid_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create sliders for PID parameters
        self._create_pid_slider(pid_frame, "P:", 0, self.pid.set_p)
        self._create_pid_slider(pid_frame, "I:", 1, self.pid.set_i)
        self._create_pid_slider(pid_frame, "D:", 2, self.pid.set_d)
        self._create_pid_slider(pid_frame, "Speed:", 3, self.set_speed, to=200.0)
        self._create_pid_slider(pid_frame, "Frequency:", 4, lambda f: self.pid.set_frecuency(int(float(f))), to=50)

        # Set up robot geometry frame
        robot_frame = ttk.LabelFrame(controls_frame, text="Geometry", padding=(10, 10))
        robot_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Create sliders for robot geometry
        self._create_geometry_slider(robot_frame, "Wheel Gauge:", 0, self.update_wheel_gauge, from_=0.01, to=0.2, initial=0.05)
        self._create_geometry_slider(robot_frame, "Sensor Position:", 1, self.update_sensor_position, from_=0.03, to=30, initial=self.sensor.distance * 100)
        self._create_geometry_slider(robot_frame, "Sensor Width:", 2, self.update_sensor_width, from_=2, to=20, initial=self.sensor.width * 100)

        # Add reset position button
        ttk.Button(controls_frame, text="Reset Position", command=self.reset_position).grid(row=2, column=0, pady=10)

        # Set up acceleration frame
        accel_frame = ttk.LabelFrame(controls_frame, text="Acceleration", padding=(10, 10))
        accel_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

        # Create acceleration slider
        self._create_geometry_slider(accel_frame, "Acceleration:", 0, self.set_acceleration, from_=1.0, to=100.0, initial=1)
        
        # Configure grid weights
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        controls_frame.grid_rowconfigure(4, weight=1)

        # Set up closing protocol
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Start the PID regulator
        self.pid.start()

    def _create_pid_slider(self, parent, label, row, command, from_=0.0, to=10.0):
        """Helper method to create PID control sliders"""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e")
        slider = ttk.Scale(parent, from_=from_, to=to, orient=tk.HORIZONTAL, command=command)
        slider.grid(row=row, column=1, sticky="ew")

    def _create_geometry_slider(self, parent, label, row, command, from_, to, initial):
        """Helper method to create geometry control sliders"""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e")
        slider = ttk.Scale(parent, from_=from_, to=to, orient=tk.HORIZONTAL, command=command)
        slider.set(initial)
        slider.grid(row=row, column=1, sticky="ew")
        setattr(self, f"{label.lower().replace(':', '_')}_slider", slider)

    def update_wheel_gauge(self, value):
        """Update the wheel gauge of the robot"""
        self.robot.set_wheel_gauge(float(value))
        self.sandbox.update_robot()

    def set_speed(self, speed):
        """Set the speed for both the sandbox and PID regulator"""
        speed_value = float(speed)
        self.sandbox.set_speed(speed_value)
        self.pid.set_speed(speed_value)

    def set_acceleration(self, acceleration):
        """Set the acceleration of the robot"""
        accel_value = float(acceleration)
        self.robot.set_acceleration(accel_value)

    def reset_position(self):
        """Reset the position of the robot in both sandbox and robot object"""
        self.sandbox.reset_position()
        self.robot.reset_position()

    def update_sensor_position(self, value):
        """Update the sensor position on the robot"""
        self.sensor.set_sensor_position(float(value))
        self.sandbox.update_robot()

    def update_sensor_width(self, value):
        """Update the sensor width on the robot"""
        self.sensor.set_sensor_width(float(value))
        self.sandbox.update_robot()

    def on_closing(self):
        """Handle the closing of the application window"""
        self.sandbox.on_close()
        self.pid.stop()
        plt.close('all')
        self.quit()
        self.destroy()

if __name__ == "__main__":
    app = ControlPanel()
    app.mainloop()