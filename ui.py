import tkinter as tk
from tkinter import ttk
import logging
import matplotlib.pyplot as plt

class ControlPanel(tk.Tk):
    def __init__(self, robot, sensor, pid, path_drawer):
        super().__init__()
        self.title("Line Follower Robot Simulator")

        self.robot = robot
        self.sensor = sensor
        self.pid = pid
        self.path_drawer = path_drawer

        # Set up the main canvas for robot visualization
        self.canvas_width = 800
        self.canvas_height = 800
        self.robot_canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create a frame for controls
        controls_frame = self._create_controls_frame()

        # Draw the initial shape of the robot and start animation
        self.robot.draw_shape(self.robot_canvas)

        # Set up PID sliders and geometry controls
        self._create_pid_controls(controls_frame)
        self._create_geometry_controls(controls_frame)

        # Add reset position button
        self._create_reset_button(controls_frame)

        # Set up acceleration controls
        self._create_acceleration_controls(controls_frame)

        # Configure the grid to expand properly
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        controls_frame.grid_rowconfigure(4, weight=1)

        # Set up closing protocol
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_controls_frame(self):
        """Create and return the control frame on the right side of the window."""
        controls_frame = tk.Frame(self)
        controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
        return controls_frame

    def _create_pid_controls(self, parent):
        """Create PID controller sliders in a labeled frame."""
        pid_frame = ttk.LabelFrame(parent, text="PID Controller", padding=(10, 10))
        pid_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Create PID parameter sliders and speed/frequency sliders
        self.p_slider, self.p_value_label = self._create_slider(pid_frame, "P:", 0, self.update_p, initial=2.0)
        self.i_slider, self.i_value_label = self._create_slider(pid_frame, "I:", 1, self.update_i, initial=2.0)
        self.d_slider, self.d_value_label = self._create_slider(pid_frame, "D:", 2, self.update_d, initial=3.0)
        self.speed_slider, self.speed_value_label = self._create_slider(pid_frame, "Speed:", 3, self.update_speed, to=500.0, initial=50.0)
        self.freq_slider, self.freq_value_label = self._create_slider(pid_frame, "Frequency:", 4, self.update_frequency, to=50, initial=20)

    def _create_geometry_controls(self, parent):
        """Create geometry control sliders (wheel gauge, sensor position, sensor width)."""
        geometry_frame = ttk.LabelFrame(parent, text="Geometry", padding=(10, 10))
        geometry_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Create sliders for wheel gauge, sensor position, and sensor width
        self.wheel_gauge_slider, self.wheel_gauge_value_label = self._create_slider(geometry_frame, "Wheel Gauge:", 0, self.update_wheel_gauge, from_=0.01, to=0.2, initial=0.05)
        self.sensor_pos_slider, self.sensor_pos_value_label = self._create_slider(geometry_frame, "Sensor Position:", 1, self.update_sensor_position, from_=0.03, to=30, initial=self.sensor.distance * 100)
        self.sensor_width_slider, self.sensor_width_value_label = self._create_slider(geometry_frame, "Sensor Width:", 2, self.update_sensor_width, from_=2, to=20, initial=self.sensor.width * 100)

    def _create_acceleration_controls(self, parent):
        """Create acceleration slider."""
        accel_frame = ttk.LabelFrame(parent, text="Acceleration", padding=(10, 10))
        accel_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")
        self.accel_slider, self.accel_value_label = self._create_slider(accel_frame, "Acceleration:", 0, self.update_acceleration, from_=1.0, to=100.0, initial=25.0)

    def _create_reset_button(self, parent):
        """Create reset position button."""
        reset_button = ttk.Button(parent, text="Reset Position", command=self.reset_position)
        reset_button.grid(row=2, column=0, pady=10)

    def _create_slider(self, parent, label, row, command, from_=0.0, to=10.0, initial=0.0):
        """Create a slider with a label and value display."""
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="e")

        # Value display label
        value_label = ttk.Label(parent, text=f"{initial:.2f}")
        value_label.grid(row=row, column=2, sticky="w")

        # Slider control
        slider = ttk.Scale(parent, from_=from_, to=to, orient=tk.HORIZONTAL,
                        command=lambda value, lbl=value_label: self._update_slider_value(value, command, lbl))
        slider.set(initial)
        slider.grid(row=row, column=1, sticky="ew")

        return slider, value_label

    def _update_slider_value(self, value, command, value_label):
        """Update the value label and execute the associated command when the slider value changes."""
        value_float = float(value)
        value_label.config(text=f"{value_float:.2f}")
        command(value_float)

    # === Update methods for sliders ===
    def update_p(self, value):
        """Update the proportional gain (P) in the PID controller."""
        self.pid.set_p(float(value))
        logging.info(f"Proportional gain set to {value}")

    def update_i(self, value):
        """Update the integral gain (I) in the PID controller."""
        self.pid.set_i(float(value))
        logging.info(f"Integral gain set to {value}")

    def update_d(self, value):
        """Update the derivative gain (D) in the PID controller."""
        self.pid.set_d(float(value))
        logging.info(f"Derivative gain set to {value}")

    def update_speed(self, speed):
        """Update the robot speed."""
        speed_value = float(speed)
        self.robot.set_speed(speed_value)
        self.pid.set_speed(speed_value)
        logging.info(f"Robot speed set to {speed_value}")

    def update_frequency(self, freq):
        """Update the frequency of the PID controller."""
        self.pid.set_frequency(int(float(freq)))
        logging.info(f"PID frequency set to {freq}")

    def update_wheel_gauge(self, value):
        """Update the wheel gauge of the robot."""
        self.robot.set_wheel_gauge(float(value))
        self.robot.update_robot()
        logging.info(f"Wheel gauge set to {value}")

    def update_sensor_position(self, value):
        """Update the position of the sensor relative to the robot."""
        self.sensor.set_sensor_position(float(value))
        self.robot.update_robot()
        logging.info(f"Sensor position set to {value}")

    def update_sensor_width(self, value):
        """Update the width of the sensor."""
        self.sensor.set_sensor_width(float(value))
        self.robot.update_robot()
        logging.info(f"Sensor width set to {value}")

    def update_acceleration(self, value):
        """Update the robot's acceleration."""
        self.robot.set_acceleration(float(value))
        logging.info(f"Acceleration set to {value}")

    def reset_position(self):
        """Reset the robot's position to its initial state."""
        self.robot.reset_position()
        logging.info("Robot position reset")

    # === Window close handling ===
    def on_closing(self):
        """Handle the window close event, ensuring all components are cleaned up."""
        self.robot.on_close()
        self.pid.stop()
        plt.close('all')
        self.quit()
        self.destroy()
        logging.info("Application closed")

def create_control_panel(robot, sensor, pid, path_drawer):
    """Function to create and return the control panel."""
    return ControlPanel(robot, sensor, pid, path_drawer)

