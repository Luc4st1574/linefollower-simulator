import tkinter as tk
from tkinter import ttk
from src.robot import Robot, MotorController, Sensor
from src.pid_regulator import PIDregulator
from src.sand_box import SandBox

class ControlPanel(tk.Tk):
    def __init__(self, path):
        super().__init__()
        self.title("Control Panel")

        # Initialize MotorController, Sensor, and Robot
        self.motor_ctrl = MotorController()
        self.sensor = Sensor(path)
        self.robot = Robot(path)
        self.pid = PIDregulator(self.motor_ctrl, self.sensor)
        
        # Initialize RobotCanvas
        self.canvas_width = 800
        self.canvas_height = 800
        self.robot_canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        controls_frame = tk.Frame(self)
        controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Draw the shape on the canvas
        self.sandbox = SandBox(self.robot)
        self.sandbox.draw_shape(self.robot_canvas)
        self.sandbox.start_animation()  # Start the animation immediately

        # PID Controls
        pid_frame = ttk.LabelFrame(controls_frame, text="PID Controller", padding=(10, 10))
        pid_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(pid_frame, text="P:").grid(row=0, column=0, sticky="e")
        self.p_slider = ttk.Scale(pid_frame, from_=0.0, to=10.0, orient=tk.HORIZONTAL, 
                                command=lambda p: self.pid.set_p(float(p)))
        self.p_slider.grid(row=0, column=1, sticky="ew")

        ttk.Label(pid_frame, text="I:").grid(row=1, column=0, sticky="e")
        self.i_slider = ttk.Scale(pid_frame, from_=0.0, to=10.0, orient=tk.HORIZONTAL, 
                                command=lambda i: self.pid.set_i(float(i)))
        self.i_slider.grid(row=1, column=1, sticky="ew")

        ttk.Label(pid_frame, text="D:").grid(row=2, column=0, sticky="e")
        self.d_slider = ttk.Scale(pid_frame, from_=0.0, to=10.0, orient=tk.HORIZONTAL, 
                                command=lambda d: self.pid.set_d(float(d)))
        self.d_slider.grid(row=2, column=1, sticky="ew")

        ttk.Label(pid_frame, text="Speed:").grid(row=3, column=0, sticky="e")
        self.speed_slider = ttk.Scale(pid_frame, from_=0.0, to=100.0, orient=tk.HORIZONTAL, 
                                    command=lambda s: self.pid.set_speed(float(s)))
        self.speed_slider.grid(row=3, column=1, sticky="ew")

        ttk.Label(pid_frame, text="Frequency:").grid(row=4, column=0, sticky="e")
        self.freq_slider = ttk.Scale(pid_frame, from_=1, to=50, orient=tk.HORIZONTAL, 
                                    command=lambda f: self.pid.set_frecuency(int(float(f))))
        self.freq_slider.grid(row=4, column=1, sticky="ew")

        # Robot Controls
        robot_frame = ttk.LabelFrame(controls_frame, text="Geometry", padding=(10, 10))
        robot_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(robot_frame, text="Wheel Gauge:").grid(row=0, column=0, sticky="e")
        self.wheel_gauge_slider = ttk.Scale(robot_frame, from_=0.01, to=0.2, orient=tk.HORIZONTAL, 
                                            command=self.update_wheel_gauge)
        self.wheel_gauge_slider.set(0.05)  # Set initial value
        self.wheel_gauge_slider.grid(row=0, column=1, sticky="ew")

        ttk.Label(robot_frame, text="Sensor Position:").grid(row=1, column=0, sticky="e")
        self.sensor_position_slider = ttk.Scale(robot_frame, from_=1, to=90, orient=tk.HORIZONTAL, 
                                                command=lambda pos: self.sensor.set_sensor_position(int(float(pos))))
        self.sensor_position_slider.grid(row=1, column=1, sticky="ew")

        ttk.Label(robot_frame, text="Sensor Width:").grid(row=2, column=0, sticky="e")
        self.sensor_width_slider = ttk.Scale(robot_frame, from_=10, to=90, orient=tk.HORIZONTAL, 
                                            command=lambda w: self.sensor.set_sensor_width(int(float(w))))
        self.sensor_width_slider.grid(row=2, column=1, sticky="ew")

        ttk.Button(controls_frame, text="Reset Position", command=self.robot.reset_position).grid(row=2, column=0, pady=10)

        # Acceleration Control
        accel_frame = ttk.LabelFrame(controls_frame, text="Acceleration", padding=(10, 10))
        accel_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(accel_frame, text="Acceleration:").grid(row=0, column=0, sticky="e")
        self.accel_slider = ttk.Scale(accel_frame, from_=0.1, to=50.0, orient=tk.HORIZONTAL, 
                                    command=lambda a: self.motor_ctrl.set_acceleration(float(a)))
        self.accel_slider.grid(row=0, column=1, sticky="ew")

        # Configure grid weights
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        controls_frame.grid_rowconfigure(4, weight=1)

    def update_wheel_gauge(self, value):
        self.robot.set_wheel_gauge(float(value))

# Example usage
if __name__ == "__main__":
    path = ([0.2, 0.5],
            [0.4, 0.8],
            [0.7, 0.9],
            [0.9, 0.6],
            [0.8, 0.3],
            [0.5, 0.2],
            [0.2, 0.4],
            [0.2, 0.5])
    
    app = ControlPanel(path)
    app.mainloop()