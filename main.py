import tkinter as tk
from src.robot import Robot, MotorController, Sensor
from src.PIDregulator import PIDregulator
from src.bezier_intersection import BezierIntersection
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
        self.canvas_width = 600
        self.canvas_height = 300
        self.robot_canvas = tk.Canvas(self, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        controls_frame = tk.Frame(self)
        controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Draw the shape on the canvas
        self.sandbox = SandBox()  # Initialize the SandBox class
        self.sandbox.draw_shape(self.robot_canvas)

        # PID Controls
        pid_frame = tk.LabelFrame(controls_frame, text="PID Controller", padx=10, pady=10)
        pid_frame.grid(row=0, column=0, padx=10, pady=10)

        tk.Label(pid_frame, text="P:").grid(row=0, column=0)
        self.p_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda p:self.pid.set_p(p))
        self.p_slider.grid(row=0, column=1)

        tk.Label(pid_frame, text="I:").grid(row=1, column=0)
        self.i_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda i: self.pid.set_i(i))
        self.i_slider.grid(row=1, column=1)

        tk.Label(pid_frame, text="D:").grid(row=2, column=0)
        self.d_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda d: self.pid.set_d(d))
        self.d_slider.grid(row=2, column=1)

        tk.Label(pid_frame, text="Speed:").grid(row=3, column=0)
        self.speed_slider = tk.Scale(pid_frame, from_=0.0, to_=100.0, orient=tk.HORIZONTAL, 
                                        command=lambda s: self.pid.set_speed(s))
        self.speed_slider.grid(row=3, column=1)

        tk.Label(pid_frame, text="Frequency:").grid(row=4, column=0)
        self.freq_slider = tk.Scale(pid_frame, from_=1, to_=50, orient=tk.HORIZONTAL, 
                            command=lambda f: self.pid.set_frecuency(int(f)))
        self.freq_slider.grid(row=4, column=1)

        # Robot Controls
        robot_frame = tk.LabelFrame(controls_frame, text="Geometry", padx=10, pady=10)
        robot_frame.grid(row=1, column=0, padx=10, pady=10)

        tk.Label(robot_frame, text="Wheel Gauge:").grid(row=0, column=0)
        self.wheel_gauge_slider = tk.Scale(robot_frame, from_=5, to_=90, orient=tk.HORIZONTAL, 
                                            command=lambda g: self.robot.set_wheel_gauge(int(g)))
        self.wheel_gauge_slider.grid(row=0, column=1)

        tk.Label(robot_frame, text="Sensor Position:").grid(row=1, column=0)
        self.sensor_position_slider = tk.Scale(robot_frame, from_=1, to_=90, orient=tk.HORIZONTAL, 
                                                command=lambda pos: self.sensor.set_sensor_position(int(pos)))
        self.sensor_position_slider.grid(row=1, column=1)

        tk.Label(robot_frame, text="Sensor Width:").grid(row=2, column=0)
        self.sensor_width_slider = tk.Scale(robot_frame, from_=10, to_=90, orient=tk.HORIZONTAL, 
                                            command=lambda w: self.sensor.set_sensor_width(int(w)))
        self.sensor_width_slider.grid(row=2, column=1)

        tk.Button(controls_frame, text="Reset Position", command=self.robot.reset_position).grid(row=2, column=0, pady=10)

        # Acceleration Control
        accel_frame = tk.LabelFrame(controls_frame, text="Acceleration", padx=10, pady=10)
        accel_frame.grid(row=3, column=0, padx=10, pady=10)

        tk.Label(accel_frame, text="Acceleration:").grid(row=0, column=0)
        self.accel_slider = tk.Scale(accel_frame, from_=0.1, to_=50.0, orient=tk.HORIZONTAL, 
                                        command=lambda a: self.motor_ctrl.set_acceleration(float(a)))
        self.accel_slider.grid(row=0, column=1)

        # Configure grid weights
        self.grid_columnconfigure(0, weight=2)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)
        controls_frame.grid_rowconfigure(4, weight=1)

# Example usage
if __name__ == "__main__":
    path = ([(0, 0), (1, 0), (1, 1), (0, 1)])
    app = ControlPanel(path)
    app.mainloop()