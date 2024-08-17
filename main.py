import tkinter as tk
from shapely.geometry import LineString
from src.robot import Robot, MotorController, Sensor
from src.PIDregulator import PIDregulator
from src.bezier_intersection import BezierIntersection

class ControlPanel(tk.Tk):
    def __init__(self, path):
        super().__init__()
        self.title("Control Panel")

        # Initialize MotorController, Sensor, and Robot
        self.motor_ctrl = MotorController()
        self.sensor = Sensor(path)
        self.robot = Robot(path, x=0.0, y=0.0, angle=0.0)
        self.pid = PIDregulator(self.motor_ctrl, self.sensor)

        print("Path coordinates:", path.coords)
        
        # Initialize RobotCanvas and SimulationEngine
        self.robot_canvas = tk.Canvas(self, width=600, height=300, bg="white")
        # Layout configuration
        self.robot_canvas.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        controls_frame = tk.Frame(self)
        controls_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # PID Controls
        pid_frame = tk.LabelFrame(controls_frame, text="PID Controller", padx=10, pady=10)
        pid_frame.grid(row=0, column=0, padx=10, pady=10)

        tk.Label(pid_frame, text="P:").grid(row=0, column=0)
        self.p_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda p: self.pid.set_p(float(p)))
        self.p_slider.grid(row=0, column=1)

        tk.Label(pid_frame, text="I:").grid(row=1, column=0)
        self.i_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda i: self.pid.set_i(float(i)))
        self.i_slider.grid(row=1, column=1)

        tk.Label(pid_frame, text="D:").grid(row=2, column=0)
        self.d_slider = tk.Scale(pid_frame, from_=0.0, to_=10.0, orient=tk.HORIZONTAL, 
                                    command=lambda d: self.pid.set_d(float(d)))
        self.d_slider.grid(row=2, column=1)

        tk.Label(pid_frame, text="Speed:").grid(row=3, column=0)
        self.speed_slider = tk.Scale(pid_frame, from_=0.0, to_=100.0, orient=tk.HORIZONTAL, 
                                        command=lambda s: self.pid.set_speed(float(s)))
        self.speed_slider.grid(row=3, column=1)

        tk.Label(pid_frame, text="Frequency:").grid(row=4, column=0)
        self.freq_intvar = tk.IntVar(pid_frame, value=50)
        self.freq_slider = tk.Scale(pid_frame, from_=1, to_=50, orient=tk.HORIZONTAL, 
                                    variable=self.freq_intvar, command=lambda f: self.pid.set_frecuency(int(f)))
        self.freq_slider.grid(row=4, column=1)

        # Robot Controls
        robot_frame = tk.LabelFrame(controls_frame, text="Geometry", padx=10, pady=10)
        robot_frame.grid(row=1, column=0, padx=10, pady=10)

        tk.Label(robot_frame, text="Wheel Gauge:").grid(row=0, column=0)
        self.wheel_gauge_slider = tk.Scale(robot_frame, from_=20, to_=100, orient=tk.HORIZONTAL, 
                                            command=lambda g: self.robot.set_wheel_gauge(float(g)))
        self.wheel_gauge_slider.grid(row=0, column=1)

        tk.Label(robot_frame, text="Sensor Position:").grid(row=1, column=0)
        self.sensor_position_slider = tk.Scale(robot_frame, from_=10, to_=50, orient=tk.HORIZONTAL, 
                                                command=lambda pos: self.sensor.set_geometry(self.sensor.width, float(pos)))
        self.sensor_position_slider.grid(row=1, column=1)

        tk.Label(robot_frame, text="Sensor Width:").grid(row=2, column=0)
        self.sensor_width_slider = tk.Scale(robot_frame, from_=10, to_=50, orient=tk.HORIZONTAL, 
                                            command=lambda w: self.sensor.set_geometry(float(w), self.sensor.location))
        self.sensor_width_slider.grid(row=2, column=1)

        # Reset Button
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

if __name__ == "__main__":
    # Example path: Define a simple LineString path for the sensor to follow
    example_path = LineString([(0, 0), (100, 0), (200, 100)])

    app = ControlPanel(example_path)
    app.mainloop()
