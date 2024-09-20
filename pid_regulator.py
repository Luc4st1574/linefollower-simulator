import threading
import time

class PidRegulator:
    def __init__(self, robot, sensor):
        """Initializes the PID regulator with default PID values and robot/sensor."""
        self.p = 0.0  # Proportional gain.
        self.i = 0.0  # Integral gain.
        self.d = 0.0  # Derivative gain.
        self.speed = 0.0  # Base speed of the robot.
        self.dt = 20  # Time interval for updates in milliseconds.
        self.last_error = 0.0  # Error from the previous cycle.
        self.sum_line_positions = 0.0  # Cumulative sum of errors (for integral term).
        self.robot = robot  # Reference to the robot being controlled.
        self.sensor = sensor  # Reference to the sensor reading path position.
        self.running = False  # Flag for whether the regulator is running.
        self.thread = None  # Thread for running the PID control loop.

    def set_frecuency(self, freq):
        """
        Sets the update frequency of the PID loop.
        """
        freq = max(1, min(int(freq), 50))  # Clamp frequency between 1 and 50 Hz.
        self.dt = 1000 / freq  # Update the time interval.
        print("Frequency:", freq)

    def run(self):
        while self.running:
            position = self.sensor.last_seen

            if abs(position) > 1:
                if position < 0:
                    self.robot.set_motor_speeds(0, self.speed)
                else:
                    self.robot.set_motor_speeds(self.speed, 0)
            else:
                error = 0.0 - position
                self.sum_line_positions += (error - self.last_error) * self.dt / 1000
                u = self.p * error + self.i * self.sum_line_positions + self.d * (error - self.last_error) / (self.dt / 1000)
                u *= self.speed
                self.last_error = error

                if u < 0:
                    vl = max(self.speed + u, 0)
                    vr = self.speed
                else:
                    vl = self.speed
                    vr = max(self.speed - u, 0)

                self.robot.set_motor_speeds(vl, vr)

            time.sleep(max(self.dt / 1000, 0.05))  # Ensure a minimum sleep time

    def start(self):
        """Starts the PID control loop in a separate thread."""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def stop(self):
        """Stops the PID control loop."""
        self.running = False
        if self.thread:
            self.thread.join()

    def set_p(self, p):
        """Sets the proportional gain (P)."""
        self.p = p
        print("P:", self.p)

    def set_i(self, i):
        """Sets the integral gain (I)."""
        self.i = i
        print("I:", self.i)

    def set_d(self, d):
        """Sets the derivative gain (D)."""
        self.d = d
        print("D:", self.d)

    def set_speed(self, speed):
        """Sets the base speed of the robot."""
        self.speed = speed
        print("Speed:", self.speed)
