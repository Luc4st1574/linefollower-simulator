import threading
import time

class PIDregulator:
    def __init__(self, motor_ctrl, sensor):
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.speed = 0.0
        self.dt = 20  # Default to 50Hz
        self.last_error = 0.0
        self.sumLinePositions = 0.0
        self.motor_ctrl = motor_ctrl
        self.sensor = sensor
        self.running = False
        self.thread = None

    def set_frecuency(self, freq):
        freq = max(1, min(int(freq), 50))  # Limit frequency between 1 and 50 Hz
        self.dt = 1000 / freq
        print("Frequency:", freq)

    def run(self):
        while self.running:
            position = self.sensor.last_seen
            
            if abs(position) > 1:
                if position < 0:
                    self.motor_ctrl.set_speed(0, self.speed)
                else:
                    self.motor_ctrl.set_speed(self.speed, 0)
            else:
                error = 0.0 - position
                self.sumLinePositions += (error - self.last_error) * self.dt / 1000  # Convert dt to seconds
                
                u = self.p * error + self.i * self.sumLinePositions + self.d * (error - self.last_error) / (self.dt / 1000)
                u = u * self.speed
                self.last_error = error
                
                if u < 0:
                    vl = max(self.speed + u, 0)
                    vr = self.speed
                else:
                    vl = self.speed
                    vr = max(self.speed - u, 0)
                
                self.motor_ctrl.set_speed(vl, vr)
            
            time.sleep(self.dt / 1000)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self.run)
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()

    def set_p(self, p):
        self.p = p
        print("P:", self.p)
    
    def set_i(self, i):
        self.i = i
        print("I:", self.i)
        
    def set_d(self, d):
        self.d = d
        print("D:", self.d)
        
    def set_speed(self, speed):
        self.speed = speed
        print("Speed:", self.speed)