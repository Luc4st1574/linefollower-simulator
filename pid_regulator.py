import threading
import time

class PIDregulator:
    def __init__(self, motor_ctrl, sensor):
        self.p = 0.0
        self.i = 0.0
        self.d = 0.0
        self.speed = 0.0
        self.dt = 0
        self.last_error = 0.0
        self.sumLinePositions = 0.0
        self.motor_ctrl = motor_ctrl
        self.sensor = sensor
        self.timer = threading.Thread(target=self.run)
        self.running = False # Flag to indicate when the loop is running and control it
        
    def star_PID_regulator(self):
        self.running = True# Set the flag to True to start the loop
        self.timer = threading.Thread(target=self.running)
        self.timer.start()
        
    def set_frecuency(self, freq):
        freq = int(freq)
        self.dt = 1000/freq
        print("Frequency:", freq)
        
    def run(self):
        tm = time.time()
        while self.running:
            
            position = self.sensor.get_line_position()
            
            if abs(position) > 1:
                if (position < 0):
                    self.motor_ctrl.set_speed(0, self.speed)
                else:
                    self.motor_ctrl.set_speed(self.speed, 0)
            else:
                error = 0.0 - position
                self.sumLinePositions += (error-self.last_error)*self.dt
                
                u = self.sumLinePositions*error/1
                u = u * self.speed
                self.last_error = position
                
                if u < 0:
                    vl = min(-u, self.speed)
                    vr = self.speed
                else:
                    vl = self.speed
                    vr = min(u, self.speed)
                
                self.motor_ctrl.set_speed(vl, vr)
                
                try:
                    tm += self.dt
                    time.sleep(max(0, tm-time.time()))
                except InterruptedError:
                    break
    
    def set_p (self, p):
        self.p = p
        print("P:", self.p)
    
    def set_i (self, i):
        self.i = i
        print("I:", self.i)
        
    def set_d (self, d):
        self.d = d
        print("D:", self.d)
        
    def set_speed (self, speed):
        self.speed = speed
        print("Speed:", self.speed)
        
    def stop(self):
        self.running = False  # Set the flag to False to stop the loop
        if self.timer is not None:
            self.timer.join()  # Wait for the thread to finish

