from robot import Robot, Sensor, PIDregulator
from path import PathDrawer
from ui import create_control_panel

def main():
    # Initialize components
    path_drawer = PathDrawer()
    sensor = Sensor()
    robot = Robot(sensor, path_drawer)
    pid = PIDregulator(robot, sensor)

    # Create and run the control panel
    app = create_control_panel(robot, sensor, pid, path_drawer)
    
    # Start the PID regulator
    pid.start()
    
    # Run the main event loop
    app.mainloop()

if __name__ == "__main__":
    main()