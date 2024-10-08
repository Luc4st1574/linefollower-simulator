import logging
from robot import Robot
from sensor import Sensor
from pid_regulator import PidRegulator
from path import PathDrawer
from ui import create_control_panel

def main():
    try:
        # Initialize components
        path_drawer = PathDrawer()
        sensor = Sensor()
        robot = Robot(sensor, path_drawer)
        pid = PidRegulator(robot, sensor)

        # Create and run the control panel
        app = create_control_panel(robot, sensor, pid, path_drawer)
        
        # Start the PID regulator
        pid.start()
        logging.info("PID regulator started")

        # Run the main event loop
        app.mainloop()

    except Exception as e:
        logging.error(f"An error occurred: {e}", exc_info=True)
    
    finally:
        # Ensure PID regulator is properly stopped
        pid.stop()
        logging.info("PID regulator stopped")

if __name__ == "__main__":
    # Set up basic logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    main()
