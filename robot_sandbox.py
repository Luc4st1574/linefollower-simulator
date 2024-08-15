import tkinter as tk
import time
from PIL import Image, ImageDraw, ImageTk  # Import ImageTk from PIL
from threading import Thread
import math

class RobotCanvas(tk.Canvas):
    def __init__(self, master, width=600, height=300):
        super().__init__(master, width=width, height=height, bg="white")
        self.pack()
        self.guideline = []
        self.robot = None

    def draw_elements(self):
        img = Image.new("RGB", (self.winfo_width(), self.winfo_height()), "white")
        draw = ImageDraw.Draw(img)
        self.draw_guideline(draw)
        self.draw_robot(draw)
        self.photo_image = ImageTk.PhotoImage(img)
        self.create_image(0, 0, anchor=tk.NW, image=self.photo_image)
        self.draw_circuit()

    def draw_circuit(self):
        # Define control points for the Bézier curves to create a smooth circuit
        points = [
            (150, 150),  # Start
            (100, 50),   # Control point for first curve
            (200, 50),   # End of first curve
            (300, 100),  # Control point for second curve
            (300, 200),  # End of second curve
            (200, 250),  # Control point for third curve
            (100, 250),  # End of third curve
            (50, 200),   # Control point for fourth curve
            (50, 100),   # End of fourth curve
        ]

        # Draw the curves to form the circuit
        for i in range(0, len(points), 3):
            if i + 3 <= len(points):
                self.create_line(
                    points[i][0], points[i][1],
                    points[i + 1][0], points[i + 1][1],
                    points[i + 2][0], points[i + 2][1],
                    smooth=True, fill="black", width=3
                )

        # Connect the end of the last curve to the start point to close the loop
        self.create_line(
            points[-1][0], points[-1][1],
            points[0][0], points[0][1],
            smooth=True, fill="black", width=3
        )
    
    def draw_guideline(self, draw):
        for curve in self.guideline:
            draw.line(curve[:2], fill="black")
            draw.line(curve[2:], fill="black")

    def draw_robot(self, draw):
        draw.ellipse([self.robot.x - 5, self.robot.y - 5, self.robot.x + 5, self.robot.y + 5], outline="blue")

class SimulationEngine:
    def __init__(self, robot, canvas, delay=1000 // 50):
        self.robot = robot
        self.canvas = canvas
        self.delay = delay
        self.running = True

    def start(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def run(self):
        while self.running:
            self.robot.move()
            self.canvas.draw_elements()
            time.sleep(self.delay / 1000.0)

    def stop(self):
        self.running = False
        self.thread.join()
        
