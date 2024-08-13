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
        
