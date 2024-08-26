import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.interpolate import splprep, splev


class SandBox:
    def __init__(self):
        # Initialize any properties you need for the SandBox
        self.points = np.array([
            [0.2, 0.5],
            [0.4, 0.8],
            [0.7, 0.9],
            [0.9, 0.6],
            [0.8, 0.3],
            [0.5, 0.2],
            [0.2, 0.4],
            [0.2, 0.5]  # Closing the shape by repeating the first point
        ])
    
    def draw_shape(self, canvas)
        # Separate the points into x and y coordinates
        x, y = self.points.T

        # Use B-spline interpolation to smooth the curve
        tck, u = splprep([x, y], s=0, per=True)  # s=0 gives us a curve that passes through all points
        u_new = np.linspace(u.min(), u.max(), 1000)
        x_new, y_new = splev(u_new, tck, der=0)

        # Create a matplotlib figure
        fig, ax = plt.subplots(figsize=(6, 3))
        ax.plot(x_new, y_new, 'k-')  # 'k-' means black line
        ax.fill(x_new, y_new, edgecolor='red', fill=False)  # Fill the shape with no color
        ax.set_aspect('equal', adjustable='box')
        ax.axis('off')  # Turn off the axis

        # Embed the plot in the Tkinter canvas
        canvas_agg = FigureCanvasTkAgg(fig, master=canvas)
        canvas_agg.draw()
        canvas_agg.get_tk_widget().pack(side="top", fill="both", expand=True)