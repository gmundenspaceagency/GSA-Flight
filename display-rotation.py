import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import threading
import time

def rotate_point(x, y, z, theta_x, theta_y):
    # Rotation um die x-Achse
    rotation_matrix_x = np.array([[1, 0, 0],
                                  [0, np.cos(theta_x), -np.sin(theta_x)],
                                  [0, np.sin(theta_x), np.cos(theta_x)]])

    # Rotation um die y-Achse
    rotation_matrix_y = np.array([[np.cos(theta_y), 0, np.sin(theta_y)],
                                  [0, 1, 0],
                                  [-np.sin(theta_y), 0, np.cos(theta_y)]])

    # Anwenden der Rotationen
    rotated_point = np.dot(rotation_matrix_y, np.dot(rotation_matrix_x, np.array([x, y, z])))

    return rotated_point

class RealTimePlotter:
    def __init__(self, root):
        self.root = root
        self.root.title("Real-Time Rotation Plot")

        self.fig = plt.Figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)
        self.canvas.draw()

        self.theta_x = 0
        self.theta_y = 0

    def update_plot(self):
        self.ax.clear()

        # Fläche nach Rotation darstellen
        rotated_points = np.array([rotate_point(xi, yi, zi, self.theta_x, self.theta_y) for xi, yi, zi in zip(x.flatten(), y.flatten(), z.flatten())])
        rotated_surface = rotated_points.reshape(x.shape[0], x.shape[1], 3)

        self.ax.plot_surface(rotated_surface[:, :, 0], rotated_surface[:, :, 1], rotated_surface[:, :, 2], cmap='viridis')
        self.ax.set_title('Rotated Surface')

        self.canvas.draw()

    def random_update(self):
        rotation = [0, 0]
        while True:
            # Generate random rotation values
            # turn = [np.random.uniform(-10, 10), np.random.uniform(-10, 10)]
            turn = [10, 0]
            rotation[0] += turn[0]
            rotation[1] += turn[1]

            # Keep rotation angles within [0, 360) range
            for r in rotation:
                if r > 360:
                    r -= 360
                elif r < 0:
                    r += 360

            self.theta_x = np.radians(rotation[0])
            self.theta_y = np.radians(rotation[1])

            self.update_plot()

            time.sleep(0.1)

# Erzeugen von Datenpunkten auf einer Ebene
x = np.linspace(-1, 1, 100)
y = np.linspace(-1, 1, 100)
x, y = np.meshgrid(x, y)
z = np.zeros_like(x)  # Constant value for a plane

root = tk.Tk()
app = RealTimePlotter(root)

# Start the random update loop in a separate thread
update_thread = threading.Thread(target=app.random_update)
update_thread.start()

root.mainloop()
