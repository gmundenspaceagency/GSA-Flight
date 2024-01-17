import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from gyroscope import GYRO
import numpy as np
import math

gyro = GYRO()

def rotation_matrix(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def rotate_rectangle(rectangle, x_angle, y_angle):
    center = np.mean(rectangle, axis=0)
    centered_rectangle = rectangle - center
    rotation_matrix_x = rotation_matrix([1, 0, 0], x_angle)
    rotation_matrix_y = rotation_matrix([0, 1, 0], y_angle)
    rotated_rectangle = np.dot(centered_rectangle, rotation_matrix_x.T)
    rotated_rectangle = np.dot(rotated_rectangle, rotation_matrix_y.T)
    rotated_rectangle += center
    return rotated_rectangle

def rotation():
    acceleration = gyro.get_scaled_acceleration()
    rotation = gyro.get_rotation(*acceleration)
    return rotation

def update_plot(frame, x_rotation, y_rotation):
    ax.cla()  # Clear the previous plot
    ax.set_xlim([0, 1])
    ax.set_ylim([0, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.grid(False)

    # Your rectangle/3D plane drawing logic here
    rectangle = np.array([(0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), (0, 0, 0)])
    rectangle = rotate_rectangle(rectangle, x_rotation / 180 * math.pi, y_rotation / 180 * math.pi)
    ax.plot([p[0] for p in rectangle], [p[1] for p in rectangle], [p[2] for p in rectangle], 'b-')

    # Display current rotation values
    ax.text2D(0.05, 0.95, f'X Rotation: {x_rotation}°', transform=ax.transAxes, fontsize=12, color='black')
    ax.text2D(0.05, 0.90, f'Y Rotation: {y_rotation}°', transform=ax.transAxes, fontsize=12, color='black')

    # Create a shadow on the xy-plane (z=0)
    shadow_rectangle = rotate_rectangle(rectangle, 0, 0)  # Shadow is only affected by the x_rotation and y_rotation
    ax.plot([p[0] for p in shadow_rectangle], [p[1] for p in shadow_rectangle], [-1] * len(shadow_rectangle), color='gray', alpha=0.5)

    # Fill the objects
    if (x_rotation == 0 or x_rotation % 90 != 0) and (y_rotation == 0 or y_rotation % 90 != 0):
        ax.plot_trisurf([p[0] for p in rectangle], [p[1] for p in rectangle], [p[2] for p in rectangle], color='green', alpha=0.9)
        ax.plot_trisurf([p[0] for p in shadow_rectangle], [p[1] for p in shadow_rectangle], [-1] * len(shadow_rectangle), color='gray', alpha=0.5)

def update_rotation(frame):
    x_rotation, y_rotation = rotation()
    update_plot(frame, x_rotation, y_rotation)

# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Use FuncAnimation to update the plot at regular intervals
ani = FuncAnimation(fig, update_rotation, interval=500)

# Show the plot
plt.show()
