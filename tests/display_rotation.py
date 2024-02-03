import matplotlib.pyplot as plt
import sys
sys.path.append('..')
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from gsa_components.mpu6050 import Mpu6050
import numpy as np
import math

gyro = Mpu6050()

def rotation_matrix(axis, theta):
    axis = np.asarray(axis)
    axis = axis / np.sqrt(np.dot(axis, axis))
    a = np.cos(theta / 2.0)
    b, c, d = -axis * np.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def rotate_octagon(octagon, x_angle, y_angle):
    center = np.mean(octagon, axis=0)
    centered_octagon = octagon - center
    rotation_matrix_x = rotation_matrix([1, 0, 0], x_angle)
    rotation_matrix_y = rotation_matrix([0, 1, 0], y_angle)
    rotated_octagon = np.dot(centered_octagon, rotation_matrix_x.T)
    rotated_octagon = np.dot(rotated_octagon, rotation_matrix_y.T)
    rotated_octagon += center
    return rotated_octagon

rotation_x = rotation_y = 0

def rotation():
    global rotation_x, rotation_y
    acceleration = gyro.get_scaled_acceleration()
    rotation = gyro.get_rotation(*acceleration)
    rotation_x = rotation[0]
    rotation_y = rotation[1]

    if (rotation_x > 180): rotation_x -= 180
    if (rotation_x < -180): rotation_x += 180
    if (rotation_y > 180): rotation_y -= 180
    if (rotation_y < -180): rotation_y += 180

    if (rotation_x == 90): rotation_x = 91
    if (rotation_x == 270): rotation_x = 271
    if (rotation_y == 90): rotation_y = 91
    if (rotation_y == 270): rotation_y = 271

    rotation = (round(rotation_x), round(rotation_y))
    return rotation

def update_plot(frame, rotation_x, rotation_y):
    ax.cla()
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.grid(False)

    # Your octagon drawing logic here
    octagon = np.array([
        (1, 0, 0), (np.cos(np.pi/4), np.sin(np.pi/4), 0),
        (0, 1, 0), (-np.sin(np.pi/4), np.cos(np.pi/4), 0),
        (-1, 0, 0), (-np.cos(np.pi/4), -np.sin(np.pi/4), 0),
        (0, -1, 0), (np.sin(np.pi/4), -np.cos(np.pi/4), 0),
        (1, 0, 0)
    ])

    octagon = rotate_octagon(octagon, rotation_x / 180 * math.pi, rotation_y / 180 * math.pi)

    # Display current rotation values
    ax.text2D(0.05, 0.95, f'X Rotation: {rotation_x}°', transform=ax.transAxes, fontsize=12, color='black')
    ax.text2D(0.05, 0.90, f'Y Rotation: {rotation_y}°', transform=ax.transAxes, fontsize=12, color='black')

    # Create a shadow on the xy-plane (z=0)
    shadow_octagon = rotate_octagon(octagon, 0, 0)

    # Fill the objects
    ax.plot_trisurf([p[0] for p in octagon], [p[1] for p in octagon], [p[2] for p in octagon], color='blue', alpha=0.9)
    ax.plot_trisurf([p[0] for p in shadow_octagon], [p[1] for p in shadow_octagon], [-1] * len(shadow_octagon), color='gray', alpha=0.5)

def update_rotation(frame):
    rotation_x, rotation_y = rotation()
    update_plot(frame, rotation_x, rotation_y)

# Create the 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Use FuncAnimation to update the plot at regular intervals
ani = FuncAnimation(fig, update_rotation, interval=500)

# Show the plot
plt.show()
