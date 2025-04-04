# Plot the scene of the simulation
import matplotlib.pyplot as plt
import numpy as np

#3d plotting
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from getElbow import getElbowLocations


# Constants
ARM_LENGTH_SMALL = 90  # mm
ARM_LENGTH = 180  # mm total
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2  # Joint is centered on the platform
MIN_SPACING = 10  # Minimum required spacing between platforms

A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)

def plot_scene(A, B, C, end_effector):

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Set labels
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')
    ax.set_zlabel('Z-axis')

    # A , B and C are all on the X axis, Y and Z are 0

    # Scatter A, B and C points
    ax.scatter(A, 0, 0, c='r', marker='o', label='A')
    ax.scatter(B, 0, 0, c='g', marker='o', label='B')
    ax.scatter(C, 0, 0, c='b', marker='o', label='C')


    # plot the end effector
    ax.scatter(end_effector[0], end_effector[1], end_effector[2], c='y', marker='o', label='End Effector')

    elbowA, elbowC = getElbowLocations(A, B, C, end_effector)

    # plot the elbow points
    ax.scatter(elbowA[0], elbowA[1], elbowA[2], c='c', marker='o', label='Elbow A')
    ax.scatter(elbowC[0], elbowC[1], elbowC[2], c='m', marker='o', label='Elbow C')

    print("Elbow A: ", elbowA)
    print("Elbow C: ", elbowC)


    plt.show()

