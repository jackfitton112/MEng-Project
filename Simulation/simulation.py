from getElbow import getElbowLocations, generate_perturbed_points_around_axis, find_third_point_triangle
from kinematics import inverse_kinematics, forward_kinematics
from plotScene import plot_scene
from tripteron import Tripteron, TripteronControl

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def inverse_kinematics(x, y, z):
    """
    Computes the carriage positions based on end effector position.
    """
    # Compute carriage positions
    A = x - y / np.tan(np.radians(60))
    C = x + y / np.tan(np.radians(60))
    #B = x - z / np.tan(np.radians(50))
    B =  (C-A) / 2 + A


    # Ensure carriages remain within rail limits
    if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
        return None
    
    # Check if the distance between A and C is within the arm length
    if abs(A - C) > ARM_LENGTH: 
        return None
    

    return A, B, C

def forward_kinematics(A, B, C):
    """
    Computes the end effector position based on carriage positions.
    Returns:
        (float, float, float): x, y, z position
    """
    x = (A + C) / 2
    y = (x - A) * np.tan(np.radians(60))
    z = 0  # or compute z based on B
    return float(x), float(y), float(z)

coordinates = [
    [130, 60, 0],
    [230, 60, 0],
    [230, 110, 0],
    [210, 110, 0],
    [210, 70, 0],
    [190, 70, 0],
    [190, 110, 0],
    [170, 110, 0],
    [170, 70, 0],
    [150, 70, 0],
    [150, 110, 0],
    [130, 110, 0],
    [130, 60, 0]
]

ARM_LENGTH_SMALL = 90  # mm
ARM_LENGTH = 180  # mm total
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2  # Joint is centered on the platform
MIN_SPACING = 10  #

A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)

startpos = [130,60,0]
endpos = [230, 120, 0]
tripteron = Tripteron(*startpos)
controller = TripteronControl(tripteron)

def hys():
    hys, og = tripteron.apply_hysteresis(coordinates)
    dyn = tripteron.apply_dynamics(hys)
    meanpos, minpos, maxpos, ideal = tripteron.apply_stochastic_path(startpos, endpos)

    og = np.array(og)
    hys = np.array(hys)
    dyn = np.array(dyn)

    ideal = np.array(ideal)
    mean = np.array(meanpos)
    min = np.array(minpos)
    max = np.array(maxpos)



    #plot og path and hys path
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    #plot hys and og
    ax.plot(og[:, 0], og[:, 1], og[:, 2], label='Original Path', color='orange')
    ax.plot(hys[:, 0], hys[:, 1], hys[:, 2], label='Hysteresis Path', color='blue')

    #plot ideal path, mean path and min and max as error bars / distribution
    #ax.plot(mean[:, 0], mean[:, 1], mean[:, 2], label='Mean Path', color='blue', linestyle="dashdot")
    #ax.plot(min[:, 0], min[:, 1], min[:, 2], label='Min Path', color='green', linestyle="dotted")
    #ax.plot(max[:, 0], max[:, 1], max[:, 2], label='Max Path', color='red', linestyle="dotted")
    #ax.plot(ideal[:, 0], ideal[:, 1], ideal[:, 2], label='Ideal Path', color='orange')
    #plot original path and hys path

    #add labels
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")

    #ax.set_title("Stochastic Analysis (10,000 iter) from (130, 60, 0) to (230, 120, 0) - Zoomed")


    ax.legend()

    ax.view_init(elev=90, azim=90, roll=0)

    fig.savefig('hys_path.png', dpi=300)

    plt.show()


def controllerrun():
    startPos = [130, 60, 0]
    endPos = [230, 120, 0]

    tripteron = Tripteron(*startpos)
    controllerlocal = TripteronControl(tripteron)

    controllerlocal.setStartPos(startPos)
    controllerlocal.setEndPos(endPos)

    times, positions, velocities = controllerlocal.PIDController()

    #plot the startpoint in green, the end point in red and the positions in blue
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(121, projection='3d')
    ax1 = fig.add_subplot(122)


    #plot the positions
    for position in positions:
        ax.scatter(position[0], position[1], position[2], color='blue')
                   
    ax.scatter(positions[0][0], positions[0][1], positions[0][2], color='blue', label='Positions')

    #plot start and end point
    ax.scatter(startPos[0], startPos[1], startPos[2], color='green', label='Start Point')
    ax.scatter(endPos[0], endPos[1], endPos[2], color='red', label='End Point')

    #plot velocity over time on ax1
    ax1.plot(times, velocities, color='blue')
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Velocity (mm/s)")
    ax1.set_title("Velocity over Time")

    plt.legend()
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("Linear Controller Path")
    fig.savefig('lin_controller_path.png', dpi=300)
    plt.show()



controllerrun()