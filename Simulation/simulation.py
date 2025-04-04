from getElbow import getElbowLocations
from kinematics import inverse_kinematics, forward_kinematics
from plotScene import plot_scene

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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


X = 170
Y = 100
Z = 0

A, B, C = inverse_kinematics(X,Y,Z)

print("A: ", A)
print("C: ", C)

# Get elbow 
elbowA, elbowC = getElbowLocations(A, B, C, np.array([X,Y,Z]))

print("Elbow A: ", elbowA)
print("Elbow C: ", elbowC)







