############################################################################################################
# This script computes the reachable workspace of the Tripteron end effector in each plane and plots it to a 3D graph while calculating the best angles for the platforms.
# Jack Fitton, 2025

# Known Bugs: Currently there is a massive singularity going through the centre of the work envolope, this is not actaully reachable by
# the robot and should be removed from the graph. Currently im not sure how to do this but I will look into it.
############################################################################################################

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
ARM_LENGTH = 180  # mm
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2 # Joint is centered on the platform
MIN_SPACING = 10  # Minimum required spacing between platforms

A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
# B and C platforms are in front of this
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)



print(f"A: {A_MIN} - {A_MAX}")
print(f"B: {B_MIN} - {B_MAX}")
print(f"C: {C_MIN} - {C_MAX}")

# Compute the inverse kinematics of the system
def inverse_kinematics(x, y, z, angle_A_C=45, angle_B=45):
    """
    Computes the carriage positions based on end effector position and platform angles.
    """
    # Convert angles to radians
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)

    # Compute carriage positions
    A = x - y / np.tan(theta_AC)
    C = x + y / np.tan(theta_AC)
    B = x - z / np.tan(theta_B)

    # Ensure carriages remain within rail limits
    if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
        return None  # Invalid configuration

    return A, B, C

# Collision check function
def check_collision(A, B, C, min_safe_distance=MIN_SPACING + PLATFORM_WIDTH):
    """
    Ensures minimum spacing between platforms to prevent collisions.
    """
    if abs(A - B) < min_safe_distance or abs(A - C) < min_safe_distance or abs(B - C) < min_safe_distance:
        return True
    return False


def compute_work_area(angle_A_C, angle_B, resolution=30):
    """
    Computes the reachable workspace volume for given angles by testing all (x, y, z)
    positions within a grid and counting how many are reachable.
    """
    x_range = np.linspace(0, RAIL_LENGTH, resolution)
    y_range = np.linspace(0, RAIL_LENGTH / 2, resolution)  # Ensure y is always positive
    z_range = np.linspace(0, RAIL_LENGTH / 2, resolution)  # Ensure z is always positive

    valid_points = []
    min_positions = []
    max_positions = []
    
    for x in x_range:
        for y in y_range:
            for z in z_range:
                result = inverse_kinematics(x, y, z, angle_A_C, angle_B)
                if result:
                    A, B, C = result
                    if not check_collision(A, B, C):
                        valid_points.append((x, y, z))
                        min_positions.append(min(A, B, C))
                        max_positions.append(max(A, B, C))

    return np.array(valid_points), min_positions, max_positions


A, B, C = inverse_kinematics(170, 70, 5, 60, 50)
print(A, B, C)