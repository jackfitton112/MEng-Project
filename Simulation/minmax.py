############################################
# This script computes the minimum and maximum end effector positions for the Tripteron robot.
# Jack Fitton, 2025
############################################

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
ARM_LENGTH =  100 # mm
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2
MIN_SPACING = 10

A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)

print(f"A: {A_MIN} - {A_MAX}")
print(f"B: {B_MIN} - {B_MAX}")
print(f"C: {C_MIN} - {C_MAX}")

# Compute inverse kinematics
def inverse_kinematics(x, y, z, angle_A_C=45, angle_B=45):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)

    A = x - y / np.tan(theta_AC)
    C = x + y / np.tan(theta_AC)
    B = x - z / np.tan(theta_B)

    if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
        return None

    return A, B, C

# Collision check
def check_collision(A, B, C, min_safe_distance=MIN_SPACING + PLATFORM_WIDTH):
    if abs(A - B) < min_safe_distance or abs(A - C) < min_safe_distance or abs(B - C) < min_safe_distance:
        return True
    return False

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


# Compute workspace
points, _, _ = compute_work_area(45, 45, resolution=30)

if len(points) > 0:
    min_x, min_y, min_z = np.min(points, axis=0)
    max_x, max_y, max_z = np.max(points, axis=0)
    
    print(f"X range: {min_x} - {max_x} mm")
    print(f"Y range: {min_y} - {max_y} mm")
    print(f"Z range: {min_z} - {max_z} mm")
else:
    print("No valid points found.")


