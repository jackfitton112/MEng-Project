import numpy as np
import matplotlib.pyplot as plt

# Constants
ARM_LENGTH = 340  # mm
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

def compute_work_area(angle_A_C, angle_B, resolution=20):
    """
    Computes the reachable workspace volume for given angles by testing all (x, y, z)
    positions within a grid and counting how many are reachable.
    """
    x_range = np.linspace(0, RAIL_LENGTH, resolution)
    y_range = np.linspace(0, RAIL_LENGTH / 2, resolution)  # Ensure y is always positive
    z_range = np.linspace(0, RAIL_LENGTH / 2, resolution)  # Ensure z is always positive

    valid_points = []
    
    for x in x_range:
        for y in y_range:
            for z in z_range:
                result = inverse_kinematics(x, y, z, angle_A_C, angle_B)
                if result:
                    A, B, C = result
                    if not check_collision(A, B, C):
                        valid_points.append((x, y, z))

    return np.array(valid_points)

def plot_reachable_area_xy(points):
    """
    Plots the reachable area in the XY plane with the rail drawn along the bottom.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title("Reachable Area in XY Plane")
    ax.scatter(points[:, 0], points[:, 1], s=2, color='blue', label="Reachable Area")
    ax.plot([0, RAIL_LENGTH], [0, 0], color='black', linewidth=2, label="Rail")
    ax.set_xlabel("X Position (mm)")
    ax.set_ylabel("Y Position (mm)")
    ax.legend()
    plt.show()

def plot_reachable_area_xz(points):
    """
    Plots the reachable area in the XZ plane with the rail drawn along the bottom.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title("Reachable Area in XZ Plane")
    ax.scatter(points[:, 0], points[:, 2], s=2, color='blue', label="Reachable Area")
    ax.plot([0, RAIL_LENGTH], [0, 0], color='black', linewidth=2, label="Rail")
    ax.set_xlabel("X Position (mm)")
    ax.set_ylabel("Z Position (mm)")
    ax.legend()
    plt.show()

def plot_reachable_area_yz(points):
    """
    Plots the reachable area in the YZ plane with the rail as a single point.
    """
    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_title("Reachable Area in YZ Plane")
    ax.scatter(points[:, 1], points[:, 2], s=2, color='blue', label="Reachable Area")
    ax.scatter([0], [0], color='black', s=50, label="Rail Location")
    ax.set_xlabel("Y Position (mm)")
    ax.set_ylabel("Z Position (mm)")
    ax.legend()
    plt.show()

# Example usage
angle_A_C, angle_B = 60, 55
reachable_points = compute_work_area(angle_A_C, angle_B)

if reachable_points.size > 0:
    plot_reachable_area_xy(reachable_points)
    plot_reachable_area_xz(reachable_points)
    plot_reachable_area_yz(reachable_points)
