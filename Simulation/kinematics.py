import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

# Sweep angles from 30° to 60° to find the best configuration
best_volume = 0
best_angles = (45, 45)
best_points = None
best_min_positions = None
best_max_positions = None

for angle_A_C in range(30, 61, 5):
    for angle_B in range(30, 61, 5):
        points, min_positions, max_positions = compute_work_area(angle_A_C, angle_B)
        volume = len(points)  # Number of valid positions

        if volume > best_volume:
            best_volume = volume
            best_angles = (angle_A_C, angle_B)
            best_points = points
            best_min_positions = min_positions
            best_max_positions = max_positions

# Plot best configuration
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_title(f"Best Work Area for Angles A/C={best_angles[0]}°, B={best_angles[1]}°")

if best_points is not None:
    ax.scatter(best_points[:, 0], best_points[:, 1], best_points[:, 2], s=2, color='blue', label="Reachable Area")
    
    # Plot min and max carriage positions as lines
    ax.plot(best_min_positions, np.zeros_like(best_min_positions), np.zeros_like(best_min_positions), label="Min Rail Movements", color="red", linestyle="dotted")
    ax.plot(best_max_positions, np.zeros_like(best_max_positions), np.zeros_like(best_max_positions), label="Max Rail Movements", color="green", linestyle="dotted")

ax.set_xlabel("X Position (mm)")
ax.set_ylabel("Y Position (mm)")
ax.set_zlabel("Z Position (mm)")
ax.legend()
plt.show()

# Output the best angles and their corresponding workspace volume
best_angles, best_volume
