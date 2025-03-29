############################################
# This script is used to predict the uncertainty in the end effector position based on play within the joints in both the axis of rotation but also off plane.
# Jack Fitton, 2025

# This script currently displays no error in the X axis but this is not correct, I will look into this further.
############################################

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constants
ARM_LENGTH = 340  # mm
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

ANGLE_UNCERTAINTY = 5  # degrees

print(f"A: {A_MIN} - {A_MAX}")
print(f"B: {B_MIN} - {B_MAX}")
print(f"C: {C_MIN} - {C_MAX}")

# Inverse kinematics for the nominal configuration
def inverse_kinematics(x, y, z, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    A = x - y / np.tan(theta_AC)
    C = x + y / np.tan(theta_AC)
    B = x - z / np.tan(theta_B)
    if (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX):
        return A, B, C
    else:
        print(f"Invalid IK result for angles A/C={angle_A_C}, B={angle_B} -> A={A:.2f}, B={B:.2f}, C={C:.2f}")
        return None

# Forward kinematics (estimating XYZ from platform positions)
def forward_kinematics(A, B, C, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    x = (A + C) / 2
    y = (x - A) * np.tan(theta_AC)
    z = (x - B) * np.tan(theta_B)
    return x, y, z

# Start and end points
start = np.array([67, 20, 0])
end = np.array([200, 100, 100])



# Get platform positions for end point
ik_end = inverse_kinematics(*end, angle_A_C=60, angle_B=50)
if ik_end is None:
    raise ValueError("End point is unreachable")
A_end, B_end, C_end = ik_end

# Compute FK for end point with uncertainty bounds
x_nom, y_nom, z_nom = forward_kinematics(A_end, B_end, C_end, angle_A_C=60, angle_B=50)
x_min, y_min, z_min = forward_kinematics(A_end, B_end, C_end, angle_A_C=60-ANGLE_UNCERTAINTY, angle_B=50-ANGLE_UNCERTAINTY)
x_max, y_max, z_max = forward_kinematics(A_end, B_end, C_end, angle_A_C=60+ANGLE_UNCERTAINTY, angle_B=50+ANGLE_UNCERTAINTY)

# Plot start, end, and error bars
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Main points
#ax.scatter(start[0], start[1], start[2], c='r', marker='o', label='Start')
ax.scatter(x_nom, y_nom, z_nom, c='g', marker='o', label=f'Nominal End (X={x_nom:.2f}, Y={y_nom:.2f}, Z={z_nom:.2f})')

# Error bars
ax.plot([x_min, x_max], [y_nom, y_nom], [z_nom, z_nom], color='red', label=f'X Error {x_min:.2f} - {x_max:.2f}')
ax.plot([x_nom, x_nom], [y_min, y_max], [z_nom, z_nom], color='blue', label=f'Y Error {y_min:.2f} - {y_max:.2f}')
ax.plot([x_nom, x_nom], [y_nom, y_nom], [z_min, z_max], color='purple', label=f'Z Error {z_min:.2f} - {z_max:.2f}')

#title
ax.set_title(f'Kinematics Uncertainty with {ANGLE_UNCERTAINTY}° Angle Uncertainty at each joint')
ax.legend()
plt.show()
