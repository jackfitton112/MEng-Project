import numpy as np

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

MAX_X_DELTA = ARM_LENGTH

def inverse_kinematics(x, y, z, angle_A_C=60, angle_B=50):
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
    
    if abs(A - C) > MAX_X_DELTA:
        return None

    return A, B, C


def forward_kinematics(A, B, C, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    x = (A + C) / 2
    y = (x - A) * np.tan(theta_AC)
    z = (x - B) * np.tan(theta_B)
    return x, y, z