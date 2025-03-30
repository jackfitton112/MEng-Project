
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


# Constants
SMALL_ARM_LENGTH = 90  # mm
ARM_LENGTH = SMALL_ARM_LENGTH*2  # mm
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


def inverse_kinematics(x, y, z, angle_A=60, angle_B=50, angle_C=60):
    """
    Computes the carriage positions based on end effector position and platform angles.
    """
    # Convert angles to radians
    theta_A = np.radians(angle_A)
    theta_B = np.radians(angle_B)
    theta_C = np.radians(angle_C)

    # Compute carriage positions
    A = x - y / np.tan(theta_A)
    C = x + y / np.tan(theta_C)
    B = x - z / np.tan(theta_B)

    # Ensure carriages remain within rail limits
    if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
        return None  # Invalid configuration

    return A, B, C

def forward_kinematics(A, B, C, angle_A=60, angle_B=50, angle_C=60):
    theta_A = np.radians(angle_A)
    theta_B = np.radians(angle_B)
    theta_C = np.radians(angle_C)

    # Midpoint defines x
    x = (A + C) / 2

    # Use both A and C to estimate Y
    y_A = (x - A) * np.tan(theta_A)
    y_C = (C - x) * np.tan(theta_C)
    y = (y_A + y_C) / 2  # Average the contributions

    z = (x - B) * np.tan(theta_B)

    return x, y, z



# Collision check function
def check_collision(A, B, C, min_safe_distance=(2*MIN_SPACING) + PLATFORM_WIDTH):
    """
    Ensures minimum spacing between platforms to prevent collisions.
    """
    if abs(A - B) < min_safe_distance or abs(A - C) < min_safe_distance or abs(B - C) < min_safe_distance:
        return True
    return False


def plotLoc(A,B,C, X,Y,Z):
    
    #plot platforms abc and end effector xyz
    fig = plt.figure()

    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(X, Y, Z, c='r', marker='o', label='End Effector')
    ax.scatter(A, 0, 0, c='b', marker='o', label='Platform A')
    ax.scatter(B, 0, 0, c='g', marker='o', label='Platform B')
    ax.scatter(C, 0, 0, c='y', marker='o', label='Platform C')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.legend()

    plt.show()


def calcArmJointPosB(start, end, arm_length=SMALL_ARM_LENGTH, start_angle=50):
    """
    Computes joint position for B (which operates in the X-Z plane at a fixed angle).
    Assumes planar triangle: start -> joint -> end, with fixed base angle = start_angle.
    """

    theta = np.radians(start_angle)

    # Extract X and Z only (we ignore Y for B platform)
    start_xz = np.array([start[0], start[2]])
    end_xz = np.array([end[0], end[2]])

    # Vector and distance in X-Z plane
    vec = end_xz - start_xz
    d = np.linalg.norm(vec)

    if d > 2 * arm_length:
        raise ValueError("Target out of reach.")

    # Law of cosines to get included angle opposite the elbow joint
    # Triangle: sides = [arm, arm, d]
    # angle at start (base) is fixed (start_angle)

    # Solve for angle between vec and first arm
    # Use triangle construction: place first arm at fixed angle from X-axis (like the IK does)
    # First arm goes out from start at 50° above horizontal
    dx = np.cos(theta) * arm_length
    dz = np.sin(theta) * arm_length

    joint_xz = start_xz + np.array([dx, dz])
    joint_position = np.array([joint_xz[0], start[1], joint_xz[1]])  # Fill Y from original start

    return joint_position

def calcArmJointPosAC(start, end, start_angle_deg=60):
    start = np.array(start)  # [112, 0, 0]
    end = np.array(end)      # [170, 100, 5]

    # Calculate vector from start to end
    delta = end - start
    distance = np.linalg.norm(delta)

    # Check if the position is reachable
    if distance > 2 * SMALL_ARM_LENGTH:
        raise ValueError("Target position is out of reach")

    # Convert start angle to radians
    start_angle_rad = np.radians(start_angle_deg)

    # Calculate midpoint distance (distance from start to elbow)
    cos_angle = (2 * SMALL_ARM_LENGTH**2 - distance**2) / (2 * SMALL_ARM_LENGTH**2)
    angle_joint = np.arccos(np.clip(cos_angle, -1.0, 1.0))

    # Calculate direction unit vector
    direction_unit = delta / distance

    # Midpoint (elbow) position
    elbow_distance = SMALL_ARM_LENGTH
    elbow = start + elbow_distance * direction_unit

    # Calculate elbow height
    height_offset = SMALL_ARM_LENGTH * np.sin(angle_joint / 2)
    horizontal_offset = SMALL_ARM_LENGTH * np.cos(angle_joint / 2)

    # Horizontal plane adjustments
    xy_plane_angle = np.arctan2(delta[1], delta[0])
    elbow_x = start[0] + horizontal_offset * np.cos(xy_plane_angle)
    elbow_y = start[1] + horizontal_offset * np.sin(xy_plane_angle)
    elbow_z = start[2] + height_offset

    return np.array([elbow_x, elbow_y, elbow_z])


def simulate_play(A, B, C, start_angle_deg, arm_length, platform, angle_play=1, samples=10):
    """
    Simulates joint play in both platform and elbow joints.
    Returns a list of possible end effector positions given play in angles.
    
    - start: platform position
    - start_angle_deg: nominal base angle
    - direction_plane: 'XY' for A/C, 'XZ' for B
    """

    # Convert angle to radians
    start_angle_rad = np.radians(start_angle_deg)
    min_angle = start_angle_deg - angle_play
    max_angle = start_angle_deg + angle_play

    print(f"Simulating play for {platform} platform from {min_angle}° to {max_angle}°")

    

    # Generate angles to test
    angles = np.linspace(min_angle, max_angle, samples)

    # Store end effector positions
    end_effector_positions = []

    for angle in angles:
        if platform == 'A' or platform == 'C':
            X, Y, Z = forward_kinematics(A, B, C, angle_A=angle, angle_B=50, angle_C=angle)
        elif platform == 'B':
            X, Y, Z = forward_kinematics(A, B, C, angle_A=60, angle_B=50, angle_C=60)

        end_effector_positions.append([X, Y, Z])

    return end_effector_positions


END = [170, 100, 5]
A, B, C = inverse_kinematics(*END, angle_A=60, angle_B=50, angle_C=60)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')



a_joint = calcArmJointPosAC([A, 0, 0], END, start_angle_deg=60)
b_joint = calcArmJointPosB([B, 0, 0], END)
c_joint = calcArmJointPosAC([C, 0, 0], END, start_angle_deg=60)

print(f"A {A}, B {B}, C {C}")
print(f"Joint A: {a_joint} Joint B: {b_joint} Joint C: {c_joint}")

#plot the end effector location and the platform locations (X: 0, Y: A, B, C)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Plot the end effector
ax.scatter(END[0], END[1], END[2], c='r', marker='o', label='End Effector')

# Plot the platforms
ax.scatter(A, 0, 0, c='b', marker='o', label='Platform A')
ax.scatter(B, 0, 0, c='g', marker='o', label='Platform B')
ax.scatter(C, 0, 0, c='y', marker='o', label='Platform C')

# Plot the arm joint
ax.scatter(a_joint[0], a_joint[1], a_joint[2], c='b', marker='o', label='Joint A')
ax.scatter(b_joint[0], b_joint[1], b_joint[2], c='g', marker='o', label='Joint B')
ax.scatter(c_joint[0], c_joint[1], c_joint[2], c='y', marker='o', label='Joint C')


#draw the lines between the platforms and the joints
ax.plot([A, a_joint[0]], [0, a_joint[1]], [0, a_joint[2]], color='b')
ax.plot([B, b_joint[0]], [0, b_joint[1]], [0, b_joint[2]], color='g')
ax.plot([C, c_joint[0]], [0, c_joint[1]], [0, c_joint[2]], color='y')

#draw the lines between the joints and the end effector
ax.plot([a_joint[0], END[0]], [a_joint[1], END[1]], [a_joint[2], END[2]], color='b')
ax.plot([b_joint[0], END[0]], [b_joint[1], END[1]], [b_joint[2], END[2]], color='g')
ax.plot([c_joint[0], END[0]], [c_joint[1], END[1]], [c_joint[2], END[2]], color='y')

#simulate error
all_end_effector_positions = []
a_err = simulate_play(A, B, C, 60, SMALL_ARM_LENGTH, 'A', angle_play=1, samples=100)
b_err = simulate_play(A, B, C, 50, SMALL_ARM_LENGTH, 'B', angle_play=1, samples=100)
c_err = simulate_play(A, B, C, 60, SMALL_ARM_LENGTH, 'C', angle_play=1, samples=100)

all_end_effector_positions.extend(a_err)
all_end_effector_positions.extend(b_err)
all_end_effector_positions.extend(c_err)

#get the min and max values for each axis
min_x = min([pos[0] for pos in all_end_effector_positions])
max_x = max([pos[0] for pos in all_end_effector_positions])

min_y = min([pos[1] for pos in all_end_effector_positions])
max_y = max([pos[1] for pos in all_end_effector_positions])

min_z = min([pos[2] for pos in all_end_effector_positions])
max_z = max([pos[2] for pos in all_end_effector_positions])

xerr = [[END[0] - min_x], [max_x - END[0]]]
yerr = [[END[1] - min_y], [max_y - END[1]]]
zerr = [[END[2] - min_z], [max_z - END[2]]]

print(f"X: {xerr}, Y: {yerr}, Z: {zerr}")

#plot the error bars
ax.errorbar(END[0], END[1], END[2], xerr=xerr, yerr=yerr, zerr=zerr, fmt='o', color='black', label='End Effector Error')


ax.legend()
plt.show()
