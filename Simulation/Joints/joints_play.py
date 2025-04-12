
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from svgpathtools import svg2paths2
from svgpathtools import Path, Line, CubicBezier, QuadraticBezier, Arc
import os
import cv2
import svgwrite
from svgpathtools import svg2paths2
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from scipy.spatial import distance_matrix
from random import randint

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

MAX_X_DELTA = ARM_LENGTH

START = (130, 60, 0)
MAX_WIDTH = 100
MAX_HEIGHT = 60

#min and max end effector positions
# X_MIN = 70, Y_MIN = 25, Z_MIN = 0
# X_MAX = 300, Y_MAX = 140 , Z_MAX = 60


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
    #B = x - z / np.tan(theta_B)
    B =  (C-A) / 2 + A

    # Ensure carriages remain within rail limits
    #if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
     #   return None  # Invalid configuration
    
    #check spacing between platforms
    #if check_collision(A, B, C):
    #    return None
    
    #if abs(A - C) > MAX_X_DELTA:
      #  return None

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

    #print(f"Joint position: {joint_position}")

    return joint_position

def calcArmJointPosAC(start, end, start_angle_deg=60):
    start = np.array(start)  # [112, 0, 0]
    end = np.array(end)      # [170, 100, 5]

    # Calculate vector from start to end
    delta = end - start
    distance = np.linalg.norm(delta)

    # half the distance
    half_distance = distance / 2

    #calc Z positon with pythagoras where hypothenuse is SMALL_ARM_LENGTH and the other side is half the distance
    z = np.sqrt(SMALL_ARM_LENGTH**2 - half_distance**2)

    # Calculate angle between delta and X-axis
    angle = np.arctan2(delta[1], delta[0])

    # Calculate joint position
    joint_x = start[0] + half_distance * np.cos(angle)
    joint_y = start[1] + half_distance * np.sin(angle)
    joint_z = z

    return np.array([joint_x, joint_y, joint_z])




def calcJointPos(A, B, C, X, Y, Z):

    #function takes the platform positions and the end effector position and calculates the joint positions
    joint_A = calcArmJointPosAC([A, 0, 0], [X, Y, Z], start_angle_deg=60)
    joint_B = calcArmJointPosB([B, 0, 0], [X, Y, Z])
    joint_C = calcArmJointPosAC([C, 0, 0], [X, Y, Z], start_angle_deg=60)

    return joint_A, joint_B, joint_C


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

    #move the platforms +/- a few mm to simulate play in the belts - lets say +/- 3mm
    movement_a = np.linspace(A - 3, A + 3, samples)
    movement_b = np.linspace(B - 3, B + 3, samples)
    movement_c = np.linspace(C - 3, C + 3, samples)

    for i in range(samples):
        if platform == 'A':
            X, Y, Z = forward_kinematics(movement_a[i], B, C, angle_A=angle, angle_B=50, angle_C=angle)
        elif platform == 'B':
            X, Y, Z = forward_kinematics(A, movement_b[i], C, angle_A=60, angle_B=50, angle_C=60)
        elif platform == 'C':
            X, Y, Z = forward_kinematics(A, B, movement_c[i], angle_A=angle, angle_B=50, angle_C=angle)

        end_effector_positions.append([X, Y, Z])


    return end_effector_positions

def plot_end_effector_pos(end_position):
    END = end_position
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


    #print the error values
    print(f"X: {END[0]} +/- {xerr}, Y: {END[1]} +/- {yerr}, Z: {END[2]} +/- {zerr}")




    ax.legend()
    plt.show()

def plot_travel(start_endEffectorPos, finish_endEffectorPos):
    #plot the start and end effector locations
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    #get list of points between the start and end effector positions
    points = np.linspace(start_endEffectorPos, finish_endEffectorPos, 10)

    start_A, start_B, start_C = inverse_kinematics(*start_endEffectorPos, angle_A=60, angle_B=50, angle_C=60)
    finish_A, finish_B, finish_C = inverse_kinematics(*finish_endEffectorPos, angle_A=60, angle_B=50, angle_C=60)

    startJoint_A, startJoint_B, startJoint_C = calcJointPos(start_A, start_B, start_C, *start_endEffectorPos)
    finishJoint_A, finishJoint_B, finishJoint_C = calcJointPos(finish_A, finish_B, finish_C, *finish_endEffectorPos)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Plot the platforms at the start with low transparency
    ax.scatter(start_A, 0, 0, c='b', marker='o', label='Platform A Start', alpha=0.1)
    ax.scatter(start_B, 0, 0, c='g', marker='o', label='Platform B Start', alpha=0.1)
    ax.scatter(start_C, 0, 0, c='y', marker='o', label='Platform C Start', alpha=0.1)

    # plot the start joints with low transparency
    ax.scatter(startJoint_A[0], startJoint_A[1], startJoint_A[2], c='b', marker='o', label='Joint A Start', alpha=0.1)
    ax.scatter(startJoint_B[0], startJoint_B[1], startJoint_B[2], c='g', marker='o', label='Joint B Start', alpha=0.1)
    ax.scatter(startJoint_C[0], startJoint_C[1], startJoint_C[2], c='y', marker='o', label='Joint C Start', alpha=0.1)

    #draw lines between the platforms and the joints
    ax.plot([start_A, startJoint_A[0]], [0, startJoint_A[1]], [0, startJoint_A[2]], color='b', alpha=0.1)
    ax.plot([start_B, startJoint_B[0]], [0, startJoint_B[1]], [0, startJoint_B[2]], color='g', alpha=0.1)
    ax.plot([start_C, startJoint_C[0]], [0, startJoint_C[1]], [0, startJoint_C[2]], color='y', alpha=0.1)

    #draw lines between the joints and the end effector
    ax.plot([startJoint_A[0], start_endEffectorPos[0]], [startJoint_A[1], start_endEffectorPos[1]], [startJoint_A[2], start_endEffectorPos[2]], color='b', alpha=0.1)
    ax.plot([startJoint_B[0], start_endEffectorPos[0]], [startJoint_B[1], start_endEffectorPos[1]], [startJoint_B[2], start_endEffectorPos[2]], color='g', alpha=0.1)
    ax.plot([startJoint_C[0], start_endEffectorPos[0]], [startJoint_C[1], start_endEffectorPos[1]], [startJoint_C[2], start_endEffectorPos[2]], color='y', alpha=0.1)

    # draw the end effector with low transparency
    ax.scatter(start_endEffectorPos[0], start_endEffectorPos[1], start_endEffectorPos[2], c='r', marker='o', label='Start End Effector', alpha=0.1)

    # plot the travel of the end effector
    maxpoints = len(points)
    increaseTransparency = ( 1 / maxpoints) * 0.9
    transparencytopup = 0
    for point in points:
        ax.scatter(point[0], point[1], point[2], c='r', marker='o', alpha=0.1+transparencytopup)
        transparencytopup += increaseTransparency

    #plot end platform and joint positions
    ax.scatter(finish_A, 0, 0, c='b', marker='o', label='Platform A Finish')
    ax.scatter(finish_B, 0, 0, c='g', marker='o', label='Platform B Finish')
    ax.scatter(finish_C, 0, 0, c='y', marker='o', label='Platform C Finish')

    ax.scatter(finishJoint_A[0], finishJoint_A[1], finishJoint_A[2], c='b', marker='o', label='Joint A Finish')
    ax.scatter(finishJoint_B[0], finishJoint_B[1], finishJoint_B[2], c='g', marker='o', label='Joint B Finish')
    ax.scatter(finishJoint_C[0], finishJoint_C[1], finishJoint_C[2], c='y', marker='o', label='Joint C Finish')

    ax.plot([finish_A, finishJoint_A[0]], [0, finishJoint_A[1]], [0, finishJoint_A[2]], color='b')
    ax.plot([finish_B, finishJoint_B[0]], [0, finishJoint_B[1]], [0, finishJoint_B[2]], color='g')
    ax.plot([finish_C, finishJoint_C[0]], [0, finishJoint_C[1]], [0, finishJoint_C[2]], color='y')

    ax.plot([finishJoint_A[0], finish_endEffectorPos[0]], [finishJoint_A[1], finish_endEffectorPos[1]], [finishJoint_A[2], finish_endEffectorPos[2]], color='b')
    ax.plot([finishJoint_B[0], finish_endEffectorPos[0]], [finishJoint_B[1], finish_endEffectorPos[1]], [finishJoint_B[2], finish_endEffectorPos[2]], color='g')
    ax.plot([finishJoint_C[0], finish_endEffectorPos[0]], [finishJoint_C[1], finish_endEffectorPos[1]], [finishJoint_C[2], finish_endEffectorPos[2]], color='y')




    ax.scatter(finish_endEffectorPos[0], finish_endEffectorPos[1], finish_endEffectorPos[2], c='b', marker='o', label='Finish End Effector')

    ax.title.set_text(f'End effector travel from {start_endEffectorPos} to {finish_endEffectorPos}')


    #ax.legend()
    #show and set camera angle
    ax.view_init(elev=20, azim=135)

    plt.show()

def plot_points(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_points = [point[0] for point in points]
    y_points = [point[1] for point in points]

    try:
        z_points = [point[2] for point in points]
    except:
        z_points = [0 for point in points]

    #print(f"X: {x_points}, Y: {y_points}, Z: {z_points}")
    print(f"len X: {len(x_points)}, len Y: {len(y_points)}, len Z: {len(z_points)}")

    ax.plot(x_points, y_points, z_points)


    last_XYZ = x_points[-1], y_points[-1], z_points[-1]
    #get the last point and plot the joint positions
    A, B, C = inverse_kinematics(*last_XYZ, angle_A=60, angle_B=50, angle_C=60)
    joint_A, joint_B, joint_C = calcJointPos(A, B, C, *last_XYZ)

    ax.scatter(A, 0, 0, c='b', marker='o', label='Platform A')
    ax.scatter(B, 0, 0, c='g', marker='o', label='Platform B')
    ax.scatter(C, 0, 0, c='y', marker='o', label='Platform C')

    ax.scatter(joint_A[0], joint_A[1], joint_A[2], c='b', marker='o', label='Joint A')
    ax.scatter(joint_B[0], joint_B[1], joint_B[2], c='g', marker='o', label='Joint B')
    ax.scatter(joint_C[0], joint_C[1], joint_C[2], c='y', marker='o', label='Joint C')

    ax.plot([A, joint_A[0]], [0, joint_A[1]], [0, joint_A[2]], color='b')
    ax.plot([B, joint_B[0]], [0, joint_B[1]], [0, joint_B[2]], color='g')
    ax.plot([C, joint_C[0]], [0, joint_C[1]], [0, joint_C[2]], color='y')

   # ax.plot([joint_A[0], points[-1][0]], [joint_A[1], points[-1][1]], [joint_A[2], points[-1][2]], color='b')
    #ax.plot([joint_B[0], points[-1][0]], [joint_B[1], points[-1][1]], [joint_B[2], points[-1][2]], color='g')
    #ax.plot([joint_C[0], points[-1][0]], [joint_C[1], points[-1][1]], [joint_C[2], points[-1][2]], color='y')



    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()

def sample_path(path, num_points):
    """
    Manually sample evenly spaced points along a path object.
    """
    points = []
    for i in range(num_points):
        t = i / (num_points - 1)
        points.append(path.point(t))
    return points

def solve_tsp(points):

    #points = [(X,Y,Z), ...]

    #start at the start point
    start = START

    # Create a distance matrix
    dist_matrix = distance_matrix(points, points)
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(len(points), 1, 0)
    # Create Routing Model
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback
    def distance_callback(from_index, to_index):

        # Returns the distance between the two nodes
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return dist_matrix[from_node][to_node]
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    )

    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)

    if not solution:
        raise Exception("No solution found!")
    # Extract the route
    index = routing.Start(0)

    tsp_order = []
    while not routing.IsEnd(index):
        tsp_order.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    tsp_order.append(manager.IndexToNode(index))  # Add the end node
    return tsp_order


def svg_to_path(svg_file, num_points_per_path=10, center=(180, 90), z_height=0, lift_height=0, gap_threshold=2.0):
    if not os.path.exists(svg_file):
        raise FileNotFoundError(f"SVG file not found: {svg_file}")

    paths, attributes, svg_attr = svg2paths2(svg_file)

    raw_points = []
    for path in paths:
        sampled = sample_path(path, num_points_per_path)
        raw_points.extend(sampled)

    if not raw_points:
        raise ValueError("No valid points found in SVG.")

    # Convert complex to 2D points
    xy_points = np.array([[pt.real, pt.imag] for pt in raw_points])

    # Compute SVG bounds
    min_xy = np.min(xy_points, axis=0)
    max_xy = np.max(xy_points, axis=0)
    svg_center = (min_xy + max_xy) / 2
    svg_size = max_xy - min_xy

    # Define workspace limits
    workspace_width = 100
    workspace_height = 60
    margin = 0.95

    # Compute scale factor
    scale_x = (workspace_width * margin) / svg_size[0]
    scale_y = (workspace_height * margin) / svg_size[1]
    scale = min(scale_x, scale_y)

    # Transform to center and scale
    transformed_points = (xy_points - svg_center) * scale + np.array(center)

    # Solve TSP or just preserve path order (optional)
    tsp_order = solve_tsp(transformed_points)
    optimized_points = transformed_points[tsp_order]

    end_effector_points = []
    platform_positions = []
    last_point = None

    def add_xyz_and_platform(x, y, z):
        inv = inverse_kinematics(x, y, z)
        if inv:
            A, B, C = inv
            B = int(C) - 50  # correction for your mechanical offset
            platform_positions.append((int(A), int(B), int(C)))
            end_effector_points.append([x, y, z])
            return True
        return False

    # Force starting point
    add_xyz_and_platform(130, 60, lift_height)
    add_xyz_and_platform(130, 60, z_height)
    last_point = np.array([130, 60])

    # Main path
    for pt in optimized_points:
        x, y = pt
        current = np.array([x, y])

        if last_point is not None:
            dist = np.linalg.norm(current - last_point)
            if dist > gap_threshold:
                # Pen up and move above next point
                add_xyz_and_platform(*last_point, lift_height)
                add_xyz_and_platform(x, y, lift_height)

        if add_xyz_and_platform(x, y, z_height):
            last_point = current

    # End gesture
    final_path = [
        (130, 60, z_height),
    ]
    for x, y, z in final_path:
        add_xyz_and_platform(x, y, z)

    # Write output files
    with open('commands.csv', 'w') as f:
        written = set()
        for X, Y, Z in end_effector_points:
            X, Y, Z = int(X), int(Y), int(Z)
            line = f"{X},{Y},{Z}"
            if line not in written:
                f.write(f"{line}\n")
                written.add(line)

    with open('platform_positions.csv', 'w') as f:
        written = set()
        for A, B, C in platform_positions:
            line = f"{A},{B},{C}"
            if line not in written:
                f.write(f"{line}\n")
                written.add(line)

    print(f"Number of unique end effector points: {len(end_effector_points)}")
    print(f"Number of unique platform positions: {len(platform_positions)}")

    return end_effector_points, platform_positions

def png_to_svg(png_file, output_file="output.svg"):
    """
    Convert PNG image to SVG by detecting edges and extracting all contours (inside + outside).
    """
    if not os.path.exists(png_file):
        raise FileNotFoundError(f"PNG file not found: {png_file}")

    # Load image and convert to grayscale
    img = cv2.imread(png_file)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect edges
    edges = cv2.Canny(gray, 100, 200)

    # Save the edges image just for reference/debug
    cv2.imwrite("edges.png", edges)

    # Find all contours (including internal holes)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Create SVG with proper size and viewBox
    height, width = edges.shape
    dwg = svgwrite.Drawing(output_file, size=(f"{width}px", f"{height}px"))
    dwg.viewbox(0, 0, width, height)

    for contour in contours:
        if len(contour) < 3:
            continue  # Skip very small contours

        # Create SVG path string
        path_data = "M " + " L ".join([f"{pt[0][0]},{pt[0][1]}" for pt in contour]) + " Z"
        dwg.add(dwg.path(d=path_data, fill="none", stroke="black", stroke_width=1))

    dwg.save()
    return output_file

def plotPointsIn2D(points):
    x = [point[0] for point in points]
    y = [point[1] for point in points]
    z = [point[2] for point in points]


    plt.xlabel('X')
    plt.ylabel('Y')

    plt.plot(x, y)


    #flip y axis
    plt.gca().invert_yaxis()
    plt.show()

def animatePlot(points):
    #add the arms and IK to the plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x = [point[0] for point in points]
    y = [point[1] for point in points]
    z = [point[2] for point in points]



    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    def update(num, x, y, z, line):
        line.set_data(x[:num], y[:num])
        line.set_3d_properties(z[:num])

        return line,

    #plot the end effector positions
    line, = ax.plot(x, y, z, color='r')

    ani = animation.FuncAnimation(fig, update, len(x), fargs=[x, y, z, line], interval=50, blit=True)

    plt.show()




    plt.show()

def png_vertex_detection(filename):
    # Load image
    img = cv2.imread(filename)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect edges
    edges = cv2.Canny(gray, 100, 200)

    # Find contours
    edges = cv2.dilate(edges, None, iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    #approximate contours to polygons and get the vertices
    approx_contours = [cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True) for cnt in contours]

    points = []

    # Draw contours and vertices
    r = 0
    b = 0
    g = 0
    i = 0
    output = {}

    for cnt in approx_contours:

        output[i] = []

        #random color for each contour
        r = randint(0, 255)
        g = randint(0, 255)
        b = randint(0, 255)

        cv2.drawContours(img, [cnt], -1, (r, g, b), 3)
        for pt in cnt:
            x, y = pt[0]
            output[i].append((x, y, 0))

        i += 1

        
    


    # Save the result
    #output_filename = os.path.splitext(filename)[0] + "_vertices.png"
    #cv2.imwrite(output_filename, img)

    return filename, output


def vertex_to_path(filename, centre=(180, 90), startPoint=(130, 60, 0), maxWidth=100, maxHeight=50):
    file, contours = png_vertex_detection(filename)
    
    # Convert contours to path
    points = []
    ik = []

    # Store the contours in order of visitation
    contour_order = []
    
    # Step 1: Find the first contour closest to the start point (0, 0)
    min_dist = float('inf')
    first_contour = None
    first_point = None
    
    for i, contour in contours.items():
        for pt in contour:
            x, y, z = pt
            dist = np.linalg.norm(np.array([x, y, z]) - np.array(startPoint))
            if dist < min_dist:
                min_dist = dist
                first_contour = i
                first_point = pt
    
    # Add the first contour to the order and set the current position to the first point
    contour_order.append(first_contour)
    current_position = first_point

    # Step 2: Find the closest contour to the current contour, then repeat
    remaining_contours = set(contours.keys()) - {first_contour}  # Remove the first contour from remaining contours

    while remaining_contours:
        min_dist = float('inf')
        closest_contour = None
        closest_point = None
        
        for contour_id in remaining_contours:
            contour = contours[contour_id]
            for pt in contour:
                x, y, z = pt
                dist = np.linalg.norm(np.array([x, y]) - np.array(current_position[:2]))  # Only compare x, y distance
                if dist < min_dist:
                    min_dist = dist
                    closest_contour = contour_id
                    closest_point = pt
        
        # Add the closest contour to the order
        contour_order.append(closest_contour)
        
        # Update current position to the closest point in the newly selected contour
        current_position = closest_point
        
        # Remove the selected contour from remaining contours
        remaining_contours.remove(closest_contour)

    # Now contour_order contains the contours in the correct order
    #print("Contour order: ", contour_order)
    
    # Step 3: Process the points in the contours in the order
    for contour_id in contour_order:
        contour = contours[contour_id]

        # Find the closest point within the current contour to the current position
        min_dist = float('inf')
        closest_point = None
        for pt in contour:
            x, y, z = pt
            dist = np.linalg.norm(np.array([x, y]) - np.array(current_position[:2]))
            if dist < min_dist:
                min_dist = dist
                closest_point = pt
        
        # Reorder the points in this contour, starting from the closest point
        contour_points = []
        closest_index = contour.index(closest_point)
        contour_points.extend(contour[closest_index:])  # Start from the closest point
        contour_points.extend(contour[:closest_index])  # Then take the previous points
        
        # Step 4: Backtrack to the closest point
        for pt in contour_points:
            points.append(pt)
            x, y, z = pt

            # Calculate inverse kinematics for each point (assumed to be defined elsewhere)
            A, B, C = inverse_kinematics(x, y, z)
            ik.append((A, B, C))
        
        # After finishing this contour, backtrack to the closest point in the contour
        points.append(closest_point)
        x, y, z = closest_point
        A, B, C = inverse_kinematics(x, y, z)
        ik.append((A, B, C))

        # Update current position to the closest point for the next contour
        current_position = closest_point

    # Scale points to fit within the workspace, centered around the given centre
    points = np.array(points)

    # Scale x and y to fit within the bounds of startPoint[0] + maxWidth and startPoint[1] + maxHeight
    x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
    y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
    
    points[:, 0] = (points[:, 0] - x_min) / (x_max - x_min) * maxWidth + startPoint[0]
    points[:, 1] = (points[:, 1] - y_min) / (y_max - y_min) * maxHeight + startPoint[1]

    # Scale z: Assuming you want to scale it to fit within 0 and maxHeight
    z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
    points[:, 2] = (points[:, 2] - z_min) / (z_max - z_min) * maxHeight

    # Clip to ensure points stay within bounds
    points[:, 0] = np.clip(points[:, 0], startPoint[0], startPoint[0] + maxWidth)
    points[:, 1] = np.clip(points[:, 1], startPoint[1], startPoint[1] + maxHeight)
    points[:, 2] = np.clip(points[:, 2], 0, maxHeight)

    # Add start point to the start of the points and box around the image
    points = np.insert(points, 0, startPoint, axis=0)
    points = np.insert(points, 0, (startPoint[0]+maxWidth, startPoint[1], 0), axis=0)
    points = np.insert(points, 0, (startPoint[0]+maxWidth, startPoint[1]+maxHeight, 0), axis=0)
    points = np.insert(points, 0, (startPoint[0], startPoint[1]+maxHeight, 0), axis=0)
    points = np.insert(points, 0, startPoint, axis=0)


    # Add the end point to the end of the points
    #points = np.append(points, [[230, 120, 0]], axis=0)
    #points = np.append(points, [[130, 120, 0]], axis=0)
    #points = np.append(points, [[130, 60, 0]], axis=0)



    return file, points, ik, contour_order


def save_path(filenameInput):
    file, points, ik, countourorder = vertex_to_path(filenameInput)

    # Save the points to a CSV file in /points with the same name as the input file but with .csv extension
    if not os.path.exists('points'):
        os.makedirs('points')

    output_filename = os.path.splitext(os.path.basename(filenameInput))[0] + "_points.csv"

    #save ik positions to a csv file
    with open(os.path.join('points', output_filename), 'w') as f:
        for point in points:
            f.write(f"{point[0]},{point[1]},{point[2]}\n")




    #plot graph of points and save in /graphs
    if not os.path.exists('graphs'):
        os.makedirs('graphs')

    plt.figure()
    plt.plot([point[0] for point in points], [point[1] for point in points])
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Path')

    #flip y axis
    plt.gca().invert_yaxis()

    plt.savefig(os.path.join('graphs', os.path.splitext(os.path.basename(filenameInput))[0] + "_path.png"))


    plt.close()




#get all files in /pic directory
files = os.listdir('pics')


for file in files:
        print(f"Processing pics/{file}")
        file = os.path.join('pics', file)

        save_path(file)
