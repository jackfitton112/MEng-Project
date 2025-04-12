import numpy as np
from scipy.optimize import minimize


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

# Define knowns
#A = np.array([129, 0, 0])
#B = np.array([165, 0, 0])
#C = np.array([210, 0, 0])
#end = np.array([170, 70, 5])

def find_third_point_triangle(A, C, side_ab=ARM_LENGTH_SMALL, side_bc=ARM_LENGTH_SMALL, prefer_above=True, tolerance=1):
    """
    Given two points A and C, and distances from a third point B to A and C,
    compute the coordinates of B that form a triangle ABC.

    The result lies in the plane perpendicular to vector AC, centered at the midpoint,
    and can be mirrored to lie 'above' or 'below' the AC line based on Z.

    Parameters:
        A (np.array): Point A, shape (3,)
        C (np.array): Point C, shape (3,)
        side_ab (float): Distance from A to B
        side_bc (float): Distance from C to B
        prefer_above (bool): If True, return the B point with Z >= midpoint Z

    Returns:
        np.array: Coordinates of point B (3D)
    """
    # Vector from A to C
    vec_ac = C - A
    dist_ac = np.linalg.norm(vec_ac)

    # Triangle inequality check
    if dist_ac > (side_ab + side_bc):
        print(f"Invalid triangle: AC ({dist_ac}) is longer than AB ({side_ab}) + BC ({side_bc})")
        #raise ValueError("Invalid triangle: AC is longer than AB + BC")

    # Midpoint of AC
    midpoint = (A + C) / 2

    # Height from midpoint to B using Pythagoras
    half_ac = dist_ac / 2
    height = np.sqrt(side_ab**2 - half_ac**2)

    # Unit vector from A to C
    vec_ac_norm = vec_ac / dist_ac

    # Get a perpendicular vector
    temp_vec = np.array([1, 0, 0]) if not np.allclose(vec_ac_norm, [1, 0, 0]) else np.array([0, 1, 0])
    perp_vec = np.cross(vec_ac_norm, temp_vec)
    perp_vec = perp_vec / np.linalg.norm(perp_vec)

    # Candidate point
    B1 = midpoint + perp_vec * height
    B2 = midpoint - perp_vec * height

    #return (B1 or B2) and midpoint

    returnData = {"midpoint": midpoint, "startPoint": A, "endPoint": C}

    # Choose based on Z preference
    if prefer_above:
        if B1[2] >= midpoint[2]:
            returnData["point"] = B1
        else:
            returnData["point"] = B2
        
    else:
        if B1[2] < midpoint[2]:
            returnData["point"] = B1
        else:
            returnData["point"] = B2


    return returnData
        
def generate_perturbed_points_around_axis(triangle_data, angle_offsets_deg=list(range(1, 3))):
    """
    Rotates a triangle point around the AC axis, centered at the triangle's midpoint,
    by specified angle offsets to simulate angular deviation.

    Parameters:
        triangle_data (dict): Output of find_third_point_triangle(), must contain:
                              - 'point': the elbow/joint point (B)
                              - 'midpoint': midpoint between A and C
                              - 'startPoint': point A
                              - 'endPoint': point C
        angle_offsets_deg (list): List of angles in degrees to rotate.

    Returns:
        dict: {
            angle: (rotated_pos_point, rotated_neg_point)
        }
    """
    point = triangle_data["point"]
    midpoint = triangle_data["midpoint"]
    axis = triangle_data["endPoint"] - triangle_data["startPoint"]
    axis = axis / np.linalg.norm(axis)  # Normalize rotation axis
    radius_vector = point - midpoint    # Vector from midpoint to point

    # Get perpendicular vectors in rotation plane
    temp = np.array([1, 0, 0]) if not np.allclose(axis, [1, 0, 0]) else np.array([0, 1, 0])
    perp1 = np.cross(axis, temp)
    perp1 /= np.linalg.norm(perp1)
    perp2 = np.cross(axis, perp1)

    # Project radius vector into the plane
    x = np.dot(radius_vector, perp1)
    y = np.dot(radius_vector, perp2)

    perturbed_points = {}

    for angle in angle_offsets_deg:
        rad = np.radians(angle)

        # Rotation in the perp1/perp2 plane
        x_pos = np.cos(rad) * x - np.sin(rad) * y
        y_pos = np.sin(rad) * x + np.cos(rad) * y

        x_neg = np.cos(-rad) * x - np.sin(-rad) * y
        y_neg = np.sin(-rad) * x + np.cos(-rad) * y

        pos_point = midpoint + x_pos * perp1 + y_pos * perp2
        neg_point = midpoint + x_neg * perp1 + y_neg * perp2

        perturbed_points[angle] = (pos_point, neg_point)

    return perturbed_points

def calcArmJointPosB(start, end, arm_length=ARM_LENGTH_SMALL, start_angle=50):
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


def getElbowLocations(platformA, platformB, platformC, endEffector, tolerance=1, B_angle=50):

    #Platforms A, B and C are all on the X axis, Y and Z are 0
    A = np.array([platformA, 0, 0])
    B = np.array([platformB, 0, 0])
    C = np.array([platformC, 0, 0])

    # endEffector is the end effector position (x, y, z)
    end = np.array(endEffector)

    platforms = [A, B, C]
    elbows = []
    i = 0

    for platform in platforms:
        i += 1


        if i == 2:
            # For platform B, we need to calculate the joint position for B
            elbowPoint = calcArmJointPosB(A, end, arm_length=ARM_LENGTH_SMALL, start_angle=B_angle)
        else:
            elbowPoint = find_third_point_triangle(platform, endEffector, ARM_LENGTH_SMALL, ARM_LENGTH_SMALL, prefer_above=True, tolerance=tolerance)
            elbowPoint = elbowPoint["point"]

        # Check if elbow point is valid
        if np.any(np.isnan(elbowPoint)):
            print("Invalid elbow point")
            return None
        
        elbows.append(elbowPoint)
        
        

        

  
    
    return elbows
            


