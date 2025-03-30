import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import os

# Constants
ARM_LENGTH = 180 # mm
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

min_position = (0, 0, 0)

print(f"A: {A_MIN} - {A_MAX}")
print(f"B: {B_MIN} - {B_MAX}")
print(f"C: {C_MIN} - {C_MAX}")

def forward_kinematics(A, B, C, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    x = (A + C) / 2
    y = (x - A) * np.tan(theta_AC)
    z = (x - B) * np.tan(theta_B)
    return x, y, z

def inverse_kinematicsXY(x, y, z, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    A = x - y / np.tan(theta_AC)
    C = x + y / np.tan(theta_AC)
    #B = x - z / np.tan(theta_B)
    #if (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX):
    #    return A, B, C

    #TEST: ignore B
    if (A_MIN <= A <= A_MAX and C_MIN <= C <= C_MAX):
        return A, C
    else:
        #print(f"Invalid IK result for angles A/C={angle_A_C}, B={angle_B} -> A={A:.2f}, B={B:.2f}, C={C:.2f}")
        return None

def inverse_kinematics(x, y, z, angle_A_C=60, angle_B=50):
    theta_AC = np.radians(angle_A_C)
    theta_B = np.radians(angle_B)
    A = x - y / np.tan(theta_AC)
    C = x + y / np.tan(theta_AC)
    B = x - z / np.tan(theta_B)
    if (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX):
        return A, B, C

    #TEST: ignore C
    #if (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX):
     #   return A, B

    else:
        #print(f"Invalid IK result for angles A/C={angle_A_C}, B={angle_B} -> A={A:.2f}, B={B:.2f}, C={C:.2f}")
        return None


def check_collision(A, B, C, min_safe_distance=MIN_SPACING):
    """
    Ensures minimum spacing between platforms to prevent collisions.
    """
    if abs(A - B) < min_safe_distance or abs(A - C) < min_safe_distance or abs(B - C) < min_safe_distance:
        return True
    return False

def min_required_arm_length(A, B, C, angle_A_C=60, angle_B=50):
    """
    Returns the minimum required arm lengths for lateral and vertical motion.
    
    For the lateral arm:
      - The forward kinematics give y = (x-A) * tan(theta_AC) with x = (A+C)/2.
      - The arm must cover the hypotenuse of the right triangle with adjacent (x-A)
        and opposite (x-A)*tan(theta_AC), so:
        L_y = (x-A) / cos(theta_AC) = ((C-A)/2) / cos(theta_AC).
    
    For the vertical arm:
      - Similarly, z = (x-B) * tan(theta_B) and the required arm length is:
        L_z = (x-B) / cos(theta_B) = (((A+C)/2 - B)) / cos(theta_B).
    
    Returns:
      (L_y, L_z): The minimum required arm lengths for lateral and vertical arms, respectively.
    """
    theta_AC_rad = np.radians(angle_A_C)
    theta_B_rad = np.radians(angle_B)
    
    # Compute x from the forward kinematics: x = (A+C)/2.
    x = (A + C) / 2.0
    
    # Lateral arm: based on the triangle with side (x-A)
    L_y = ((C - A) / 2.0) / np.cos(theta_AC_rad)
    
    # Vertical arm: based on the triangle with side (x-B)
    L_z = (x - B) / np.cos(theta_B_rad)
    
    return L_y, L_z 


#for every given position of the platofrms, calc the end effector position
xmax = forward_kinematics(A_MAX, B_MAX, C_MAX)[0]
xmin = forward_kinematics(A_MIN, B_MIN, C_MIN)[0]



#max reach is going to be both A and C arms at their max reach, we can pythagorean theorem to find the max reach
max_reach = np.sqrt(ARM_LENGTH**2 - (ARM_LENGTH/2)**2)
print(f"Max reach: {max_reach}")


y_min = -500 # this is just a placeholder
y_max = max_reach
#as A and C are fixed at 60 degrees, the max delta in x is going to be whatever arm length is as it will form and equilateral triangle
max_delta_x = ARM_LENGTH

#save the points to a file
def save_points(points, invalid_points, filename):
    with open(filename+"Valid.csv", "w") as f:
        for point in points:
            f.write(f"{point[0]},{point[1]}\n")
    with open(filename+"Invalid.csv", "w") as f:
        for point in invalid_points:
            f.write(f"{point[0]},{point[1]}\n")

def load_points(filename):
    points = []
    invalid_points = []
    with open(filename+"Valid.csv", "r") as f:
        for line in f:
            x, y = line.strip().split(",")
            points.append((x, y))
    with open(filename+"Invalid.csv", "r") as f:
        for line in f:
            x, y = line.strip().split(",")
            invalid_points.append((x, y))

    return points, invalid_points


        
def get_y_points():
    #calc list of valid Y points
    points = []
    invalid_points = []

    #so as it loops through, C gets closer to A, for now we are ignoring the B arm
    for A_Platform in range (int(A_MIN), int(A_MAX), 10):
        New_A_Min = A_Platform + HALF_PLATFORM_WIDTH + MIN_SPACING + PLATFORM_WIDTH + MIN_SPACING #simulates a B platform
        for C_Platform in range (int(New_A_Min), int(C_MAX), 10):

            X_pos = (A_Platform + C_Platform) / 2
            Y_pos = (X_pos - A_Platform) * np.tan(np.radians(60))

            #if the X delta is more than can be reached by the arm, add to invalid points
            if (C_Platform - A_Platform) > max_delta_x:
                invalid_points.append((int(X_pos), int(Y_pos)))
                continue

            #if the Y position is within the max reach of the arm
            if Y_pos < y_max and Y_pos > y_min:
                #points.append((X_pos, Y_pos))
                #run inverse kinematics to see if the platform positions are valid
                #if it is valid, add to the list of valid points
                pos = inverse_kinematics(int(X_pos), int(Y_pos), 0)
                if pos is not None:
                    points.append((X_pos, Y_pos))
                else:
                    invalid_points.append((int(X_pos), int(Y_pos)))

    #check for and remove duplicates
    points = list(set(points))
    #invalid_points = list(set(invalid_points))   



    return points#, invalid_points

def get_z_points():
    #as platform A moves towards C, B must also move towards C as it is in between A and C
    #start A B and C at their minimum positions, move C away until the x delta is violated
    #then move B towards C until it collides with C
    #then move A towards B until it collides with B
    #then reverse the process and start all platforms at their max positions and move A towards its min position until the x delta is violated
    #then move B towards A until it collides with A
    #then move C towards B until it collides with B
    #add all these points to the list of valid points
    points = []
    invalid_points = []
    count = 0
    min_z = 0
    max_z = 0

    for A_Platform in range (int(A_MIN), int(A_MAX), 5):
        New_B_Min = A_Platform + PLATFORM_WIDTH + MIN_SPACING
        for B_Platform in range(int(New_B_Min), int(B_MAX), 5):
            New_C_Min = B_Platform + PLATFORM_WIDTH + MIN_SPACING
            for C_Platform in range(int(New_C_Min), int(C_MAX), 5):
                
                X, Y, Z = forward_kinematics(A_Platform, B_Platform, C_Platform)
                


                if (C_Platform - A_Platform) > max_delta_x:
                    invalid_points.append((int(X), int(Y), int(Z)))
                    #print(f"Invalid X delta: A: {A_Platform}, C: {C_Platform}")
                    continue

                points.append((int(X), int(Y), int(Z)))
                    
    print(f"Total points: {count}")
    print(f"Min Z: {min_z}")
    print(f"Max Z: {max_z}")

    xy_map = {}

    for point in points:
        x, y, z = point

        key = (x, y)

        if key not in xy_map:
            # First time seeing this (x, y), set initial min and max z
            xy_map[key] = [z, z]
        else:
            # Update min z
            if z < xy_map[key][0]:
                if z > 0:
                    xy_map[key][0] = z
                else:
                    xy_map[key][0] = 0
            # Update max z
            if z > xy_map[key][1]:
                if z > 0:
                    xy_map[key][1] = z
                else:
                    xy_map[key][1] = 0

    return xy_map
        


def plot_points(points, invalid_points, title):




    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title(title)
    ax.set_xlabel("Y")
    ax.set_ylabel("Z")
    for point in points:
        ax.scatter(point[0], abs(point[1]), c='g')
    #for point in invalid_points:
     #   ax.scatter(point[0], point[1], c='r')
    fig.savefig("Z_points.png")
    plt.show()

def plot_points3D(points, invalid_points, title):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(xmin, xmax)
    ax.set_ylim(y_min, y_max)
    for point in points:
        ax.scatter(point[0], point[1], point[2], c='g')
    for point in invalid_points:
        ax.scatter(point[0], point[1], point[2], c='r')
    plt.show()



def armlengthtester(arm_length):

    global ARM_LENGTH
    global max_reach
    global max_delta_x

    ARM_LENGTH = arm_length
    max_reach = np.sqrt(ARM_LENGTH**2 - (ARM_LENGTH/2)**2)
    max_delta_x = ARM_LENGTH

    xy_map  = get_z_points()
    y_points = get_y_points()

    #for valid y points, find the min and max z points and plot them


    #3d plot of all points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    for point in y_points:
        x, y = point
        x = int(x)
        y = int(y)
        z_min = 0
        z_max = 0
        
        if (x, y) in xy_map:
            z_min, z_max = xy_map[(x, y)]
        #else:
            #print(f"Invalid point: {x}, {y}")

        ax.scatter(x, y, z_min, c='r')
        ax.scatter(x, y, z_max, c='b')
        #print(f"X: {x}, Y: {y}, Z min: {z_min}, Z max: {z_max}")

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.title.set_text(f"Arm Length {arm_length}mm")



    #plt.show()
    plt.savefig(f"Arm Length {arm_length}.png")

    #calculate volume for each arm length
    volume = 0
    for key in xy_map:
        z_min, z_max = xy_map[key]
        volume += z_max - z_min

    print(f"Volume: {volume} max reach {max_reach} for arm length {arm_length}")

armlengthtester(100)
armlengthtester(150)
armlengthtester(180)
armlengthtester(200)
armlengthtester(250)
armlengthtester(300)
armlengthtester(340)
