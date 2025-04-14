#covert a csv file to python list
import numpy as np

FILENAME = "trip_points.csv"

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


with open(FILENAME, 'r') as file:
    lines = file.readlines()
    coordinates = []

    for line in lines:
        # Split the line by commas and convert to float
        point = list(map(float, line.strip().split(',')))

        point = inverse_kinematics(*point)

        coordinates.append(point)

print(coordinates)