from getElbow import getElbowLocations, generate_perturbed_points_around_axis, find_third_point_triangle
from kinematics import inverse_kinematics, forward_kinematics
from plotScene import plot_scene
from tripteron import Tripteron, TripteronDynamics, TripteronHysteresisSimulator

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def inverse_kinematics(x, y, z):
    """
    Computes the carriage positions based on end effector position.
    """
    # Compute carriage positions
    A = x - y / np.tan(np.radians(60))
    C = x + y / np.tan(np.radians(60))
    #B = x - z / np.tan(np.radians(50))
    B = C - 50

    # Ensure carriages remain within rail limits
    if not (A_MIN <= A <= A_MAX and B_MIN <= B <= B_MAX and C_MIN <= C <= C_MAX) or x < 0:
        return None
    
    # Check if the distance between A and C is within the arm length
    if abs(A - C) > ARM_LENGTH: 
        return None
    

    return A, B, C

def forward_kinematics(A, B, C):
    """
    Computes the end effector position based on carriage positions.
    Returns:
        (float, float, float): x, y, z position
    """
    x = (A + C) / 2
    y = (x - A) * np.tan(np.radians(60))
    z = 0  # or compute z based on B
    return float(x), float(y), float(z)

ARM_LENGTH_SMALL = 90  # mm
ARM_LENGTH = 180  # mm total
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2  # Joint is centered on the platform
MIN_SPACING = 10  #

A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)


#set up a tripteron controller
tripteron = Tripteron(100, 130, 160) # represents an XYZ of 150.0, 51.961524227066306, 0.0


initialPos = [100, 110, 160]
endPos = [100, 130, 160]
#angles = [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90]
angles = [45, 60]

link_mass = 0.1 # kg
end_effector_share = 0.1  # each limb carries 1/3 of 0.3kg
link_length = 0.09  # meters

# Inertia approximation for slender rod (about CoM): I = (1/12) * m * L^2
I_link = (1/12) * link_mass * link_length**2 * np.eye(3)
I_effector = (1/12) * end_effector_share * (0.1**2) * np.eye(3)  # assume 10cm wide load

link_params = {
    'A': {
        'masses': [link_mass, link_mass, end_effector_share],
        'lengths': [link_length, link_length, 0],  # no length for platform mass
        'inertias': [I_link, I_link, I_effector],
    },
    'B': {
        'masses': [link_mass, link_mass, end_effector_share],
        'lengths': [link_length, link_length, 0],
        'inertias': [I_link, I_link, I_effector],
    },
    'C': {
        'masses': [link_mass, link_mass, end_effector_share],
        'lengths': [link_length, link_length, 0],
        'inertias': [I_link, I_link, I_effector],
    }
}

#dynamics = TripteronDynamics(tripteron, link_params)

#dyn = TripteronDynamics(tripteron, link_params)

#start = (320, 65, 0)
#end = (65, 65, 0)
#dyn.simulate_motion(start, end, duration=0.2, dt=0.01)

#tripteron.plotXYReach(angles, plot=True)

trip = Tripteron(0.1, 0.15, 0.2)
sim = TripteronHysteresisSimulator(trip, backlash=0.02e-3)

path = [[65, 60, 0], [100, 60, 0], [120, 60, 0], [200, 60, 0],[300, 60, 0],[300, 100, 0],[65, 60, 0]] 

sim.simulate_path(path)


