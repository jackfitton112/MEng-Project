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


def getElbowLocations(platformA, platformB, platformC, endEffector):
    side_a = ARM_LENGTH_SMALL  # elbow → end effector
    side_b = ARM_LENGTH_SMALL  # elbow → platform

    platforms = [platformA, platformC]


    def solveForElbow(A, B):
        def objective(p):
            C = np.array(p)
            CA = C - A
            CB = C - B
            return abs(np.dot(CA, CB))  # want 90 deg → dot = 0

        def constraint_ca(p):
            return np.linalg.norm(np.array(p) - A) - side_b

        def constraint_cb(p):
            return np.linalg.norm(np.array(p) - B) - side_a

        constraints = [
            {'type': 'eq', 'fun': constraint_ca},
            {'type': 'eq', 'fun': constraint_cb}
        ]

        initial_guess = (A + B) / 2 + np.array([0, 0, 90])

        #increase max iterations to 1000
        result = minimize(objective, initial_guess, constraints=constraints, method='SLSQP', options={'maxiter': 10000})

        if not result.success:
            print(f"Optimization failed: {result.message}")
        return result.x

    elbowA = solveForElbow(platformA, endEffector)
    #elbowB = calcArmJointPosB(platformB, endEffector, arm_length=ARM_LENGTH_SMALL, start_angle=50)
    elbowC = solveForElbow(platformC, endEffector)

    print("\n--- Elbow Locations ---")
    print(f"Elbow A: {elbowA}")
    print(f"Elbow C: {elbowC}")
    print("-----------------------")

    elbows = [elbowA, elbowC]

    # Print segment lengths to verify constraints
    print("\n--- Segment Lengths ---")
    for i in range(2):
        platform = platforms[i]
        elbow = elbows[i]

        length_platform_to_elbow = np.linalg.norm(elbow - platform)
        length_elbow_to_effector = np.linalg.norm(endEffector - elbow)

        print(f"Arm {i+1}:")
        print(f"  Platform → Elbow:     {length_platform_to_elbow:.4f}")
        print(f"  Elbow → End Effector: {length_elbow_to_effector:.4f}")

    print("\n-----------------------")

    #print the angles between the segments
    print("\n--- Angles Between Segments ---")
    for i in range(2):
        platform = platforms[i]
        elbow = elbows[i]

        length_platform_to_elbow = np.linalg.norm(elbow - platform)
        length_elbow_to_effector = np.linalg.norm(endEffector - elbow)

        cos_angle = (length_platform_to_elbow ** 2 + length_elbow_to_effector ** 2 - side_a ** 2) / (2 * length_platform_to_elbow * length_elbow_to_effector)
        angle = np.arccos(cos_angle) * (180 / np.pi)

        print(f"Arm {i+1}:")
        print(f"  Angle: {angle:.4f} degrees")

    return elbows

