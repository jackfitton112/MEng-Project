import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from getElbow import getElbowLocations
from matplotlib.patches import Arc
import math

ARM_LENGTH_SMALL = 90  # mm
ARM_LENGTH = 180  # mm total
RAIL_LENGTH = 390  # mm

PLATFORM_WIDTH = 30  # mm
HALF_PLATFORM_WIDTH = PLATFORM_WIDTH / 2  # Joint is centered on the platform
MIN_SPACING = 10  #

# Constants (Adjust as necessary for your system)
A_MIN = MIN_SPACING + HALF_PLATFORM_WIDTH
A_MAX = RAIL_LENGTH - ((2 * PLATFORM_WIDTH) + (2 * MIN_SPACING) + HALF_PLATFORM_WIDTH)

B_MIN = A_MIN + PLATFORM_WIDTH + MIN_SPACING
B_MAX = A_MAX - PLATFORM_WIDTH - MIN_SPACING

C_MIN = B_MIN + PLATFORM_WIDTH + MIN_SPACING
C_MAX = RAIL_LENGTH - (HALF_PLATFORM_WIDTH + MIN_SPACING)

ARM_LENGTH = 150  # Example arm length

MAX_VEL = 100  # mm/s
MAX_ACC = 2  # mm/s^2

class Tripteron:
    def __init__(self, A, B, C):
        self.A, self.B, self.C = A, B, C
        self.X, self.Y, self.Z = None, None, None


        
        # Geometry setup
        self.Bangle = 50
        self.ACangle = 60
        self.elbowA = self.elbowB = self.elbowC = None
        #self.update_kinematics()
        self.time = 0

        self.hysteresis_value = 20

        #velocity and acceleration vars
        self._PrevXYZ = None, None, None
        self._PrevABC = None, None, None
        self._PrevTime = 0 #simulation so time is relative

        self._Xvel = self._Yvel = self._Zvel = 0
        self._Xacc = self._Yacc = self._Zacc = 0


        #dynamics setup
        self.end_mass = 0.5 # kg

        self.forward_kinematics()

        


    def forward_kinematics(self):
        x = (self.A + self.C) / 2
        y = (self.C - self.A) / 2 * np.tan(np.radians(self.ACangle))
        z = (x - self.B) * np.tan(np.radians(self.Bangle))

        if abs(self.A - self.C) > ARM_LENGTH:
            #X delta volated, return None
            return None, None, None
        
        if y > math.sqrt(ARM_LENGTH**2 - ARM_LENGTH_SMALL**2):
            #Y max vilated, return none
            return None, None, None
        
        self.X = x
        self.Y = y
        self.Z = 0 # z

    def inverse_kinematics(self):
        x, y, z = self.X, self.Y, self.Z
        A = x - y / np.tan(np.radians(self.ACangle))
        C = x + y / np.tan(np.radians(self.ACangle))
        B = x - z / np.tan(np.radians(self.Bangle))

        if not (A_MIN <= A <= B <= C <= C_MAX):
            print(f"Carriage positions out of bounds - A: {A}, B: {B}, C: {C}")
            return None
        if abs(C - A) > ARM_LENGTH:
            print("Carriage positions exceed arm length - A: {}, C: {}".format(A, C))
            return None
        
        self.A, self.B, self.C = A, B, C
    
    def updateElbowLocations(self):
        self.elbowA, self.elbowB, self.elbowC = getElbowLocations(self.A, self.B, self.C, self.end, tolerance=1, B_angle=self.Bangle)

    def set_ABC(self, A, B, C):

        self._PrevABC = self.A, self.B, self.C
        self._PrevXYZ = self.X, self.Y, self.Z

        self.A, self.B, self.C = A, B, C
        self.forward_kinematics()

    def set_XYZ(self, X, Y, Z):

        self._PrevABC = self.A, self.B, self.C
        self._PrevXYZ = self.X, self.Y, self.Z

        self.X, self.Y, self.Z = X, Y, Z
        self.inverse_kinematics()

    def get_XYZ(self):
        return self.X, self.Y, self.Z
    
    def get_ABC(self):
        return self.A, self.B, self.C

    def get_elbows(self):
        self.updateElbowLocations()
        return self.elbowA, self.elbowB, self.elbowC

    def apply_hysteresis(self, target_path):
        X_BACKLASH = 5
        Y_BACKLASH = 0.4
        direction = None  # F,B,L,R
        prevDirection = None

        original_path = target_path.copy()
        prevPoint = np.array([self.X, self.Y, self.Z])  # Start position
        new_path = []

        # Ensure the start point is the same for both paths
        new_path.append(original_path[0])

        for index, path in enumerate(original_path):
            # Convert current path point to numpy array
            path = np.array(path)

            X, Y, Z = path
            Y = Y *0.95
            Z = 0

            if index == 0:
                new_path.append(path)
                continue

            X_DIST = path[0] - original_path[index-1][0]
            Y_DIST = path[1] - original_path[index-1][1]

            print(f"X_DIST: {X_DIST}, Y_DIST: {Y_DIST}")

            if X_DIST != 0:
                if X_DIST > 0:
                    X -= X_BACKLASH
                    print("X - Backlash")
                else:
                    X += X_BACKLASH
                    print("X + Backlash")

            if Y_DIST != 0:
                if Y_DIST > 0:
                    Y += Y_BACKLASH
                else:
                    Y -= Y_BACKLASH



            # Add modified path with backlash compensation
            new_path.append([X, Y, Z])

            # Update previous point and direction for next iteration
            prevPoint = [X, Y, Z]
            prevDirection = direction

        return new_path, target_path

    def apply_dynamics(self, target_path):

        #for the dynamics and estimated angle error. we will randomly apply error to the end effectors position
        # we will do this by simulating inaccuracies in the platforms A,B and C positions and thrn getting the new FK values for them
        output = []


        for index in range(len(target_path)-1):

            # get the distance between the two points
            print(f"Index: {index} - {target_path[index]} to {target_path[index+1]}")
            dist = np.subtract(target_path[index+1], target_path[index])  # Corrected order here

            # Calculate the distance (Euclidean)
            dist_magnitude = np.linalg.norm(dist)  # This gives the total distance between points
            
            # Number of points in the line segment
            num_points = int(np.ceil(dist_magnitude))  # Rounding up to ensure the step is at least 1 unit

            mm_path = np.linspace(target_path[index], target_path[index+1], num_points)


            for point in mm_path:

                A, B, C  = self.get_ABC()
                self.set_XYZ(*point)
                A, _, C = self.get_ABC()

                random_A = np.random.uniform(-0.2, 0.2)
                random_B = np.random.uniform(-0.2, 0.2)
                random_C = np.random.uniform(-0.2, 0.2)

                A += random_A
                C += random_C

                self.set_ABC(A, B, C)

                self.forward_kinematics()

                output.append((self.X, self.Y, self.Z))


        output_path = []
        #moving window average all of the points
        for index, path in enumerate(output):
            
            if index == 0:
                continue

            if index == len(output)-1:
                continue

            X = (output[index-1][0] + path[0] + output[index+1][0])
            Y = (output[index-1][1] + path[1] + output[index+1][1])
            Z = 0

            output_path.append((X,Y,Z))



        return output

    def apply_stochastic_path(self, start_point, end_point, num_points=100, min_variation=-1.5, max_variation=1.5):
        """
        Simulate noise in the path, provide a mean, max and min of X, Y positions that have been seen in this path.

        Parameters:
            start_point (tuple): The start point [X1, Y1, Z1]
            end_point (tuple): The end point [X2, Y2, Z2]
            num_points (int): The number of points between start and end to simulate
            min_variation (float): Minimum variation to apply to each point
            max_variation (float): Maximum variation to apply to each point
        """

        # Generate a linear path from start_point to end_point
        path = np.linspace(start_point, end_point, num_points)

        # Apply random variation to each point in the path
        overall_Path = []

        for i in range(num_points):
            noisy_path = []
            for point in path:
                # Apply random noise within the defined range
                X_noise = point[0] + np.random.uniform(min_variation, max_variation)
                Y_noise = point[1] + np.random.uniform(min_variation, max_variation)
                Z_noise = 0  # Assuming Z remains 0 in this case
                
                noisy_path.append((X_noise, Y_noise, Z_noise))
            
            overall_Path.append(noisy_path)

        # Calculate min, max, and mean for each point in the noisy path
        minPath = []
        maxPath = []
        meanPath = []

        for i in range(len(overall_Path[0])):  # For each point in the noisy path

            X_vals = []
            Y_vals = []

            for j in range(len(overall_Path)):  # For each noisy path (generated)
                X_vals.append(overall_Path[j][i][0])
                Y_vals.append(overall_Path[j][i][1])

            # Get the min, max, and mean of X, Y for this point
            minX, maxX = min(X_vals), max(X_vals)
            minY, maxY = min(Y_vals), max(Y_vals)

            meanX = np.mean(X_vals)
            meanY = np.mean(Y_vals)

            minPath.append((minX, minY, 0))  # Store min values
            maxPath.append((maxX, maxY, 0))  # Store max values
            meanPath.append((meanX, meanY, 0))  # Store mean values

        # Print results for inspection
        for i in range(len(minPath)):
            print(f"Point {i+1} - Min: {minPath[i]}, Max: {maxPath[i]}, Mean: {meanPath[i]}")


        return meanPath, minPath, maxPath, path

        


    


    def calculate_velocity(self, targetXYZ=None, targetABC=None, duration=1, dt=0.1):
        startXYZ = self.X, self.Y, self.Z
        startABC = self.A, self.B, self.C

        if targetXYZ:
            points = np.linspace(np.array(startXYZ), np.array(targetXYZ), int(duration/dt))
            kinematic_type = "XYZ"
        elif targetABC:
            points = np.linspace(np.array(startABC), np.array(targetABC), int(duration/dt))
            kinematic_type = "ABC"
        else:
            print("No target XYZ or ABC provided")
            return

        velocities = []
        accelerations = []

        prev_Xvel = prev_Yvel = prev_Zvel = 0.0

        for index, point in enumerate(points):
            if kinematic_type == "XYZ":
                self.set_XYZ(*point)
            elif kinematic_type == "ABC":
                self.set_ABC(*point)

            # Velocity
            Xvel = (self.X - self._PrevXYZ[0]) / dt
            Yvel = (self.Y - self._PrevXYZ[1]) / dt
            Zvel = (self.Z - self._PrevXYZ[2]) / dt

            # Acceleration
            Xacc = (Xvel - prev_Xvel) / dt
            Yacc = (Yvel - prev_Yvel) / dt
            Zacc = (Zvel - prev_Zvel) / dt

            self._Xacc = Xacc
            self._Yacc = Yacc
            self._Zacc = Zacc

            # Store results
            velocities.append((Xvel, Yvel, Zvel))
            accelerations.append((Xacc, Yacc, Zacc))

            # Update previous values
            self._PrevXYZ = (self.X, self.Y, self.Z)
            self._PrevABC = (self.A, self.B, self.C)

            prev_Xvel, prev_Yvel, prev_Zvel = Xvel, Yvel, Zvel



        return velocities, accelerations, points

    def print_jacobian(self):
        """
        Compute and print the Jacobian matrix for the manipulator.
        The Jacobian relates joint velocities to end-effector velocities.
        """
        # Define the angles alpha, beta, and gamma based on the class attributes
        alpha = np.radians(self.ACangle)  # Convert to radians
        beta = np.radians(self.ACangle)   # Assuming same angle for both prismatic joints
        gamma = np.radians(self.Bangle)   # Angle for Z direction

        # Compute the elements of the Jacobian matrix
        J11 = -np.tan(alpha) * np.tan(beta) / (np.tan(alpha) + np.tan(beta))
        J13 = np.tan(alpha) * np.tan(beta) / (np.tan(alpha) + np.tan(beta))

        J21 = np.tan(alpha) / (np.tan(alpha) + np.tan(beta))
        J23 = np.tan(beta) / (np.tan(alpha) + np.tan(beta))

        J31 = np.tan(alpha) / (np.tan(gamma) * (np.tan(alpha) + np.tan(beta)))
        J32 = 1 / np.tan(gamma)
        J33 = np.tan(beta) / (np.tan(gamma) * (np.tan(alpha) + np.tan(beta)))

        # Construct the Jacobian matrix
        J = np.array([
            [J11, 0, J13],
            [J21, 0, J23],
            [J31, J32, J33]
        ])



        return J

    def compute_end_effector_velocity(self, joint_velocities):
        """
        Compute the end-effector velocity given joint velocities using the Jacobian matrix.
        """
        # Get the Jacobian matrix
        J = self.print_jacobian()

        # Convert joint velocities to a vector
        joint_velocity_vector = np.array(joint_velocities)  # [dot_A, dot_B, dot_C]

        # Compute the end-effector velocity: v = J * q
        end_effector_velocity = np.dot(J, joint_velocity_vector)

        return end_effector_velocity 

    def plot2PointsChangeinAngle(self, InitialPos, EndPos, Bangles, plot=True):


        numOfAngles = len(Bangles)

        if numOfAngles > 2:
            cols = math.ceil(numOfAngles / 2)
            rows = 2
        else:
            cols = 1
            rows = 2


        #plot the initial position of A B and C
        fig = plt.figure(figsize=(7.5,15))

        for index, angle in enumerate(Bangles):
            self.Bangle = angle
            self.A, self.B, self.C = InitialPos
            self.X, self.Y, self.Z = self.forward_kinematics()
            self.update_kinematics()
            index += 1

            ax = fig.add_subplot(rows, cols, index, projection="3d")

            ax.scatter(self.A, 0,0, c="r", label="Initial A position")
            ax.scatter(self.B, 0,0, c="g", label="Initial B position")
            ax.scatter(self.C, 0,0, c="b", label="Initial C position")

            ax.text(self.B, 0,0, f"B_init = ({self.B}, 0, 0)")

            #ax.scatter(*self.elbowA, c="r")
            #ax.scatter(*self.elbowB, c="g")
            #ax.scatter(*self.elbowC, c="b")

            ax.scatter(self.X, self.Y, self.Z, c="k", label="Initial End Effector Positon")
            ax.text(self.X, self.Y, self.Z, f"({int(self.X)},{int(self.Y)},{int(self.Z)})" )

            initialXYZ = self.X, self.Y, self.Z

            self.A, self.B, self.C = EndPos
            self.X, self.Y, self.Z = self.forward_kinematics()
            self.update_kinematics()

            ax.scatter(self.A, 0,0, c="r", label="Final A position")
            ax.scatter(self.B, 0,0, c="g", label="Final B position")
            ax.scatter(self.C, 0,0, c="b", label="Final C position")


            ax.text(self.B, 0,0, f"B_end = ({self.B}, 0, 0)")

            #ax.scatter(*self.elbowA, c="r")
            #ax.scatter(*self.elbowB, c="g")
            #ax.scatter(*self.elbowC, c="b")



            ax.scatter(self.X, self.Y, self.Z, c="k", label="Final End Effector Positon")
            ax.text(self.X, self.Y, self.Z, f"({int(self.X)},{int(self.Y)},{int(self.Z)})" )


            Z_Delta = int(abs(initialXYZ[2] - self.Z))
            ax.set_title(f"Platform B angle: {angle} Degrees, Z Delta = {Z_Delta}mm")
            print(f"Platform B angle: {angle} Degrees, Z Delta = {Z_Delta}mm")

        if plot:
            plt.tight_layout()  
            plt.savefig("AngleTests.jpg")
            plt.show()
        
    def plotXYReach(self, angles, plot=True):
        #plot the initial position of A B and C
        fig = plt.figure(figsize=(7.5,15))

        numOfAngles = len(angles)

        if numOfAngles > 2:
            cols = math.ceil(numOfAngles / 2)
            rows = 2
        else:
            cols = 1
            rows = 2

        

        #calc every reachable position in x and y
        A_Pos = np.linspace(A_MIN, A_MAX, 100)

        for index, angle in enumerate(angles):

            index += 1
            self.ACangle = angle
            output = []


            for pos in A_Pos:
                C_min = pos + (2*PLATFORM_WIDTH) + (2*MIN_SPACING)
                C_pos = np.linspace(C_min, C_MAX, 100)

                for posC in C_pos:
                    self.A = pos
                    self.C = posC
                    self.B = posC - (PLATFORM_WIDTH + MIN_SPACING)

                    self.forward_kinematics()

                    if self.X and self.Y:
                        output.append((self.X, self.Y, 0))

            #plot the reachable positions
            ax = fig.add_subplot(rows, cols, index, projection="3d")

            #get min and max X and Y
            minX = 500
            maxX = -500
            minY = 500
            maxY = -500
            
            for pos in output:
                ax.scatter(*pos, c="b")

                if pos[0] < minX: #minX
                    minX = pos[0]
                elif pos[0] > maxX:
                    maxX = pos[0]

                if pos[1] < minY:
                    minY = pos[1]
                elif pos[1] > maxY:
                    maxY = pos[1]


            minX = int(minX)
            minY = int(minY)
            maxX = int(maxX)
            maxY = int(maxY)

            ax.set_title(f"XY Reach for A/C angle: {angle} X = {minX} - {maxX}, Y = {minY} - {maxY}")
            
            #set axis titles
            ax.set_xlabel("X (mm)")
            ax.set_ylabel("Y (mm)")
            ax.set_zlabel("Z (mm)")


            print(f"XY Reach for A/C angle: {angle} X = {minX} - {maxX}, Y = {minY} - {maxY}")

        if plot:
            plt.tight_layout()
            fig.savefig("ACAngles")
            plt.show()


class TripteronDynamics:
    def __init__(self, tripteron):
        self.tripteron = tripteron
        self.StartXYZ = None, None, None
        self.EndXYZ = None, None, None


