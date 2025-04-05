import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from getElbow import getElbowLocations, find_third_point_triangle, generate_perturbed_points_around_axis


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


class Tripteron:
    """
    Class representing a Tripteron mechanism.
    """

    def __init__(self, A, B, C):
        self.init = False
        self.A = A
        self.B = B
        self.C = C
        self.X, self.Y, self.Z = self.forward_kinematics()
        self.end = np.array([self.X, self.Y, self.Z])
        self.elbowA, self.elbowB, self.elbowC = self.get_elbow_locations()
        self.init = True
        #self.elbowA_perturbed, self.elbowB_perturbed, self.elbowC_perturbed = self._calc_perturbed_points()

        if self._calc_arm_length() == False:
            print("Arm lengths are not equal to ARM_LENGTH_SMALL")
            return None

    def forward_kinematics(self):
        """
        Computes the end effector position based on carriage positions.
        """
        x = (self.A + self.C) / 2
        y = (x - self.A) * np.tan(np.radians(60))
        #self.B = self.C - 50
        #z = (x - self.B) * np.tan(np.radians(50))
        z = 0
        return x, y, z
    
    def inverse_kinematics(self, x, y, z):
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
    
    def get_elbow_locations(self):
        endEffector = np.array([self.X, self.Y, self.Z])
        elbow = getElbowLocations(self.A, self.B, self.C, endEffector)

        if self.init == True:
            self.elbowA, self.elbowB, self.elbowC = elbow

        return elbow

    def _calc_arm_length(self):
        #ensure the lengths between the platforms and elbows and the end effector are equal to ARM_LENGTH_SMALL
        # Calculate the lengths of the arms
        armA = np.linalg.norm(self.elbowA - np.array([self.A, 0, 0]))
        armB = np.linalg.norm(self.elbowB - np.array([self.B, 0, 0]))
        armC = np.linalg.norm(self.elbowC - np.array([self.C, 0, 0]))

        armEnd = np.linalg.norm(self.elbowA - self.end)
        armEnd2 = np.linalg.norm(self.elbowB - self.end)
        armEnd3 = np.linalg.norm(self.elbowC - self.end)

        if armA != ARM_LENGTH_SMALL or armB != ARM_LENGTH_SMALL or armC != ARM_LENGTH_SMALL:
            print("Arm lengths are not equal to ARM_LENGTH_SMALL")
            return False
        if armEnd != ARM_LENGTH_SMALL or armEnd2 != ARM_LENGTH_SMALL or armEnd3 != ARM_LENGTH_SMALL:
            print("End effector lengths are not equal to ARM_LENGTH_SMALL")
            return False
        return True  # All arms are equal to ARM_LENGTH_SMALL


        # Check if the lengths are equal to ARM_LENGTH_SMALL

    def printValues(self):
        print(f"Carriage A: {self.A}")
        print(f"Carriage B: {self.B}")
        print(f"Carriage C: {self.C}")
        print(f"End Effector Position: ({self.X}, {self.Y}, {self.Z})")
        print(f"Elbow A Position: {self.elbowA}")
        print(f"Elbow B Position: {self.elbowB}")
        print(f"Elbow C Position: {self.elbowC}")
    
    def plot_error(self, xError=10, yError=10, zError=0, size=20, ignore_B=True, filename=None):
        """
        Plots the error in the end effector position and elbow positions.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        original_end_effector = np.array([self.X, self.Y, self.Z])
        xError = np.random.uniform(-xError, xError, size)
        yError = np.random.uniform(-yError, yError, size)
        zError = np.random.uniform(-zError, zError, size)

        xErrorData = []
        yErrorData = []
        zErrorData = []



        for x in xError:
            self.X = original_end_effector[0] + x
            X = self.X
            self.get_elbow_locations()
            xErrorData.append((X, self.elbowA, self.elbowB, self.elbowC))

        self.X = original_end_effector[0]

        for y in yError:
            self.Y = original_end_effector[1] + y
            Y = self.Y
            self.get_elbow_locations()
            yErrorData.append((Y, self.elbowA, self.elbowB, self.elbowC))

        self.Y = original_end_effector[1]

        for z in zError:
            self.Z = original_end_effector[2] + z
            Z = self.Z
            self.get_elbow_locations()
            zErrorData.append((Z, self.elbowA, self.elbowB, self.elbowC))

        self.Z = original_end_effector[2]
        self.get_elbow_locations()

        


        # Plot the perturbed end effector positions and elbow positions
        for data in xErrorData:
            #end effector
            ax.scatter(data[0], self.Y, self.Z, c='r', marker='x')
            #elbow A
            ax.scatter(data[1][0], data[1][1], data[1][2], c='b', marker='x')
            if not ignore_B:
                ax.scatter(data[2][0], data[2][1], data[2][2], c='b', marker='x')
            #elbow C
            ax.scatter(data[3][0], data[3][1], data[3][2], c='b', marker='x')

        for data in yErrorData:
            #end effector
            ax.scatter(self.X, data[0], self.Z, c='r', marker='x')
            #elbow A
            ax.scatter(data[1][0], data[1][1], data[1][2], c='b', marker='x')
            if not ignore_B:
                ax.scatter(data[2][0], data[2][1], data[2][2], c='b', marker='x')
            #elbow C
            ax.scatter(data[3][0], data[3][1], data[3][2], c='b', marker='x')

        for data in zErrorData:
            #end effector
            ax.scatter(self.X, self.Y, data[0], c='r', marker='x')
            #elbow A
            ax.scatter(data[1][0], data[1][1], data[1][2], c='b', marker='x')
            #elbow B
            if not ignore_B:
                ax.scatter(data[2][0], data[2][1], data[2][2], c='b', marker='x')

            #elbow C
            ax.scatter(data[3][0], data[3][1], data[3][2], c='b', marker='x')

        # Plot the original end effector position
        ax.scatter(original_end_effector[0], original_end_effector[1], original_end_effector[2], c='g', marker='o', label='Original End Effector')

        # plot the original elbow positions
        ax.scatter(self.elbowA[0], self.elbowA[1], self.elbowA[2], c='g', marker='o', label='Original Elbow A')

        if not ignore_B:
            #plot elbow B
            ax.scatter(self.elbowB[0], self.elbowB[1], self.elbowB[2], c='g', marker='o', label='Original Elbow B')

        ax.scatter(self.elbowC[0], self.elbowC[1], self.elbowC[2], c='g', marker='o', label='Original Elbow C')

        #plot platform positions
        ax.scatter(self.A, 0, 0, c='r', marker='o', label='Carriage A')

        if not ignore_B:
            ax.scatter(self.B, 0, 0, c='r', marker='o', label='Carriage B')
        ax.scatter(self.C, 0, 0, c='r', marker='o', label='Carriage C')

        #plot lines to elbows and end effector
        ax.plot([self.A, self.elbowA[0]], [0, self.elbowA[1]], [0, self.elbowA[2]], c='b', linestyle='--')

        if not ignore_B:
            ax.plot([self.B, self.elbowB[0]], [0, self.elbowB[1]], [0, self.elbowB[2]], c='b', linestyle='--')

        ax.plot([self.C, self.elbowC[0]], [0, self.elbowC[1]], [0, self.elbowC[2]], c='b', linestyle='--')

        #plot lines to end effector
        ax.plot([self.elbowA[0], self.X], [self.elbowA[1], self.Y], [self.elbowA[2], self.Z], c='b', linestyle='--')

        if not ignore_B:
            #plot elbow B
            ax.plot([self.elbowB[0], self.X], [self.elbowB[1], self.Y], [self.elbowB[2], self.Z], c='b', linestyle='--')

        ax.plot([self.elbowC[0], self.X], [self.elbowC[1], self.Y], [self.elbowC[2], self.Z], c='b', linestyle='--')



        #plot graph
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')

        plt.title('Error in End Effector Position and Elbow Positions')

        #set figure size
        fig.set_size_inches(10, 10)

        ax.legend()

        if filename:
            plt.savefig(filename)

        plt.show()

        


            




    def plot(self, ignore_B=True, filename=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot the carriages
        ax.scatter(self.A, 0, 0, c='r', marker='o', label='Carriage A')
        ax.scatter(self.C, 0, 0, c='r', marker='o', label='Carriage C')

        # Plot the end effector
        ax.scatter(self.X, self.Y, self.Z, c='g', marker='o', label='End Effector')

        # Plot the elbows
        ax.scatter(self.elbowA[0], self.elbowA[1], self.elbowA[2], c='b', marker='o', label='Elbow A')
        ax.scatter(self.elbowC[0], self.elbowC[1], self.elbowC[2], c='b', marker='o', label='Elbow C')

        # Draw lines between carriages and elbows
        ax.plot([self.A, self.elbowA[0]], [0, self.elbowA[1]], [0, self.elbowA[2]], c='b', linestyle='--')
        ax.plot([self.C, self.elbowC[0]], [0, self.elbowC[1]], [0, self.elbowC[2]], c='b', linestyle='--')

        # Draw line between elbows and end effector
        ax.plot([self.elbowA[0], self.X], [self.elbowA[1], self.Y], [self.elbowA[2], self.Z], c='b', linestyle='--')
        ax.plot([self.elbowC[0], self.X], [self.elbowC[1], self.Y], [self.elbowC[2], self.Z], c='b', linestyle='--')


        #plot perturbed points
        #for angle, points in self.elbowA_perturbed.items():
         #   ax.scatter(points[0][0], points[0][1], points[0][2], c='y', marker='o', label=f'Elbow A Perturbed {angle}°')
         #   ax.scatter(points[1][0], points[1][1], points[1][2], c='y', marker='o')


        if not ignore_B:
            ax.scatter(self.elbowB[0], self.elbowB[1], self.elbowB[2], c='b', marker='o', label='Elbow B')
            ax.plot([self.B, self.elbowB[0]], [0, self.elbowB[1]], [0, self.elbowB[2]], c='b', linestyle='--')
            ax.plot([self.elbowB[0], self.X], [self.elbowB[1], self.Y], [self.elbowB[2], self.Z], c='b', linestyle='--')
            ax.scatter(self.B, 0, 0, c='r', marker='o', label='Carriage B')



        # Set labels
        ax.set_xlabel('X axis')
        ax.set_ylabel('Y axis')
        ax.set_zlabel('Z axis')
        ax.legend()

        fig.set_size_inches(10, 10)

        if filename:
            plt.savefig(filename)

        plt.show()