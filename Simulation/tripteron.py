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

class Tripteron:
    def __init__(self, A, B, C):
        self.A, self.B, self.C = sorted([A, B, C])
        
        # Store previous positions
        self.prev_A = self.A
        self.prev_B = self.B
        self.prev_C = self.C
        
        # Store previous time for velocity/acceleration estimation
        self.prev_time = time.time()  # ✅ moved here after A/B/C init

        # Initialize velocities and accelerations
        self.vel_A = self.vel_B = self.vel_C = 0.0
        self.acc_A = self.acc_B = self.acc_C = 0.0

        # PID and controller state
        self.integral = np.zeros(3)
        self.prev_error = np.zeros(3)

        # Geometry setup
        self.Bangle = 50
        self.ACangle = 60
        self.elbowA = self.elbowB = self.elbowC = None

        self.update_kinematics()

    def update_dynamics(self, dt):
        # Compute velocities
        new_vel_A = (self.A - self.prev_A) / dt
        new_vel_B = (self.B - self.prev_B) / dt
        new_vel_C = (self.C - self.prev_C) / dt

        # Clamp to physical motor limits (optional)
        MAX_VEL = 0.02  # 20 mm/s = 0.02 m/s
        new_vel_A = np.clip(new_vel_A, -MAX_VEL, MAX_VEL)
        new_vel_B = np.clip(new_vel_B, -MAX_VEL, MAX_VEL)
        new_vel_C = np.clip(new_vel_C, -MAX_VEL, MAX_VEL)

        # Compute accelerations
        self.acc_A = (new_vel_A - self.vel_A) / dt
        self.acc_B = (new_vel_B - self.vel_B) / dt
        self.acc_C = (new_vel_C - self.vel_C) / dt

        # Update velocities
        self.vel_A, self.vel_B, self.vel_C = new_vel_A, new_vel_B, new_vel_C

        # Update previous positions
        self.prev_A, self.prev_B, self.prev_C = self.A, self.B, self.C


    def update_kinematics(self):
        self.X, self.Y, self.Z = self.forward_kinematics()
        self.end = np.array([self.X, self.Y, self.Z])
        try:
            self.updateElbowLocations()
        except:
            print("error updating elbows")

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

        return x, y, z

    def inverse_kinematics(self, x, y, z):
        A = x - y / np.tan(np.radians(self.ACangle))
        C = x + y / np.tan(np.radians(self.ACangle))
        B = x - z / np.tan(np.radians(self.Bangle))

        if not (A_MIN <= A <= B <= C <= C_MAX):
            print(f"Carriage positions out of bounds - A: {A}, B: {B}, C: {C}")
            return None
        if abs(C - A) > ARM_LENGTH:
            print("Carriage positions exceed arm length - A: {}, C: {}".format(A, C))
            return None
        return A, B, C
    
    def updateElbowLocations(self):
        self.elbowA, self.elbowB, self.elbowC = getElbowLocations(self.A, self.B, self.C, self.end, tolerance=1, B_angle=self.Bangle)

    def get_XYZ(self):
        return self.X, self.Y, self.Z
    
    def get_ABC(self):
        return self.A, self.B, self.C

    def get_elbows(self):
        return self.elbowA, self.elbowB, self.elbowC

    def plot(self):
        self.update_kinematics()

        fig = plt.figure(figsize=(15, 10))
        FONT_SIZE = 20
        
        # === 3D View ===
        ax3d = fig.add_subplot(221, projection='3d')
        ax3d.set_title("3D View")
        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Z")

        def plot_limb_3d(ax, base_x, elbow, color, label_base, label_elbow):
            ax.scatter(base_x, 0, 0, c=color)
            ax.text(base_x, 0, 0, label_base, size=FONT_SIZE)
            ax.scatter(*elbow, c=color)
            ax.text(*elbow, label_elbow, size=FONT_SIZE)
            ax.plot([base_x, elbow[0]], [0, elbow[1]], [0, elbow[2]], c=color)
            ax.plot([elbow[0], self.X], [elbow[1], self.Y], [elbow[2], self.Z], c=color)
            #plot a dashed line from end effector x to x=0
            ax.plot([self.X, self.X], [self.Y, 0], [self.Z, self.Z], ls="--")

        ax3d.scatter(self.X, self.Y, self.Z, c="r")
        ax3d.text(self.X, self.Y, self.Z, "G", size=FONT_SIZE)

        plot_limb_3d(ax3d, self.A, self.elbowA, "b", "A", "D")
        plot_limb_3d(ax3d, self.B, self.elbowB, "g", "B", "E")
        plot_limb_3d(ax3d, self.C, self.elbowC, "y", "C", "F")
        

        # === 2D Top View (XY) ===
        ax_top = fig.add_subplot(222)
        ax_top.set_title("Top View (XY)")
        ax_top.set_xlabel("X")
        ax_top.set_ylabel("Y")
        ax_top.grid(True)
        ax_top.set_aspect('equal')

        def plot_top(ax, base_x, elbow, color, label_base, label_elbow):
            ax.scatter(base_x, 0, c=color)
            ax.text(base_x, 0, label_base, size=FONT_SIZE)
            ax.scatter(elbow[0], elbow[1], c=color)
            ax.text(elbow[0], elbow[1], label_elbow, size=FONT_SIZE)
            ax.plot([base_x, elbow[0]], [0, elbow[1]], c=color)
            ax.plot([elbow[0], self.X], [elbow[1], self.Y], c=color)
            ax.plot([self.X, self.X], [self.Y, 0], ls="--")

        ax_top.scatter(self.X, self.Y, c="r")
        ax_top.text(self.X, self.Y, "G", size=FONT_SIZE)
        plot_top(ax_top, self.A, self.elbowA, "b", "A", "D")
        plot_top(ax_top, self.B, self.elbowB, "g", "B", "E")
        plot_top(ax_top, self.C, self.elbowC, "y", "C", "F")

        # === 2D Front View (XZ) ===
        ax_front = fig.add_subplot(223)
        ax_front.set_title("Front View (XZ)")
        ax_front.set_xlabel("X")
        ax_front.set_ylabel("Z")
        ax_front.grid(True)
        ax_front.set_aspect('equal')

        def plot_front(ax, base_x, elbow, color, label_base, label_elbow):
            ax.scatter(base_x, 0, c=color)
            ax.text(base_x, 0, label_base, size=FONT_SIZE)
            ax.scatter(elbow[0], elbow[2], c=color)
            ax.text(elbow[0], elbow[2], label_elbow, size=FONT_SIZE)
            ax.plot([base_x, elbow[0]], [0, elbow[2]], c=color)
            ax.plot([elbow[0], self.X], [elbow[2], self.Z], c=color)
            #ax.plot([self.X, self.X], [self.Y, 0], [self.Z, self.Z], ls="--")

        ax_front.scatter(self.X, self.Z, c="r")
        #ax_front.text(self.X, self.Z, "G", size=FONT_SIZE)
        plot_front(ax_front, self.A, self.elbowA, "b", "A", "D")
        plot_front(ax_front, self.B, self.elbowB, "g", "B", "E")
        plot_front(ax_front, self.C, self.elbowC, "y", "C", "F")

        # === 2D Left View (YZ) ===
        ax_left = fig.add_subplot(224)
        ax_left.set_title("Left View (YZ)")
        ax_left.set_xlabel("Y")
        ax_left.set_ylabel("Z")
        ax_left.grid(True)
        ax_left.set_aspect('equal')

        def plot_left(ax, elbow, color, label_elbow):
            ax.scatter(elbow[1], elbow[2], c=color)
            ax.text(elbow[1], elbow[2], label_elbow, size=FONT_SIZE)
            ax.plot([0, elbow[1]], [0, elbow[2]], c=color)
            ax.plot([elbow[1], self.Y], [elbow[2], self.Z], c=color)

        ax_left.scatter(self.Y, self.Z, c="r")
        ax_left.text(self.Y, self.Z, "G", size=FONT_SIZE)
        plot_left(ax_left, self.elbowA, "b", "D")
        plot_left(ax_left, self.elbowB, "g", "E")
        plot_left(ax_left, self.elbowC, "y", "F")


        # === Show All ===
        plt.tight_layout()
        plt.show()


    def plot3D(self):
        self.update_kinematics()

        fig = plt.figure(figsize=(15, 15))

        FONT_SIZE = 24
        
        # === 3D View ===
        ax3d = fig.add_subplot(111, projection='3d')
        #ax3d.set_title("3D View")
        ax3d.set_xlabel("X (mm)")
        ax3d.set_ylabel("Y (mm)")
        ax3d.set_zlabel("Z (mm)")

        def plot_limb_3d(ax, base_x, elbow, color, label_base, label_elbow):
            ax.scatter(base_x, 0, 0, c=color)
            ax.text(base_x, 0, 0, label_base, fontsize=FONT_SIZE)
            ax.scatter(*elbow, c=color)
            ax.text(*elbow, label_elbow, fontsize=FONT_SIZE)
            ax.plot([base_x, elbow[0]], [0, elbow[1]], [0, elbow[2]], c=color)
            ax.plot([elbow[0], self.X], [elbow[1], self.Y], [elbow[2], self.Z], c=color)

        ax3d.scatter(self.X, self.Y, self.Z, c="r")
        ax3d.text(self.X, self.Y, self.Z, "G", fontsize=FONT_SIZE)

        plot_limb_3d(ax3d, self.A, self.elbowA, "b", "A", "D")
        plot_limb_3d(ax3d, self.B, self.elbowB, "g", "B", "E")
        plot_limb_3d(ax3d, self.C, self.elbowC, "y", "C", "F")


        fig.savefig("3d_simple.jpg")
        fig.tight_layout()

        plt.show()

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

                    self.X, self.Y, self.Z = self.forward_kinematics()

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
    def __init__(self, tripteron, link_params):
        """
        tripteron: an instance of the Tripteron class
        link_params: a dictionary containing link properties (mass, inertia, length, etc.)
        """
        self.tripteron = tripteron
        self.params = link_params  # Should include limb-specific data for A, B, and C

    def compute_inverse_dynamics(self):
        """
        Compute the required actuator forces (inverse dynamics) for each limb.
        Returns:
            (F_A, F_B, F_C): tuple of required actuator forces at the prismatic joints
        """
        # Extract current state
        A, B, C = self.tripteron.get_ABC()
        elbows = self.tripteron.get_elbows()
        end_effector = self.tripteron.get_XYZ()
        
        # TODO: You will need velocities and accelerations too
        # Example: vel_A, acc_A = self.tripteron.vel_A, self.tripteron.acc_A
        
        # Compute inverse dynamics per limb
        F_A = self.inverse_dynamics_limb(A, 'A')
        F_B = self.inverse_dynamics_limb(B, 'B')
        F_C = self.inverse_dynamics_limb(C, 'C')

        return F_A, F_B, F_C

    def inverse_dynamics_limb(self, q, limb_id):
        link = self.params[limb_id]

        # Get total mass of limb
        m_total = sum(link['masses'])

        # Get acceleration along this prismatic axis
        acc_map = {
            'A': self.tripteron.acc_A,
            'B': self.tripteron.acc_B,
            'C': self.tripteron.acc_C
        }
        a = acc_map[limb_id]

        # Assume prismatic joint moves vertically (gravity included)
        g = 9.81  # m/s²
        F = m_total * a + m_total * g

        return F
    
    def simulate_motion(self, start_xyz, end_xyz, duration, dt=0.01):
        """
        Simulates motion from start to end over a given duration and plots actuator forces.
        start_xyz, end_xyz: 3D tuples or arrays (X, Y, Z)
        duration: total motion time in seconds
        dt: simulation step in seconds
        """
        start_xyz = np.array(start_xyz)
        end_xyz = np.array(end_xyz)
        
        steps = int(duration / dt)
        times = np.linspace(0, duration, steps)
        
        forces_A, forces_B, forces_C = [], [], []

        for t in times:
            alpha = t / duration
            xyz = (1 - alpha) * start_xyz + alpha * end_xyz

            ik_result = self.tripteron.inverse_kinematics(*xyz)
            if ik_result is None:
                print(f"Invalid IK at time {t:.3f}s for position {xyz}")
                forces_A.append(np.nan)
                forces_B.append(np.nan)
                forces_C.append(np.nan)
                continue

            self.tripteron.A, self.tripteron.B, self.tripteron.C = ik_result
            self.tripteron.update_kinematics()
            self.tripteron.update_dynamics(dt)

            F_A, F_B, F_C = self.compute_inverse_dynamics()
            forces_A.append(F_A)
            forces_B.append(F_B)
            forces_C.append(F_C)

            #print forces
            print(f"Time: {t:.3f}s, Forces: A={F_A:.2f}, B={F_B:.2f}, C={F_C:.2f}")

        # Plotting
        plt.figure(figsize=(10, 5))
        plt.plot(times, forces_A, label='Force A', color='red')
        plt.plot(times, forces_B, label='Force B', color='green')
        plt.plot(times, forces_C, label='Force C', color='blue')
        plt.xlabel('Time (s)')
        plt.ylabel('Force (N)')
        plt.title('Prismatic Joint Forces During Motion')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()

    
