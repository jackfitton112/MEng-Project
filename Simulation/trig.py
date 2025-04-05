import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def verify_triangle(A, B, C, side_ab_target, side_bc_target, tol=1e-3):
    ab = np.linalg.norm(B - A)
    bc = np.linalg.norm(B - C)
    print(f"Verifying triangle:")
    print(f"  |AB| = {ab:.4f} (Target: {side_ab_target})")
    print(f"  |BC| = {bc:.4f} (Target: {side_bc_target})")

    if abs(ab - side_ab_target) > tol:
        print("  ❌ AB side length invalid!")
    else:
        print("  ✅ AB side length valid.")

    if abs(bc - side_bc_target) > tol:
        print("  ❌ BC side length invalid!")
    else:
        print("  ✅ BC side length valid.")
    print()

# Known points
A = np.array([100, 0, 0])
C = np.array([100, 150, 50])

# Side lengths
SIDE_AB = 100
SIDE_BC = 100
SIDE_AC = np.linalg.norm(C - A)

#check if AC is more than the sum of AB and BC as this would be invalid
if SIDE_AC > (SIDE_AB + SIDE_BC):
    print("Invalid triangle: AC is longer than the sum of AB and BC.")
    exit(1)

# Midpoint of AC
midpoint = (A + C) / 2

# Height from midpoint to elbow (Pythagoras)
half_ac = SIDE_AC / 2
height = np.sqrt(SIDE_AB**2 - half_ac**2)

# Vector from A to C
vec_ac = C - A
vec_ac_norm = vec_ac / np.linalg.norm(vec_ac)

# Find a vector perpendicular to AC
# Since AC is not axis-aligned, pick any non-parallel vector to cross with
if np.allclose(vec_ac_norm, [0, 0, 1]):
    temp_vec = np.array([0, 1, 0])
else:
    temp_vec = np.array([1, 0, 0])

perp_vec = np.cross(vec_ac_norm, temp_vec)
perp_vec = perp_vec / np.linalg.norm(perp_vec)

# Elbow point is midpoint + height in perpendicular direction
# Choose the "above" side by checking Z relative to midpoint
B_candidate = midpoint + perp_vec * height

# If B is below the midpoint (in Z), flip it
if B_candidate[2] < midpoint[2]:
    B = midpoint - perp_vec * height
else:
    B = B_candidate

print("Point A:", A)
print("Point C:", C)
print("Computed Point B:", B)
verify_triangle(A, B, C, SIDE_AB, SIDE_BC)

# Plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.scatter(*A, c='r', label='A')
ax.scatter(*C, c='g', label='C')
ax.scatter(*B, c='b', label='B (above AC)')

# Lines
ax.plot([A[0], C[0]], [A[1], C[1]], [A[2], C[2]], 'k-', label='AC')
ax.plot([A[0], B[0]], [A[1], B[1]], [A[2], B[2]], 'r--', label='AB')
ax.plot([C[0], B[0]], [C[1], B[1]], [C[2], B[2]], 'g--', label='BC')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
ax.set_title("Equilateral Triangle in 3D (Elbow Above Line AC)")

plt.show()
