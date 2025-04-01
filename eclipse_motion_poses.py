import numpy as np
from spatialmath import SE3, SO3
import roboticstoolbox as rtb
from roboticstoolbox import DHRobot, RevoluteDH
import matplotlib.pyplot as plt

# def generate_ellipse_poses(center, radii, orientation, num_points=50):
#     """Generate SE(3) poses for an elliptical trajectory."""
#     a, b = radii  # Ellipse radii along X and Y
#     xc, yc, zc = center  # Ellipse center
#     theta_vals = np.linspace(0, np.degrees(2*np.pi), num_points)  # Angle values

#     poses = []
#     for theta in theta_vals:
#         x = xc + a * np.cos(theta)
#         y = yc + b * np.sin(theta)
#         z = zc
        
#         # Create the pose using SE3 (xyz position + fixed orientation)
#         T = SE3(x, y, z) * SE3.RPY(orientation, order="xyz", unit="deg")
#         poses.append(T)
    
#     return poses

def generate_tilted_ellipse(center, radii, tilt_angle, num_points=50):
    """Generate SE(3) poses for an elliptical trajectory on a tilted plane."""
    a, b = radii  # Ellipse radii along local X’ and Y’
    xc, yc, zc = center  # Ellipse center
    theta_vals = np.linspace(0, 2*np.pi, num_points)  # Angle values

    # Rotation matrix for tilting around X-axis
    R_tilt = np.array([
        [1, 0, 0],
        [0, np.cos(tilt_angle), -np.sin(tilt_angle)],
        [0, np.sin(tilt_angle), np.cos(tilt_angle)]
    ])

    poses = []
    for theta in theta_vals:
        # Ellipse points in local (flat) coordinates
        x_local = a * np.cos(theta)
        y_local = b * np.sin(theta)
        z_local = 0  # Flat ellipse

        # Apply tilt transformation
        pos_tilted = R_tilt @ np.array([x_local, y_local, z_local]) + np.array([xc, yc, zc])
        pos_tilted = pos_tilted.flatten() + np.array([xc, yc, zc])  # Convert back to 1D
        # TCP orientation: Maintain normal to tilted plane
        # TCP should have its Z-axis perpendicular to the tilted plane
        # orientation = (tilt_angle, 0, theta)  # Align with ellipse direction

        # Construct SE(3) pose
        T = SE3(pos_tilted) * SE3.RPY([tilt_angle, 0, theta], order="xyz", unit="rad")
        poses.append(T)
    
    return poses

 
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0
"""
d: link offset 
a: link length
alpha: link twist
offset: kinematic-joint variable offset
"""
std_dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),  # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),  # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),  # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi / 2),  # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi / 2),  # joint 5
    RevoluteDH(d=d6, a=0, alpha=0),  # joint 6
]

robot = DHRobot(std_dh_table, name="smallRobotArm")
# smRobot.islimit([0, 0, -4, 4, 0, 0])
# Robot kinematics as an elemenary transform sequence
robot.ets()

# Example parameters
# robot = rtb.models.DH.Puma560()  # Replace with your robot model
center = (50, 0, 140)  # Ellipse centered at (300, 0, 200) in mm
# center = (200, 100, 20)  # Ellipse centered at (300, 0, 200) in mm
radii = (100, 50)  # Ellipse radii (X = 100mm, Y = 50mm)
orientation_deg = np.array([0, 0, 0])  # Fixed orientation (90° pitch)
tilt_angle=np.radians(-30)
tile_z= 150  # Z coordinate for the tile plane
poses = generate_tilted_ellipse(center, radii, tilt_angle)

# Store end-effector positions
traj_x, traj_y, traj_z = [], [], []

print("Generated Ellipse Poses:")
for pose in poses:
    print(pose)

joint_angles = []
for T in poses:
    q = robot.ikine_LM(T)  # Use Levenberg-Marquardt solver
    position = T.t
    if q.success:
        joint_angles.append(q.q)  # Store joint angles
        traj_x.append(position[0])
        traj_y.append(position[1])
        traj_z.append(position[2])  
    else:
        print("IK failed for a pose")

from scipy.interpolate import interp1d

# Interpolate joint angles for smooth trajectory
joint_angles = np.array(joint_angles)
num_interpolation = 100  # More points for smooth motion
interp_func = interp1d(np.linspace(0, 1, len(joint_angles)), joint_angles, axis=0, kind="cubic")

smoothed_angles = interp_func(np.linspace(0, 1, num_interpolation))

# Send smoothed joint angles to the robot
# for angles in smoothed_angles:
#     robot.q = angles  # Set robot joints
#     robot.plot(angles)  # Visualize motion (optional)


# Plot in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(traj_x, traj_y, traj_z, marker="o", linestyle="-", label="End-Effector Path")
ax.set_xlabel("X-axis (mm)")
ax.set_ylabel("Y-axis (mm)")
ax.set_zlabel("Z-axis (mm)")
ax.set_title("End-Effector Trajectory")
ax.legend()

plt.show()