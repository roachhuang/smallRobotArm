import numpy as np
import roboticstoolbox as rtb

# from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3


def se3_to_pose_zyz(T: SE3):
        """
        Converts an SE(3) transformation matrix (with ZYZ Euler orientation) to a pose.

        Args:
            T (np.ndarray): The 4x4 SE(3) transformation matrix.

        Returns:
            tuple: A tuple containing the position (list) and ZYZ Euler angles (radians).
        """

        # se3_transform = SE3(T)  # Create an SE3 object from the matrix

        position = T.t  # Extract position as a list
        zyz_euler = T.eul(
            unit="deg"
        )  # Extract ZYZ Euler angles in radians

        return (np.round(position, 2), np.round(zyz_euler, 2))


# Define your DH table parameters
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d4, d6 = 133.0, 117.50, 28.0

dh_table = [
    rtb.RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),  # joint 1
    rtb.RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),  # joint 2
    rtb.RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),  # joint 3
    rtb.RevoluteDH(d=d4, a=0, alpha=np.pi / 2),  # joint 4
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi / 2),  # joint 5
    rtb.RevoluteDH(d=d6, a=0, alpha=0),  # joint 6
]

# Create the robot object with the DH parameters
robot = rtb.DHRobot(dh_table)
print("Reach of the robot:", robot.reach)
print("nbranches", robot.nbranches)
# robot.islimit([0, 0, -4, 4, 0, 0])
print("is spherical:", robot.isspherical())
# Robot kinematics as an elemenary transform sequence
# robot.ets()
# Print the robot's kinematic model




"""
         ti     q1     q2     q3      q4      q5      q6
0   2.0  11.56  73.55 -24.84  180.00  138.72   58.44
1   2.0  11.56  97.43 -45.92  180.00  141.51   58.44
2   0.0  45.87  -5.92 -33.41  105.86   59.99 -119.22
3   0.0  59.29  19.31 -63.19   95.05   49.31 -116.00
4  21.0  -0.00 -79.45  71.98    0.00   97.47   90.00
"""
# Define the end-effector pose (position and orientation)
pose =  (164.5, 0.0, 241.0, 90.0, 0.0, 0.0)
corrected_pose = (
        pose[0],
        pose[1],
        pose[2],
        np.float64(pose[3]),  # Explicitly convert to np.float64
        np.float64(pose[4]),
        np.float64(pose[5]),
    )
T1 = (
        SE3(corrected_pose[0], corrected_pose[1], corrected_pose[2])
        * SE3.Rz(corrected_pose[3], unit='deg')
        * SE3.Ry(corrected_pose[4], unit='deg')
        * SE3.Rz(corrected_pose[5], unit='deg')
    )

T2 = (
    SE3(164.5, 20.0, 241.0)
    * SE3.Rz(90, unit="deg")
    * SE3.Ry(0, unit="deg")
    * SE3.Rz(0, unit="deg")
)
# initial_guess = np.zeros(robot.n) # example initial guess.
# T = SE3(51.98, 0, 218.2) * SE3.Rz(0) * SE3.Ry(0) * SE3.Rz(np.pi)
ik_sol = robot.ikine_LM(T1)
j = np.degrees(ik_sol.q)
print(j)

# folding pose
# j=np.array([21, 51.98, 0, 218.2-52, 90.0, 180.0, -90.0])
# j = np.array([11.56,   73.55,    -24.84,   180.00, 138.72, 58.44])

T = robot.fkine(ik_sol.q)
print(se3_to_pose_zyz(T))
# Define desired end-effector pose
# T_desired = SE3.Tx(0.5) * SE3.Ty(-0.2) * SE3.Rz(np.pi/2)
# Compute inverse kinematics
# p = robot.fkine([0,-78.51,73.90,0,-90.0,0])
