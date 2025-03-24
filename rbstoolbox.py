import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from scipy.spatial.transform import Rotation as R
import roboticstoolbox as rtb
import robotarm_class as myRobot
from math import radians
from spatialmath import SE3, SO3

print(np.__version__)
# Define your DH parameters
a1, a2, a3 = 47.0, 110.0, 26.0
d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0

std_dh_table = [
    RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),  # joint 1
    RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),  # joint 2
    RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),  # joint 3
    RevoluteDH(d=d4, a=0, alpha=np.pi / 2),  # joint 4
    RevoluteDH(d=0, a=0, alpha=-np.pi / 2),  # joint 5
    RevoluteDH(d=d6, a=0, alpha=0),  # joint 6
]

# Define theta offset
th_offset = (0.0, -np.pi / 2, 0.0, 0.0, 0.0, 0.0)

# Create the robot
robot = DHRobot(std_dh_table)
print(robot)
# Define a test transformation matrix T_06 (End-effector pose)
# This should be a 4x4 matrix. You can replace this with a real transformation.
T_06 = np.array([[0, -1, 0, 0.5], [1, 0, 0, 0.3], [0, 0, 1, 0.7], [0, 0, 0, 1]])

# Now call the inverse kinematics method

def pose_to_T(position, zyz_euler_angles):
    """
    Convert a pose (position and ZYZ Euler angles) to a transformation matrix.
    
    Args:
        position (tuple or list): A tuple or list of three elements representing [x, y, z] position.
        zyz_euler_angles (tuple or list): A tuple or list of three elements representing [alpha, beta, gamma] ZYZ Euler angles.
    
    Returns:
        np.ndarray: 4x4 homogeneous transformation matrix.
    """
    # Create an SE3 object from the position and ZYZ Euler angles
    T = SE3(position) * SE3.ZYZ(zyz_euler_angles)
    
    # Return the transformation matrix
    return T.A  # This gives the 4x4 matrix


def se3_to_pose_zyz(T: SE3):
    """
    Converts an SE(3) transformation to a pose (position and ZYZ Euler angles).

    Args:
        se3_transform (SE3): The SE(3) transformation.

    Returns:
        tuple: A tuple containing the position (list) and ZYZ Euler angles (radians).
    """
    position = T.t # Extract position as a list
    
    orientation = T.R
    euler_angles=orientation.ezyz
    euler_angles_deg = np.degrees(euler_angles)
    return (position, euler_angles_deg)
##########################################################

a1, a2, a3 = 47.0, 110.0, 26.0
d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0

dh_tbl = np.array([[np.radians(-90), a1, d1],
                    [0, a2, 0],
                    [np.radians(-90), a3, 0],
                    [np.radians(90), 0, d4],
                    [np.radians(-90), 0, 0],
                    [0, 0, d6]])

    # stdDh.setDhTbl(dh_tbl)
smallRobotArm = myRobot.SmallRbtArm(dh_tbl)


poses0 = np.array(
    [
        # todo: fk j = (0, 0, 0, 0, 0, 0) to see if it is home
        (164.5, 0.0, 241.0, 90.0, 180.0, -90.0),  # Home (x, y, z, ZYZ Euler angles)
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (164.5 + 14.7, 35.4, 141.0, 90.0, 180.0, -90.0),  # j11
        (164.5 + 50.0, 50.0, 141.0, 90.0, 180.0, -90.0),  # j12
        (164.5 + 85.3, 35.4, 141.0, 90.0, 180.0, -90.0),  # j13
        (164.5 + 100.0, 0.0, 141.0, 90.0, 180.0, -90.0),  # j14
        (164.5 + 85.3, -35.4, 141.0, 90.0, 180.0, -90.0),  # j15
        (164.5 + 50.0, -50.0, 141.0, 90.0, 180.0, -90.0),  # j16
        (164.5 + 14.7, -35.4, 141.0, 90.0, 180.0, -90.0),  # j17
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (164.5 + 50.0, 0.0, 141.0, 90.0, 180.0, -90.0),  # j18
        (164.5 + 100.0, 0.0, 141.0, 90.0, 180.0, -90.0),  # j14
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (264.5, 0.0, 141.0, 0.0, 90.0, 0.0),  # j2
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (164.5, 100.0, 141.0, 90.0, 90.0, 0.0),  # j3
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (164.5, -100.0, 141.0, 90.0, -90.0, 0.0),  # j4
        (164.5, 0.0, 141.0, 90.0, 180.0, -90.0),  # j1
        (164.5, 0.0, 241.0, 90.0, 180.0, -90.0),  # Home (x, y, z, ZYZ Euler angles)
    ]
)
np.set_printoptions(precision=4, suppress=True)
J = np.zeros((6,), dtype=float)

# 创建一个旋转对象，绕z轴旋转90度
rot = R.from_euler('zyz', (0, 90, 90), degrees=True)

# 定义一个点在原始坐标系中的位置
point = np.array([1, 0, 0])

# 应用旋转
rotated_point = rot.apply(point)

print(rotated_point)
for pose in poses0:
    
    # T = smallRobotArm.pose2T(pose, seq="ZYZ")
    T= pose_to_T(pose[:3], pose[3:])
    # J = smallRobotArm.ik(T)
    # pos = smallRobotArm.fk(J)    
    # print('my pos: ', pos)
    print('--------------------------------------\n')
    
    ik_solution = robot.ikine_LM(T)
    # q = np.degrees(ik_solution.q)
    T=robot.fkine(ik_solution.q)
    print('pose:', pose)
    print("IK Solution using roboticstoolbox:")
    print('rbt pose', smallRobotArm.T2Pose(T.A))
    # print('rbt pose', smallRobotArm.T2Pose(T))

