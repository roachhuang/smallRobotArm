
"""
If there is a gripper attached to the z-axis of frame 6, you can still use the T06 matrix as input to the inverse kinematics algorithm, but you need to make sure that the transformation matrix accounts for the additional offset of the gripper.

Here are the general steps you can follow:

Define the transformation matrix T06 that relates the end-effector to the robot base, taking into account the additional offset of the gripper along the z-axis of frame 6.
The transformation matrix should include the gripper's offset from the end-effector in the z-direction.

Use the T06 matrix as input to the inverse kinematics algorithm to compute the required joint angles for the robot to achieve the desired end-effector pose.
The inverse kinematics algorithm will use the T06 matrix to calculate the position and orientation of the end-effector relative to the robot base,
including the offset of the gripper.

Once you have the joint angles, you can use them to drive the robot to the desired configuration, including the gripper's offset.

Finally, you can activate the gripper to close or open it as required for the task.

Note that the gripper's offset should be taken into account when defining the transformation matrix T06, as this will affect the position and orientation of the end-effector relative to the robot base.
By including the gripper's offset in the T06 matrix, you can ensure that the inverse kinematics algorithm computes the joint angles required to achieve the desired end-effector pose, including the gripper's offset.
"""

# import std_dh_tbl as stdDh
import numpy as np
# from ik_smallrobotarm import InverseK
from math import radians
import robotarm_class as robot
# import smallrobotarm_fk as sra

def main():
    r1, r2, r3 = 47.0, 110.0, 26.0
    d1, d3, d4, d6 = 133.0, 0.0, 117.50, 28.0

    dh_tbl = np.array([[radians(-90), r1, d1],
                       [0, r2, 0],
                       [radians(-90), r3, 0],
                       [radians(90), 0, d4],
                       [radians(-90), 0, 0],
                       [0, 0, d6]])

    # stdDh.setDhTbl(dh_tbl)
    smallRobotArm = robot.RobotArm(6, dh_tbl)

    # p = (x, y, z, ZYZ euler angles)
    Poses = np.array([[164.5, 0.0, 241.0, 90.0, 180.0, -90],
                 [164.5, 0.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0],
                 [164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0],
                 [164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0]])
    '''
    P = np.array([
        [164.5, 0.0, 200.0, 90.0, 180.0, -90.0],
        [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0],
        [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0],
        [264.5, 0.0, 141.0, 0.0, 90.0, 0.0]
    ])
    '''
    np.set_printoptions(precision=4, suppress=True)
    J = np.zeros((6,), dtype=float)
    for pose in Poses:
        #J = sra.inverse_kinematics(p1[0], p1[1], p1[2], p1[3], p1[4], p1[5])
        J = smallRobotArm.ik(pose)
        print('2: ', J)

        pos = smallRobotArm.fk(J)
        #print('fk:', pos.round(2))
    print('--------------------------------------\n')
    # power on pose
    pos = smallRobotArm.fk([0.0, -78.51, 73.90, 0.0, -90, 0.0])
    # home pose
    # pos = smallRobotArm.fk([0.0, 0.0, 0.0, 0.0, 90.0, 0.0])
    print(pos)
    j = smallRobotArm.ik(pos)
    print(j)
    print('--------------------------------------\n')

    # [x, y, z, ZYZ Euler angles]
    Xhome = [164.5, 0.0, 241.0, 90.0, 180.0, -90.0]
    X1 = [164.5, 0.0, 141.0, 90.0, 180.0, -90.0]
    X11 = [164.5+14.7, 35.4, 141.0, 90.0, 180.0, -90.0]
    X12 = [164.5+50.0, 50.0, 141.0, 90.0, 180.0, -90.0]
    X13 = [164.5+85.3, 35.4, 141.0, 90.0, 180.0, -90.0]
    X14 = [164.5+100.0, 0.0, 141.0, 90.0, 180.0, -90.0]
    X15 = [164.5+85.3, -35.4, 141.0, 90.0, 180.0, -90.0]
    X16 = [164.5+50.0, -50.0, 141.0, 90.0, 180.0, -90.0]
    X17 = [164.5+14.7, -35.4, 141.0, 90.0, 180.0, -90.0]

    X18 = [164.5+50.0, 0.0, 141.0, 90.0, 180.0, -90.0]

    X2 = [264.5, 0.0, 141.0, 0.0, 90.0, 0.0]
    X3 = [164.5, 100.0, 141.0, 90.0, 90.0, 0.0]
    X4 = [164.5, -100.0, 141.0, 90.0, -90.0, 0.0]


if __name__ == "__main__":
    main()
