from time import sleep, perf_counter
import logging
# import pandas as pd
import numpy as np
# import pyvista
# preferred - use absolute imports for package structure
from robot_tools.kinematics import SmallRbtArm
from robot_tools.controller import RobotController
from robot_tools.trajectory import Interp
from robot_tools.misc.signal_handler import setup_signal_handler

from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import sys

# from scipy.spatial.transform import Rotation as R
# from scipy.interpolate import CubicSpline

# Define your DH table parameters
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

"""
todo:
    1. Gimbal Lock: A potential issue with Euler angles is "gimbal lock," where you lose a degree of freedom. This can happen when the second rotation (around the Y-axis in Z-Y-Z) is 90 degrees.
        convert t to quaternion and convert it back then pass it to ik.    
    2. ik needs to take offset into account since dh alreay has offset?
    3. sigularities, numberical instability.
    5. add ai onto it.
    7. ar4 or ar4-mk3   
    8. reachable workspace
    9. from a point to b point, may have mutilple soultions from ik. choose minimum thetas
    10. research spatialmath
    13. ik takes desired pose of end-effector frame relative to the world frame.
    14. ik and fk must take world frame and end-effector frame into consideration.
    always use np.array coz it can represent any number of dimensions (vectors, matrices, tensors, etc.). 
    15. minimizing joint movement.
Create a Virtual Environment
python -m venv myenv
source myenv/bin/activate  # Linux/macOS
"""

# smallRobotArm = None  # Declare at module level

def main() -> None:
    logging.basicConfig()
    DOF = 6

    np.set_printoptions(precision=2, suppress=True)

    # Create a custom robot object based on my DH parameters for std dh tbl.
    smRobot = DHRobot(std_dh_table, name="smallRobotArm")
    print("Reach of the robot:", smRobot.reach)
    print("nbranches", smRobot.nbranches)
    # smRobot.islimit([0, 0, -4, 4, 0, 0])
    print("is spherical:", smRobot.isspherical())
    # Robot kinematics as an elemenary transform sequence
    smRobot.ets()    
    
    # r6=distance btw axis6 and end-effector
    std_dh_params = np.array(
        [
            [np.radians(-90), a1, d1],
            [0, a2, 0],
            [np.radians(-90), a3, 0],
            [np.radians(90), 0, d4],
            [np.radians(-90), 0, 0],
            [0, 0, d6],
        ]
    )
    
    # create an instance of the robotarm.
    smallRobotArm = SmallRbtArm(std_dh_params)
    controller = RobotController(smRobot)
    interp = Interp()
    # Set up signal handler for graceful exit on Ctrl+C
    setup_signal_handler(smallRobotArm)
                    
    # these values derived from fig 10 of my smallrobotarm notebook
    """move to rest angles at begining won't work coz 1. the motors don't have encoder. 2. If your robot uses incremental encoders, it loses its position when power is cycled or the program is restarted.
    Solution: Implement homing at startup using absolute encoders or external reference sensors.e motors have no encoder to record previous angles.
    """    

    # zero positon (see fig1)
    zero_j = (0, 0, 0, 0, 0, 0)
    T = smRobot.fkine(np.radians(zero_j))
    # j=smallRobotArm.ik(T.A)
    # smallRobotArm.move_to_angles(j)
    print(f"zero joint pose: {smallRobotArm.T2Pose(T.A)}")

    # there must be a delay here right after sieal is initialized
    sleep(1)  # don't remove this line!!!
    controller.enable()  # enable the robot arm
    sleep(1)
    # calibration
    # smallRobotArm.calibrate()
    # smallRobotArm.conn._event_ok2send.clear()    
    # print("[DEBUG] Waiting for ack...")
    # sleep(2)
    
    # T = smallRobotArm.fk(smallRobotArm.robot_rest_angles)
    # print(f"rest pose: {smallRobotArm.T2Pose(T)}")
    
    """ 
    these end-effector posese are wrt frame{0}
    initially, think of the orientation in rotation matrix is easier to understand:
        suppose you wnat the end-effector's X-axis to point in the opposite direction of the base frame's X-axis,
        and the Y and Z axes to be aligned. rotation_matrix = np.array([
        [-1, 0, 0],
        [0, 1, 0],
        [0, 0, 1]
        ])
        then convert the R into degrees in 'ZYZ' sequence
    """
    # end-effector pose = the position and orientation of the robot's last link (often frame 6) relative to the robot's base frame (frame 0).
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
            (164.5, 0.0, 241.0, 90.0, 180.0, 90.0),  # Home (x, y, z, ZYZ Euler angles)
        ]
    )
   
    T_inter_poses=[]
    # Loop through consecutive pairs and interpolate
    for i in range(len(poses0) - 1):
        start_pose = smallRobotArm.pose2T(poses0[i])
        end_pose = smallRobotArm.pose2T(poses0[i + 1])
        
        T_inter_poses.append(start_pose)  # Keep the original
        interpolated = interp.interpolate_poses(SE3(start_pose), SE3(end_pose), num_poses=5)
        T_inter_poses.extend(interpolated)

    # Append the final pose
    T_inter_poses.append(smallRobotArm.pose2T(poses0[-1]))
    # T_smooth_poses = path.smooth_pose_path(SE3(T_inter_poses))
    
    for T in T_inter_poses:
        # poses' coordiation system is end-effector's wrt world frame.
        # T_0E = smallRobotArm.pose2T(pose, seq="zyz")
        
        # euler angles ZYZ according to smallrobot arm's demo
        # my_j = smallRobotArm.ik(T_0E)
        # T=smallRobotArm.pose2T(corrected_pose, seq="ZYZ")
        ik_sol = smRobot.ikine_LM(SE3(T), q0=np.ones(smRobot.n))
        if not ik_sol.success:
            print("IK solution failed!!!")
            continue
        kine_j = np.degrees(ik_sol.q)
        
        '''
        # cross comparison of ik results from both libraries
        myfk_T = smallRobotArm.fk(kine_j)
        fkine_T = smRobot.fkine(np.radians(my_j))
        if not np.allclose(fkine_T.A, myfk_T, rtol=1e-2, atol=1e-2):
            print(myfk_T)
            print(fkine_T.A)
            raise ValueError("T and myfk_T are not close")
        '''
        
        # print(kine_j.round(2))
        
        diff = np.linalg.norm(np.array(kine_j) - np.array(controller.current_angles))
        steps = min(20, max(5, int(diff / 10 * 10)))  # Scale reasonably (5 deg per step)s = int(diff * 10)  # 10 steps per radian of total joint-space distance
        
        linear_path = interp.generate_linear_path_in_js(controller.
                                                                    current_angles, kine_j, steps)
        
        # compute the joint-space inertia matrix
        M=smRobot.inertia(kine_j)  # 6x6 joint-space inertia matrix
        eigvals, eigvecs = np.linalg.eig(M)
        # 2. Find "easy-move" direction
        easy_direction = eigvecs[:, np.argmin(eigvals)]
        # 3. Adjust motion path to align with easy direction            
        adjusted_j = controller.align_path_to_vector(linear_path, easy_direction)
        for angles in adjusted_j:
            controller.move_to_angles(angles, header="g", ack=True)
        
        # smallRobotArm.move_to_angles(kine_j, header="g", ack=True)
        # controller.current_angles = kine_j  # Important to track for next segment
    
    input("Press Enter to continue...")
    controller.go_home()
    print("THREAD TERMINATED!")

if __name__ == "__main__":
    sys.exit(main())
