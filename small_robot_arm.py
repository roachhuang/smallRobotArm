from time import sleep, perf_counter
import logging
import pandas as pd
import numpy as np

# import matplotlib.pyplot as plt
# from eclipse_motion_poses import generate_tilted_ellipse

# import pyvista
import equations as eq
import robotarm_class as robot
import plan_traj as pt
from roboticstoolbox import DHRobot, RevoluteDH

from spatialmath import SE3

# from spatialmath.base import trplot

# from scipy.spatial.transform import Rotation as R
# from scipy.interpolate import CubicSpline

# Define your DH table parameters
# 50 is distance btw 6th axis and the extended end-effector
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

#   x, y, z, ZYZ Euler angles}
# Xhome = [(164.5, 0.0, 241.0, 90.0, 180.0, -90.0]

"""
todo:
    1. Gimbal Lock: A potential issue with Euler angles is "gimbal lock," where you lose a degree of freedom. This can happen when the second rotation (around the Y-axis in Z-Y-Z) is 90 degrees.
        convert t to quaternion and convert it back then pass it to ik.    
    2. ik needs to take offset into account since dh alreay has offset?
    3. sigularities, numberical instability.
    4. precision test for position
    5. add ai onto it.
    6. ros2
    7. ar4 or ar4-mk3   
    8. reachable workspace
    9. from a point to b point, may have mutilple soultions from ik. choose minimum thetas
    10. research spatialmath
    11. refact s1.ino
    12. use 'g' command, like 3-d printer. 
    13. ik takes desired pose of end-effector frame relative to the world frame.
    14. ik and fk must take world frame and end-effector frame into consideration.
    what about cup frame? ask gpt.
    always use np.array coz it can represent any number of dimensions (vectors, matrices, tensors, etc.). 
    15. minimizing joint movement.
"""

"""
Create a Virtual Environment
python -m venv myenv
source myenv/bin/activate  # Linux/macOS
myenv\Scripts\activate  # Windows
"""

def main() -> None:
    logging.basicConfig()
    DOF = 6
    bTrajectory = True
    # bTrajectory = False

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
    smallRobotArm = robot.SmallRbtArm(std_dh_params)
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
    # input("Press Enter to continue...")

    # there must be a delay here right after sieal is initialized
    sleep(1)  # don't remove this line!!!
    smallRobotArm.enable()
    T = smallRobotArm.fk(smallRobotArm.robot_rest_angles)
    print(f"rest pose: {smallRobotArm.T2Pose(T)}")
    
    """
    for curPos are all zero
    after fk for [0, 78.51, -73,9, 0, -1.14, 0], computed manually from arduino goHome code.
    home postion = [301.85, 0.0, 168.6, -180, -84.0, 0]
    """
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
            (164.5, 0.0, 241.0, 90.0, 180.0, -90.0),  # Home (x, y, z, ZYZ Euler angles)
        ]
    )

    # end-effec or pose = the position and orientation of the robot's last link (often frame 6) relative to the robot's base frame (frame 0).
    # poses = np.array(
    #     [
    #         # current is(90,180,-90), after rotating the current orientation by 35 deg about z axis
    #         # the new zyz is (35,180,-55)
    #         [0,  284,     90,       60,  0.0,  0.0, 35.0],
    #         [8,  284,     90,       100, 0.0,  0.0, 35.0],
    #         [20, 284-160, 90.0+80,  350, 0.0, -60.0, 0.0],
    #         [24, 284-160, 90.0+160, 350, 0.0, -60.0, 0.0],
    #     ],
    #     dtype=np.float64,
    # )

    # we use ntu example: orientation with euler FIXED angles
    # Cup poses wrt frame {0}.
    poses = np.array(
        [
            (0, 250, 70.0 + 20, 60, 0, 0.0, 35.0),
            (8, 264.5 + 19, 70.0 + 20, 90, 0, 0.0, 35.0),
            # rotate cup 60 degrees around y axis wrt the world frame.
            (20, 264.5 - 120, 70.0 + 60, 350.0, 0, -60.0, 0.0),
            (24, 264.5 - 120, 70.0 + 100, 355.0, 0, -60.0, 0.0),
        ],
        dtype=np.float64,
    )

    # this is only for giving time col to joints
    joints = poses.copy()

    
    # elliptical trajectory
    '''
    center = (50, 0, 140)  # Ellipse centered at (300, 0, 200) in mm
    radii = (100, 50)  # Ellipse radii (X = 100mm, Y = 50mm)
    tilt_angle=np.radians(30)
    """Generate SE(3) poses for an elliptical trajectory on a tilted plane."""
    # poses = em.generate_tilted_ellipse(center, radii, tilt_angle)
    for T in generate_tilted_ellipse(
        center=(50, 0, 140),
        radii=(100, 50),
        tilt_angle=np.radians(-30),
        num_points=50,
    ):
        my_j= smallRobotArm.ik(T.A)
        ik_sol = smRobot.ikine_LM(SE3(T), q0=np.ones(smRobot.n))
        if not ik_sol.success:
            print("IK solution failed!!!")
            continue
        # cross comparison of ik results from both libraries
        kine_j = np.degrees(ik_sol.q)
        myfk_T = smallRobotArm.fk(kine_j)
        fkine_T = smRobot.fkine(np.radians(my_j))
        if not np.allclose(fkine_T.A, myfk_T, rtol=1e-2, atol=1e-2):
            print(myfk_T)
            print(fkine_T.A)
            raise ValueError("T and myfk_T are not close")
        smallRobotArm.move_to_angles(kine_j)
        # cmd = {"header": "m", "joint_angle": kine_j, "ack": True}
        # smallRobotArm.conn.send2Arduino(cmd)
        
    smallRobotArm.go_home()
    exit(0)
    
    '''
    '''
    # interpolate between two poses
    T1 = SE3.Trans(240, -140, 40) * SE3.RPY(30, 0, 30, unit='deg')
    T2 = SE3.Trans(160, 140, 140) * SE3.RPY(0, 0, 60, unit='deg')
    poses = smallRobotArm.interpolate_poses(T1.A, T2.A) # Get 10 smooth poses.
    for pose in poses:        
        ik_sol = smRobot.ikine_LM(pose, q0=np.ones(smRobot.n))
        if not ik_sol.success:
            print("IK solution failed!!!")
            exit(0)
        smallRobotArm.move_to_angles(np.degrees(ik_sol.q))
    input("Press Enter to continue...")
    smallRobotArm.go_home()  
    exit(0)
    '''
    
    if bTrajectory == False:
        for pose in poses0:
            # poses' coordiation system is end-effector's wrt world frame.
            T_0E = smallRobotArm.pose2T(pose, seq="zyz")
            # T_06 = T_0E @ smallRobotArm.T_6E_inv
            # euler angles ZYZ according to smallrobot arm's demo
            my_j = smallRobotArm.ik(T_0E)
            # T=smallRobotArm.pose2T(corrected_pose, seq="ZYZ")
            ik_sol = smRobot.ikine_LM(SE3(T_0E), q0=np.ones(smRobot.n))
            if not ik_sol.success:
                print("IK solution failed!!!")
                continue
            # cross comparison of ik results from both libraries
            kine_j = np.degrees(ik_sol.q)
            myfk_T = smallRobotArm.fk(kine_j)
            fkine_T = smRobot.fkine(np.radians(my_j))
            # SE3(T_0E).printline()
            if not np.allclose(fkine_T.A, myfk_T, rtol=1e-2, atol=1e-2):
                print(myfk_T)
                print(fkine_T.A)
                raise ValueError("T and myfk_T are not close")

            """            
            # Plot the frame coordinate with customization
            smRobot.plot(np.radians(my_j), backend='pyplot', block=False, jointaxes=True)
            # Get the current axes from robot.plot()          
            ax = plt.gca()                        
            fkine_all_T = smRobot.fkine_all(ik_sol.q)
            # Plot coordinate frames for each link
            for i, t in enumerate(fkine_all_T):
                T_arr = t.A
                smb.trplot(T_arr, ax=ax, width=2, length=20, color=('r','g','b'))
                              
            # Rotate the view for better visibility (optional)
            # ax.view_init(elev=30, azim=45)  # Adjust as needed
            plt.draw()
            plt.pause(0.1)  # w/o this line frame coordiantes won't be displayed.
            """
            # print(kine_j.round(2))
            smallRobotArm.move_to_angles(kine_j)

            # input("Press Enter to continue...")
        # plt.show()

    ###################################################################
    else:
        # this line is required coz reach to this pose at 0 sec as the poses says. ntu: fixed euler anglers
        T_0E_at_0s = SE3.Trans(poses[0, 1:4]) * SE3.RPY(
            poses[0, 4:7], order="zyx", unit="deg"
        )
        T_06_at_0s = T_0E_at_0s.A @ smallRobotArm.T_6E_inv
        j = smallRobotArm.ik(T_06_at_0s)
        smallRobotArm.move_to_angles(j)
        input("Press Enter to continue...")
        # traj planning in joint-space.
        """
        there is another method: in cartesian-space. if it needs 100 operating point in 1s (100hz) for period of 9s,
        in this caseg we need to do 100x9=900 times of ik.
        """
        for i, pose in enumerate(poses, start=0):
            # col 0 are time data
            _, *p = pose
            # fixed angles according to NTU course

            T_0E = SE3.Trans(p[0:3]) * SE3.RPY(p[3:6], order="zyx", unit="deg")
            T_06 = T_0E.A @ smallRobotArm.T_6E_inv
            joints[i, 1:7] = smallRobotArm.ik(T_06)

        # display easily readable ik resutls on the screen
        J = pd.DataFrame(joints, columns=["ti", "q1", "q2", "q3", "q4", "q5", "q6"])
        print(J.round(4))

        print("--- Start trajectory planning ---")
        (v, a) = pt.planTraj(joints)

        (totalPoints, _) = np.shape(joints)
        logging.info(v)
        logging.info(a)
        ts = joints[:, 0]
        # get rid of time col
        js = joints[:, 1:7]
        sleep(1)
        # get time from col 0 of p

        start_time = curr_time = perf_counter()
        Xx = np.zeros([6], dtype=np.float64)

        """
        # Create CubicSpline interpolators for each joint
        splines = [CubicSpline(ts, js[:, col]) for col in range(DOF - 1)]

        while curr_time - start_time <= ts[-1]:
            dt = curr_time - start_time
            curr_time = perf_counter()

            for col in range(DOF - 1):
                Xx[col] = splines[col](dt)
            # ask arduino to run goTractory(Xx)
            cmd =   "header": "m", "joint_angle": Xx, "ack": False}
            smallRobotArm.conn.send2Arduino(cmd)
            # must be a delay here. ack takes too long causing discontinued arm movement.
            sleep(1 / 100)
        """

        # ts[-1] last one element
        while curr_time - start_time <= ts[-1]:
            dt = curr_time - start_time
            # print('Time elasped:{time:.4f}'.format(time=dt))

            for col in range(DOF):
                if dt >= ts[0] and dt <= ts[0] + 0.5:
                    Xx[col] = js[0, col] + eq.eq1(dt, v[0, col], a[0, col])
                elif dt > ts[0] + 0.5 and dt <= ts[1] - 0.25:
                    Xx[col] = js[0, col] + eq.eq2(dt, v[1, col])
                elif dt > ts[1] - 0.25 and dt <= ts[1] + 0.25:
                    Xx[col] = js[0, col] + eq.eq3(dt, ts[1], v[1, col], a[1, col])
                elif dt > ts[1] + 0.25 and dt <= ts[2] - 0.25:
                    Xx[col] = js[1, col] + eq.eq4(dt - ts[1], v[2, col])
                elif dt > ts[2] - 0.25 and dt <= ts[2] + 0.25:
                    Xx[col] = js[1, col] + eq.eq5(dt, ts, v[2, col], a[2, col])
                elif dt > ts[2] + 0.25 and dt <= ts[totalPoints - 1] - 0.5:
                    Xx[col] = js[2, col] + eq.eq6(dt - ts[2], v[3, col])
                elif dt > ts[totalPoints - 1] - 0.5 and dt <= ts[totalPoints - 1]:
                    Xx[col] = js[2, col] + eq.eq7(
                        dt, ts, v[3, col], a[3, col], totalPoints
                    )

            # ask arduino to run goTractory(Xx)
            cmd = {"header": "m", "joint_angle": Xx, "ack": True}
            # print(f"Xx: {Xx}")
            smallRobotArm.conn.send2Arduino(cmd)
            # must be a delay here. ack takes too long causing discontinued arm movement.
            # sleep(0.3)
            curr_time = perf_counter()

        # input("Press Enter to continue...")

        # this is to set ji in arduino coz of from and to args for goStrightLine
        # T_0C = smallRobotArm.pose2T(rest_pose)
        # # T_0C = smallRobotArm.pose2T((110, 323.2, 320, 0.0, -60.0, 0.0))
        # T_06 = T_0C @ T_C6_inv
        # ji = smallRobotArm.ik(T_06)
        # # ji = smallRobotArm.ik((264.5 - 120, 70.0 + 100, 355.0, 0.0, -60.0, 0.0))
        # cmd = {"header": "c", "joint_angle": ji, "ack": False}
        # smallRobotArm.conn.send2Arduino(cmd)

    # smallRobotArm.moveTo([47.96, 0.0, 268.02, 180, 94.61, 180.0])
    # initPose[0:3] = [11.31000000e02, 1.94968772e-31, 2.78500000e02]
    # initPose[3:6] = [10.00000000e00, 1.27222187e-14, 1.80000000e02]
    # smallRobotArm.moveTo(initPose)
    smallRobotArm.go_home()
    print("THREAD TERMINATED!")


if __name__ == "__main__":
    main()
