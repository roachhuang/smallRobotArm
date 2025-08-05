from time import sleep, perf_counter
import logging
import pandas as pd
import numpy as np
import sys   
# import pyvista
from robot_tools.trajectory import plan_traj_with_lfpb, equations as polynom
from robot_tools.kinematics import SmallRbtArm, std_dh_params, std_dh_tbl
from robot_tools.controller import PositionController
from robot_tools.misc.signal_handler import setup_signal_handler
from roboticstoolbox import DHRobot
from spatialmath import SE3

# from scipy.spatial.transform import Rotation as R
# from scipy.interpolate import CubicSpline

def main() -> None:
    logging.basicConfig()
    np.set_printoptions(precision=2, suppress=True)

    # Create a custom robot object based on my DH parameters for std dh tbl.
    smRobot = DHRobot(std_dh_tbl, name="smallRobotArm")
    print("Reach of the robot:", smRobot.reach)
    print("nbranches", smRobot.nbranches)
    # smRobot.islimit([0, 0, -4, 4, 0, 0])
    print("is spherical:", smRobot.isspherical())
    # Robot kinematics as an elemenary transform sequence
    smRobot.ets()        
   
    # create an instance of the robotarm.
    smallRobotArm = SmallRbtArm(std_dh_params)
    controller = PositionController(smRobot)
    setup_signal_handler(controller) 
                    
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
    
  
    # we use ntu example: orientation with euler FIXED angles
    # Cup poses wrt frame {desk}.
    cartesian_path = np.array(
        [
            (0, -150,        190, 60, 0.0, 0.0, 35.0),
            (8, -164.5 + 19, 190, 90, 0.0, 0.0, 35.0),
            # rotate cup 60 degrees around y axis wrt the world frame.
            (20, -164.5 - 120, 170.0 + 60, 350.0, 0, -60.0, 0.0),
            (24, -164.5 - 120, 170.0 + 100, 355.0, 0, -60.0, 0.0),
        ],
        dtype=np.float64,
    )

    # this is only for giving time col to joints
    joint_path = cartesian_path.copy()
    
    # the pose at 0s. ntu: fixed euler anglers
    p_dc = cartesian_path[0, 1:]
    # Convert to transformation matrix
    T_dc = SE3.Trans(p_dc[0:3]) * SE3.RPY(p_dc[3:6], order="zyx", unit="deg")
    # Define grasp candidates (different approach directions/orientations)
    grasp_candidates=[ 
        {'pose': p_dc, 'approach_vector': [0, 0, 1]},   # Top-down
        {'pose': p_dc, 'approach_vector': [1, 0, 0]},   # Side approach
        {'pose': p_dc, 'approach_vector': [0, 1, 0]},   # Front approach
    ]
        
    # new_T_dc = controller.compute_approach_pose(T_dc.A, approach_vec_cup=[0,1,0], offset=50)
    # new_p_dc = smallRobotArm.T2Pose(new_T_dc)
    # T_approach = smallRobotArm.convert_p_dc_to_T06(new_p_dc)    
    # j = smallRobotArm.ik(T_approach)
    # controller.move_to_angles(j)
    # input("approach pose. Press Enter to continue...")
    
    T_06_at_0s = smallRobotArm.convert_p_dc_to_T06(p_dc)
    j = smallRobotArm.ik(T_06_at_0s)
    controller.move_to_angles(j)
    
    input("Press Enter to continue...")
    # traj planning in joint-space.
    """
    there is another method: in cartesian-space. if it needs 100 operating point in 1s (100hz) for period of 9s,
    in this caseg we need to do 100x9=900 times of ik.
    """
    for i, pose in enumerate(cartesian_path, start=0):
        # col 0 are time data
        _, *p = pose
        # fixed angles according to NTU course
        T_06 = smallRobotArm.convert_p_dc_to_T06(p)        
        joint_path[i, 1:7] = smallRobotArm.ik(T_06)

    # display easily readable ik resutls on the screen
    J = pd.DataFrame(joint_path, columns=["ti", "q1", "q2", "q3", "q4", "q5", "q6"])
    print(J.round(4))

    print("--- Start trajectory planning ---")
    (v, a) = plan_traj_with_lfpb(joint_path)

    (totalPoints, _) = np.shape(joint_path)
    logging.info(v)
    logging.info(a)
    ts = joint_path[:, 0]
    # get rid of time col
    js = joint_path[:, 1:7]
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

        for col in range(smallRobotArm.dof):
            if dt >= ts[0] and dt <= ts[0] + 0.5:
                Xx[col] = js[0, col] + polynom.eq1(dt, v[0, col], a[0, col])
            elif dt > ts[0] + 0.5 and dt <= ts[1] - 0.25:
                Xx[col] = js[0, col] + polynom.eq2(dt, v[1, col])
            elif dt > ts[1] - 0.25 and dt <= ts[1] + 0.25:
                Xx[col] = js[0, col] + polynom.eq3(dt, ts[1], v[1, col], a[1, col])
            elif dt > ts[1] + 0.25 and dt <= ts[2] - 0.25:
                Xx[col] = js[1, col] + polynom.eq4(dt - ts[1], v[2, col])
            elif dt > ts[2] - 0.25 and dt <= ts[2] + 0.25:
                Xx[col] = js[1, col] + polynom.eq5(dt, ts, v[2, col], a[2, col])
            elif dt > ts[2] + 0.25 and dt <= ts[totalPoints - 1] - 0.5:
                Xx[col] = js[2, col] + polynom.eq6(dt - ts[2], v[3, col])
            elif dt > ts[totalPoints - 1] - 0.5 and dt <= ts[totalPoints - 1]:
                Xx[col] = js[2, col] + polynom.eq7(
                    dt, ts, v[3, col], a[3, col], totalPoints
                )

        # ask arduino to run goTractory(Xx)
        controller.move_to_angles(Xx, header="g", ack=True)
        curr_time = perf_counter()       
    
    input("Press Enter to continue...")
    controller.go_home()
    print("THREAD TERMINATED!")

def simple_combined_pick(robot, controller, target_pose, grasp_candidates):
        """Simplest MIT + approach pose combination"""
        
        # 1. MIT: Select best grasp by reachability only
        best_grasp = None
        for grasp in grasp_candidates:
            joints = robot.ik(robot.convert_p_dc_to_T06(grasp['pose']))
            if joints is not None:  # Just check if reachable
                best_grasp = grasp
                break
        
        # 2. Your approach: Generate approach pose
        approach_pose = controller.compute_approach_pose(
            SE3.Trans(best_grasp['pose'][:3]) * SE3.RPY(best_grasp['pose'][3:], order="zyx", unit="deg").A,
            best_grasp['approach_vector'], 
            offset=50
        )
        
        # 3. Execute: approach → grasp → retreat → place
        controller.move_to_angles(robot.ik(robot.convert_p_dc_to_T06(robot.T2Pose(approach_pose))))
        controller.move_to_angles(robot.ik(robot.convert_p_dc_to_T06(best_grasp['pose'])))
        controller.grab()
        controller.move_to_angles(robot.ik(robot.convert_p_dc_to_T06(robot.T2Pose(approach_pose))))
        controller.move_to_angles(robot.ik(robot.convert_p_dc_to_T06(target_pose)))
        controller.drop()

if __name__ == "__main__":
    sys.exit(main())

