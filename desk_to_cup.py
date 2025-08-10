from time import sleep, perf_counter
# import logging
import numpy as np
from signal import signal, SIGINT   
from robot_tools.kinematics.robotarm_class import SmallRbtArm
import robot_tools.controller.robot_controller as controller
import robot_tools.trajectory.traj_plan_lfpb as pt

from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

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

smallRobotArm = None  # Declare at module level
def handler(signum, frame):
    """
    Signal handler to gracefully exit the program on Ctrl+C.
    """
    print("\nSignal received, exiting gracefully...")
    if smallRobotArm is not None:
        smallRobotArm.go_home()  # Move the robot arm to home position
    exit(0)  # Exit the program

def main() -> None:
    global smallRobotArm  # Tell Python to use the global variable
    signal(SIGINT, handler)
    # Create a custom robot object based on my DH parameters for std dh tbl.
    smRobot = DHRobot(std_dh_table, name="smallRobotArm")
    print("Reach of the robot:", smRobot.reach)
    print("nbranches", smRobot.nbranches)
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
    rbt_controller = controller.RobotController(smRobot)
    smallRobotArm.controller = rbt_controller  # Assign the controller to the robot arm instance
                    
    # there must be a delay here right after sieal is initialized
    sleep(1)  # don't remove this line!!!
    smallRobotArm.controller.enable()  # enable the robot arm
    sleep(1)
    # calibration
    # smallRobotArm.calibrate()
    # smallRobotArm.conn._event_ok2send.clear()    
    # print("[DEBUG] Waiting for ack...")
    # sleep(2)
    
    # P_desk_to_cup
    p = (-160, 150, 35, 0.0 ,0.0, 35.0)
    T_06 = smallRobotArm.convert_p_dc_to_T06(p)
    j = smRobot.ikine_LM(SE3(T_06), q0=np.ones(smRobot.n))
    if not j.success:
        print("IK solution failed!!!")
        exit(1)
    
    kine_j = np.degrees(j.q)   
    # j= smallRobotArm.ik(T0_6) 
    smallRobotArm.controller.move_to_angles(kine_j, header="g", ack=True)
    input("Press Enter to continue...")
    smallRobotArm.controller.go_home()
        
if __name__ == "__main__":
    main()

#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
from ikpy.chain import Chain

# ====== CONFIG ======
URDF_PATH = "my_robot.urdf"  # change to your robot's URDF
GRIPPER_OFFSET = [0.0, 0.0, 0.10]  # 10 cm above object
GRIPPER_RPY = [180, 0, 0]          # align gripper Z with object Z
APPROACH_DIST = 0.10               # approach distance in meters
RETREAT_DIST = 0.10                # retreat distance in meters

# ====== UTILS ======
def make_pose(position, rpy_deg):
    """Create 4x4 homogeneous transform from pos + RPY."""
    rot = R.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = position
    return pose

def grasp_pose(object_pose, offset_pose):
    """Object pose × offset → gripper pose."""
    return object_pose @ offset_pose

def offset_along_gripper_z(pose, dist):
    """Move along gripper +Z axis."""
    new_pose = np.copy(pose)
    new_pose[:3, 3] += new_pose[:3, 2] * dist
    return new_pose

# ====== MAIN PLANNER ======
def plan_pick_and_place(object_pose, place_pose, chain):
    """Generate IK joint sequences for pick-and-place."""
    offset_pose = make_pose(GRIPPER_OFFSET, GRIPPER_RPY)

    # PICK
    grasp = grasp_pose(object_pose, offset_pose)
    approach = offset_along_gripper_z(grasp, APPROACH_DIST)
    retreat = offset_along_gripper_z(grasp, -RETREAT_DIST)

    # PLACE
    place = grasp_pose(place_pose, offset_pose)
    pre_place = offset_along_gripper_z(place, APPROACH_DIST)
    post_place = offset_along_gripper_z(place, -RETREAT_DIST)

    # Solve IK for each waypoint
    waypoints = [approach, grasp, retreat, pre_place, place, post_place]
    joint_paths = []
    for pose in waypoints:
        pos = pose[:3, 3]
        rot = pose[:3, :3]
        joints = chain.inverse_kinematics(pos, target_orientation=rot)
        joint_paths.append(joints)

    return joint_paths

# ====== DEMO ======
if __name__ == "__main__":
    # Load robot chain
    chain = Chain.from_urdf_file(URDF_PATH)

    # Example object & place poses in WORLD frame
    X_object = make_pose([0.5, 0.0, 0.2], [0, 0, 0])  # object at 50cm forward, 20cm high
    X_place  = make_pose([0.3, 0.3, 0.2], [0, 0, 90]) # place at right side

    # Plan sequence
    joint_seq = plan_pick_and_place(X_object, X_place, chain)

    # Print joint sequences
    for i, q in enumerate(joint_seq):
        print(f"Waypoint {i+1}: {np.round(q, 3)}")


'''
def pick_and_place(T_cup: np.ndarray, ik_solver, move_to_pose, gripper):
    """
    High-level pick and place routine.
    
    Args:
        T_cup: 4x4 np.ndarray, cup pose in base frame
        ik_solver: function that returns joint angles given a 4x4 pose
        move_to_pose: function to command the robot to joint angles
        gripper: object with open() and close() methods
    """
    # 1. Define grasp and approach poses
    T_approach_offset = np.eye(4)
    T_approach_offset[2, 3] = -0.1  # 10cm above the cup

    T_approach = T_cup @ T_approach_offset  # offset in cup frame
    T_grasp = T_cup                         # directly at the cup

    # 2. Move to approach pose
    q_approach = ik_solver(T_approach)
    move_to_pose(q_approach)

    # 3. Move to grasp pose
    q_grasp = ik_solver(T_grasp)
    move_to_pose(q_grasp)

    # 4. Close gripper
    gripper.close()

    # 5. Retreat back to approach
    move_to_pose(q_approach)

    # 6. Move to place location (define as needed)
    T_place = np.eye(4)
    T_place[:3, 3] = [0.3, -0.2, 0.1]  # Example place location
    T_place_approach = T_place.copy()
    T_place_approach[2, 3] += 0.1

    q_place_approach = ik_solver(T_place_approach)
    q_place = ik_solver(T_place)

    move_to_pose(q_place_approach)
    move_to_pose(q_place)

    # 7. Open gripper
    gripper.open()

    # 8. Retreat
    move_to_pose(q_place_approach)

'''