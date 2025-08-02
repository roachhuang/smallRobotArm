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
