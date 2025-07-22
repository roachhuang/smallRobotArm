from time import sleep, perf_counter
# import logging
import numpy as np
import sys

'''When you do import robot_tools.controller.robot_controller as controller,
controller refers to the module robot_controller.py, not to an object.
The method velocity_control is defined inside the RobotController class, so you need to call it on an instance of that class.'''
   
# from robot_tools.kinematics import SmallRbtArm
from robot_tools.controller import RobotController
# import robot_tools.trajectory.plan_traj as pt

from roboticstoolbox import DHRobot, RevoluteDH
from robot_tools.misc.signal_handler import setup_signal_handler

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

# smallRobotArm = None  # Declare at module level

def main() -> None:
    # Create a custom robot object based on my DH parameters for std dh tbl.
    smRobot = DHRobot(std_dh_table, name="smallRobotArm")
    # Robot kinematics as an elemenary transform sequence
    smRobot.ets()    
    
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
    # smallRobotArm = SmallRbtArm(std_dh_params)
    controller = RobotController(smRobot)
    # Set up signal handler for graceful exit on Ctrl+C
    setup_signal_handler(controller)                    
                    
    # there must be a delay here right after sieal is initialized
    sleep(1)  # don't remove this line!!!
    controller.enable()  # enable the robot arm
    sleep(1)
    # calibration
    # smallRobotArm.calibrate()
    # smallRobotArm.conn._event_ok2send.clear()    
    # print("[DEBUG] Waiting for ack...")
    # sleep(2)
    
    # zero_j = (0, 0, 0, 0, 0, 0)
    # controller.move_to_angles(zero_j, header="g", ack=True)
    
    # Compute the Jacobian matrix in the base frame
    # jacobian_matrix = smRobot.jacob0(controller.robot_rest_angles)
    # Desired end-effector velocity in world frame (ẋ = [vx, vy, vz, ωx, ωy, ωz])
    # my dhtbl is in mm, so here x,y,z in mm/s
    ''' 
    linear vel: vx, vy, vz; angular vel: wx, wy, wz
    '''
    x_dot = np.array([20.0, 20.
                      0, 20.0, 0.0, 0.0, 0.0])  # [mm/s, rad/s]
    dt = 0.02  # control period (e.g. 50Hz)
    q = np.radians(controller.current_angles)  # joint angles in radians
    
    controller.velocity_control(robot=smRobot, q_init=q, x_dot_func=lambda t: x_dot, dt=dt, duration=5.0)       
      
    # input("Press Enter to continue... square w/ corners vel")    
    # controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t: controller.square_with_corners_velocity(
    #     t, side_length=100, edge_period=2.0, hover_duration=2.0, circle_radius=25.0, circle_period=2.0),
    # dt=0.05,
    # duration=4 * (4.0 + 2.0 + 2.0)  # 32s = 4 corners * (edge+hover+circle)
    # )
    
    
    input("Press Enter to continue circle vel...") 
    # duration/period = number of circular loop. t goes from 0 to duration (stepping by dt)
    controller.velocity_control(robot=smRobot, q_init=np.radians(controller.current_angles), x_dot_func=lambda t: controller.fourier_circle_velocity(t, radius=40, period=15.0), dt=0.05, duration=15.0)
   
    # input("Press Enter to continue square vel...")
    # controller.velocity_control(robot=smRobot, q_init=np.radians(controller.current_angles), x_dot_func=lambda t: controller.square_xz_velocity(t, side_length=40, period=15.0), dt=0.05, duration=15.0)
    
    # input("Press Enter to continue figure_eight vel...")
    # controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t:controller.figure_eight_velocity(t, radius=30, freq=1.0),
    # dt=0.05,
    # duration=10.0    
    # )
    
    # input("Press Enter to continue Zigzag/sawtooth pattern...")
    # controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t:controller.zigzag_velocity(t, side_length=40, period=30.0),
    # dt=0.05,
    # duration=30.0    
    # )

    # input("Press Enter to continue spiral vel...")
    # controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t:controller.spiral_velocity(t, radius_rate=2.0, angular_speed=0.5),
    # dt=0.05,
    # duration=30.0    
    # )

    input("Press Enter to go home...")   
    controller.go_home()
        
if __name__ == "__main__":
    sys.exit(main())

