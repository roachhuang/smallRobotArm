from time import sleep, perf_counter
import logging
import numpy as np
import signal

'''When you do import robot_tools.controller.robot_controller as controller,
controller refers to the module robot_controller.py, not to an object.
The method velocity_control is defined inside the RobotController class, so you need to call it on an instance of that class.'''
   
from robot_tools.kinematics.robotarm_class import SmallRbtArm
import robot_tools.controller.robot_controller as controller
# import robot_tools.trajectory.plan_traj as pt

from roboticstoolbox import DHRobot, RevoluteDH
# from spatialmath import SE3

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
        smallRobotArm.controller.go_home()  # Move the robot arm to home position
    exit(0)  # Exit the program

def main() -> None:
    global smallRobotArm  # Tell Python to use the global variable
    signal.signal(signal.SIGINT, handler)
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
    
    # zero_j = (0, 0, 0, 0, 0, 0)
    # smallRobotArm.controller.move_to_angles(zero_j, header="g", ack=True)
    
    # Compute the Jacobian matrix in the base frame
    # jacobian_matrix = smRobot.jacob0(smallRobotArm.controller.robot_rest_angles)
    # Desired end-effector velocity in world frame (ẋ = [vx, vy, vz, ωx, ωy, ωz])
    # my dhtbl is in mm, so here x,y,z in mm/s
    ''' 
    linear vel: vx, vy, vz; angular vel: wx, wy, wz
    '''
    x_dot = np.array([20.0, 20.0, 10.0, 0.0, 0.0, 0.0])  # [mm/s, rad/s]
    dt = 0.02  # control period (e.g. 50Hz)
    q = np.radians(smallRobotArm.controller.current_angles)  # joint angles in radians
    
    # simulating velocity control in joint space
    for step in range(250):  # run for 5 seconds. steps= duration in sec/0.02
        J = smRobot.jacob0(q)  # 6x6 Jacobian at current joint state
        U, Sigma, Vt = np.linalg.svd(J)
        Sigma_plus = np.diag([1/s if s > 1e-6 else 0 for s in Sigma])  # Damped pseudoinverse
        J_plus = Vt.T @ Sigma_plus @ U.T
        # q_dot = np.linalg.pinv(J, rcond=1e-4) @ x_dot  # 6x1 joint velocity vector
        q_dot = J_plus @ x_dot  # Joint velocities
        # q_dot = np.clip(q_dot, 0.1, 0.5)
        q += q_dot * dt  # integrate to get next joint position

        deg_q = np.degrees(q)
        smallRobotArm.controller.move_to_angles(deg_q, header="g", ack=True)
        # smallRobotArm.controller.current_angles = deg_q.copy()

        sleep(dt)
      
    # input("Press Enter to continue... square w/ corners vel")    
    # smallRobotArm.controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(smallRobotArm.controller.current_angles),
    # x_dot_func=lambda t: smallRobotArm.controller.square_with_corners_velocity(
    #     t, side_length=100, edge_period=2.0, hover_duration=2.0, circle_radius=25.0, circle_period=2.0),
    # dt=0.05,
    # duration=4 * (4.0 + 2.0 + 2.0)  # 32s = 4 corners * (edge+hover+circle)
    # )
    
    input("Press Enter to continue circle vel...")    
    smallRobotArm.controller.velocity_control(robot=smRobot, q_init=np.radians(smallRobotArm.controller.current_angles), x_dot_func=lambda t: smallRobotArm.controller.circle_xy_velocity(t, radius=40, period=15.0), dt=0.05, duration=15.0)
   
    # input("Press Enter to continue square vel...")
    # smallRobotArm.controller.velocity_control(robot=smRobot, q_init=np.radians(smallRobotArm.controller.current_angles), x_dot_func=lambda t: smallRobotArm.controller.square_xz_velocity(t, side_length=40, period=15.0), dt=0.05, duration=15.0)
    
    # input("Press Enter to continue figure_eight vel...")
    # smallRobotArm.controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(smallRobotArm.controller.current_angles),
    # x_dot_func=lambda t:smallRobotArm.controller.figure_eight_velocity(t, radius=30, freq=1.0),
    # dt=0.05,
    # duration=30.0    
    # )
    
    # input("Press Enter to continue Zigzag/sawtooth pattern...")
    # smallRobotArm.controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(smallRobotArm.controller.current_angles),
    # x_dot_func=lambda t:smallRobotArm.controller.zigzag_velocity(t, side_length=40, period=20.0),
    # dt=0.05,
    # duration=30.0    
    # )

    # input("Press Enter to continue spiral vel...")
    # smallRobotArm.controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(smallRobotArm.controller.current_angles),
    # x_dot_func=lambda t:smallRobotArm.controller.spiral_velocity(t, radius_rate=2.0, angular_speed=0.5),
    # dt=0.05,
    # duration=30.0    
    # )

    input("Press Enter to go home...")   
    smallRobotArm.controller.go_home()
        
if __name__ == "__main__":
    main()
