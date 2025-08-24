"""Jacobian-based Velocity Control Demo

Demonstrates velocity control for a 6-DOF robot arm using Jacobian-based
kinematics. Includes various motion patterns like circles, zigzag, and spirals.
"""

from time import sleep
import numpy as np
import sys

from roboticstoolbox import DHRobot

from robot_tools.kinematics import SmallRbtArm
from robot_tools.kinematics import std_dh_tbl, std_dh_params
# from robot_tools.controller import RobotController
from robot_tools.controller import VelocityController
from robot_tools.trajectory.velocity_optimizer import VelOptimizer
from robot_tools.misc.signal_handler import setup_signal_handler
from robot_tools.trajectory.motion_patterns import MotionPatterns

# from scipy.spatial.transform import Rotation as R
# from scipy.interpolate import CubicSpline

def main() -> None:
    """Main function demonstrating velocity control with various motion patterns."""
    
    # Initialize robot models
    toolbox_robot = DHRobot(std_dh_tbl, name="smallRobotArm")
    toolbox_robot.ets()  # Generate elementary transform sequence
    
    custom_robot = SmallRbtArm(std_dh_params)
    controller = VelocityController(toolbox_robot)
    motion = MotionPatterns()
    # VelOptimizer(custom_robot, controller, controller.joint_limits['vel'])
    
    # Setup signal handling and initialize hardware
    setup_signal_handler(controller)
    sleep(1)  # Required delay after signal initialization
    controller.enable()
    sleep(1)
    # calibration
    # smallRobotArm.calibrate()
    # smallRobotArm.conn._event_ok2send.clear()    
    # print("[DEBUG] Waiting for ack...")
    # sleep(2)
    
    # Test Jacobian computation
    q_test = np.radians(controller.robot_rest_angles)
    J_custom = custom_robot.jacob0(q_test)
    J_toolbox = toolbox_robot.jacob0(q_test)
    print(f"Jacobian comparison - Custom: {J_custom.shape}, Toolbox: {J_toolbox.shape}")
    print(f"Max difference: {np.max(np.abs(J_custom - J_toolbox)):.6f}")
    
    # Desired end-effector velocity in world frame (ẋ = [vx, vy, vz, ωx, ωy, ωz])
    # my dhtbl is in mm, so here x,y,z in mm/s
    ''' 
    linear vel: vx, vy, vz; angular vel: wx, wy, wz
    '''
    x_dot = np.array([20.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # [mm/s, rad/s]
    # dt = 0.02  # control period (e.g. 50Hz)
    # q = np.radians(controller.current_angles)  # joint angles in radians    
    # controller.velocity_control(robot=smRobot, q_init=q, x_dot_func=lambda t: x_dot, dt=dt, duration=5.0)       
    
    # Move to circle center position
    p_dc = (-260, 100, 110.0, 0.0, 90.0, 0.0)  # Pose relative to desk frame
    # p_dc = (-250, 180, 100.0, 0.0, 90.0, 0.0)  # Pose relative to desk frame
    T_06 = custom_robot.convert_p_dc_to_T06(p_dc)
    j = custom_robot.ik(T_06)
    controller.move_to_angles(j)
    sleep(2)
    
    # move in x direction by 5mm (radius of circle)
    controller.cartesian_space_vel_ctrl(x_dot_func=lambda t: x_dot, duration=2.5)
   
    input("Press Enter to continue circle motion...") 
    '''
    w=2pi/t = 6.283185/15 rad/s, v=w x radius= 0.4189 rad/s x 5 ~= 1.04725cm/s
    15s seems max.
    linear vel=2pi * radius / 15s = 29.0mm/s
    '''
    
    controller.cartesian_space_vel_ctrl(x_dot_func=lambda t: motion.fourier_circle(t, radius=50, period=15), duration=15)
    
    # input("Press Enter to continue square vel...")
    # controller.velocity_control(q_init=np.radians(controller.current_angles), x_dot_func=lambda t: square_xz(t, side_length=50, period=15.0), dt=dt, duration=15.0)
    
    # zero_j = (0, 0, 0, 0, 0, 0)
    # controller.move_to_angles(zero_j, header="g", ack=True)
    # input("Press Enter to continue figure_eight vel...")
    # controller.velocity_control(   
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t:figure8(t, radius=30, freq=1.0),
    # dt=dt,
    # duration=110.0    
    # )
    
    controller.go_init()
    input("Press Enter to continue Zigzag pattern...")
    controller.cartesian_space_vel_ctrl(
        x_dot_func=lambda t: motion.zigzag(t, side_length=40, period=30.0),
        # x_dot_func=lambda t: motion.wave(t),
        duration=30.0
    )

    # controller.move_to_angles(zero_j, header="g", ack=True)
    # input("Press Enter to continue spiral vel...")
    # controller.velocity_control(  
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t:spiral(t, radius_rate=2.0, angular_speed=0.5),
    # dt=dt,
    # duration=30.0    
    # )

    # input("Press Enter to continue... square w/ corners vel")    
    # controller.velocity_control(
    # robot=smRobot,
    # q_init=np.radians(controller.current_angles),
    # x_dot_func=lambda t: controller.square_with_corners_velocity(
    #     t, side_length=100, edge_period=2.0, hover_duration=2.0, circle_radius=25.0, circle_period=2.0),
    # dt=0.05,
    # duration=4 * (4.0 + 2.0 + 2.0)  # 32s = 4 corners * (edge+hover+circle)
    # )
    
    input("Press Enter to go home...")   
    controller.go_home()
        
if __name__ == "__main__":
    sys.exit(main())

