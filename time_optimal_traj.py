from time import sleep
import logging
import numpy as np
import sys   
from robot_tools.kinematics import SmallRbtArm, std_dh_params
from robot_tools.controller import RobotController
from robot_tools.misc.signal_handler import setup_signal_handler
import matplotlib.pyplot as plt

def main() -> None:
    logging.basicConfig()
    np.set_printoptions(precision=2, suppress=True)
  
    # create an instance of the robotarm.
    smallRobotArm = SmallRbtArm(std_dh_params)
    controller = RobotController(smallRobotArm)
    setup_signal_handler(controller)                    
   
    # there must be a delay here right after sieal is initialized
    sleep(1)  # don't remove this line!!!
    controller.enable()  # enable the robot arm
    sleep(1)
    
    joint_limits = {
    'vel': [np.radians(25)]*6,
    'acc': [2.0]*6, # 1.0 ~3.0 rad/s^2
    'jerk': [10]*6, # 5.0 ~20.0 rad/s^3, higher jerk=snappier motion, but more mechanical stress.
    }
    
    timestamps=[0,8,20,24]
    # dt_list = [T1-T0, T2-T1, T3-T2]: 3 elements. Durations of the segments
    dt_list=[timestamps[i]-timestamps[i-1] for i in range(1,len(timestamps))]   # 8,12,4
    # cartesian_path = [WP0, WP1, WP2, WP3]: 4 elements. Cartesian waypoints.
    cartesian_path = np.array(
        [
            [-150,        190, 60, 0.0, 0.0, 35.0],
            [-164.5 + 19, 190, 90, 0.0, 0.0, 35.0],
            # rotate cup 60 degrees around y axis wrt the world frame.
            [-164.5 - 120, 170.0 + 60, 350.0, 0, -60.0, 0.0],
            [-164.5 - 120, 170.0 + 100, 355.0, 0, -60.0, 0.0],
        ],
        dtype=np.float64,
    )

    # this is only for giving time col to joints
    # joint_path = cartesian_path.copy()
    
    # the pose at 0s. ntu: fixed euler anglers
    p_dc = cartesian_path[0]
    # Convert to transformation matrix
    T_06_at_0s = smallRobotArm.convert_p_dc_to_T06(p_dc)
    j = smallRobotArm.ik(T_06_at_0s)
    controller.move_to_angles(j)    
    input("Press Enter to start time optimal trajectory...")
    
    # joint_path = [J0, J1, J2, J3]: 4 elements. Joint angles corresponding to cartesian_path.
    joint_path=[smallRobotArm.ik(smallRobotArm.convert_p_dc_to_T06(pose)) for pose in 
                cartesian_path]
    # now in rad
    joint_path=np.array(np.radians(joint_path))
    
    # 3. Estimate Raw Velocities, defined per segment (between waypoints),
    raw_velocities = []
    for i in range(1, len(joint_path)):
        dq = joint_path[i] - joint_path[i-1]
        dt = dt_list[i-1]
        raw_velocities.append(dq / dt)
    # 2. Insert zero initial velocity
    # raw_velocities.insert(0, np.zeros_like(raw_velocities[0]))
    # raw_velocities[0] should be (J1 - J0) / dt_list[0] (for segment 0-1), and so on...
    raw_velocities = np.array(raw_velocities)  # shape: (3, 6)
    
    # Print output (for testing)
    print("Timestamps:", timestamps)
    print("dt_list:", dt_list)
    print("Raw velocities:\n", raw_velocities)
    
    # Plot raw velocities
    plt.plot(raw_velocities)
    plt.title("Raw Joint Velocities")
    plt.xlabel("Segment")
    plt.ylabel("Velocity (rad/s)")
    plt.legend([f'Joint {i+1}' for i in range(6)])
    plt.show()

    # 6. send velocity cmds in ctrl loop
    for i, duration_seg in enumerate(dt_list):
        q_dot = raw_velocities[i]        
        print(f"\nSending velocity {q_dot} for {duration_seg} seconds")
        controller.joint_space_vel_ctrl(q_dot_raw=q_dot, duration=duration_seg)
        
    # for i, v in enumerate(smoothed_velocities, start=1):
    #     controller.velocity_control(
    #         x_dot_func=lambda t, v=v: v,
    #         duration=dt_list[i-1]
    #     )

    controller.go_home()
    
if __name__ == "__main__":
    sys.exit(main())

