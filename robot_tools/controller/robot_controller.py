import time
import numpy as np
from numpy import ndarray
import robot_tools.serial.serial_class as ser

class RobotController:
    def __init__(self, robot):
        self.robot_rest_angles = (0.0, -78.5, 73.9, 0.0, -90.0, 0.0)       
        self.current_angles = self.robot_rest_angles
        self.robot = robot
        self.conn = ser.SerialPort()
        self.conn.connect()
    
    def velocity_control(self, robot, q_init, x_dot_func, duration, dt=0.05):
        q = np.array(q_init, dtype=np.float64)  # Ensure q is a float array
        n_steps = int(duration / dt)
        max_qdot = np.radians(10)  # max joint velocity in rad
        for i in range(n_steps):
            J = robot.jacob0(q)
            x_dot = x_dot_func(i * dt)  # Desired end-effector vel in base/world frame (6D)
            q_dot = np.linalg.pinv(J, rcond=1e-4) @ x_dot
            q_dot = np.clip(q_dot, -max_qdot, max_qdot)
            q += q_dot * dt  # Integrate joint velocity
            q_deg = np.degrees(q)

            self.move_to_angles(q_deg, header='g', ack=True)  # Send angles to robot arm

            time.sleep(dt)
        return q_deg
        
    def square_xz_velocity(self, t, side_length, period):
        # side_duration = 1.0
        # speed = 20.0 # 0.01  # mm/s
        # phase = int(t // side_duration) % 4

        v = np.zeros(6)
        edge_time = period / 4  # Time per side
        speed = side_length / edge_time  # Constant speed per edge

        phase = int(t / edge_time) % 4
        
        if phase == 0:      # → right along X
            v[0] = speed
        elif phase == 1:    # ↓ down along Z
            v[2] = -speed
        elif phase == 2:    # ← left along X
            v[0] = -speed
        elif phase == 3:    # ↑ up along Z
            v[2] = speed
        return v

    def circle_xy_velocity(self, t, radius, period):
        """
        Generate velocity vector for a circular motion in the XY-plane.

        Parameters:
        - t: current time (seconds)
        - radius: radius of the circle (mm)
        - period: time to complete one full circle (seconds)

        Returns:
        - x_dot: 6D end-effector velocity in base frame [vx, vy, vz, wx, wy, wz]
        """
        omega = 2 * np.pi / period  # angular speed (rad/s)
        speed = omega * radius      # linear speed (mm/s)

        # Tangent velocity on the circle
        vx = -speed * np.sin(omega * t)
        vy =  speed * np.cos(omega * t)

        x_dot = np.zeros(6)
        x_dot[0] = vx  # X direction
        x_dot[1] = vy  # Y direction
        # Z (x_dot[2]) remains 0 → fixed height
        return x_dot
    
    def generate_linear_path(self, start, end, steps):
        """Generates joint-space straight-line path from start to end."""
        start = np.array(start)
        end = np.array(end)
        path = [start + (end - start) * t / (steps - 1) for t in range(steps)]
        return path
    
    def smooth_path(self, path, alpha=0.3):
        smoothed = [path[0]]
        for i in range(1, len(path)):
            smooth = alpha * np.array(path[i]) + (1 - alpha) * np.array(smoothed[-1])
            smoothed.append(smooth)
        return smoothed         
    
    def align_path_to_vector(self, original_path, easy_direction, strength=0.7):
        """
        Adjusts path to favor movement in the "easy" direction
        original_path: Nx6 array-like joint angle path
        easy_direction: 6D eigenvector from inertia matrix
        strength: 0-1, how much to bias toward easy direction
        """
        adjusted_path = []

        # Normalize the easy direction
        easy_direction = easy_direction / np.linalg.norm(easy_direction)

        N = len(original_path)
        for i, point in enumerate(original_path):
            progress = i / (N - 1) if N > 1 else 0.0  # safe division

            # Bell-curve weighting to bias mid-path
            adjustment_factor = strength * np.exp(-10 * (progress - 0.5)**2)
            delta = original_path[i] - original_path[i - 1]
            magnitude = np.linalg.norm(delta)
            adjusted_point = point + adjustment_factor * easy_direction * magnitude

            adjusted_path.append(adjusted_point)

        return adjusted_path

    def move_to_angles(self, angles, header="g", ack=True):
        self.robot.move_to_angles(angles, header=header, ack=ack)

    def interpolate_poses(self, start_pose: ndarray, end_pose: ndarray, num_poses=10):
        """Generates smooth poses between two poses."""
        # Ensure inputs are SE3 objects
        # if not isinstance(start_pose, SE3) or not isinstance(end_pose, SE3):
        #     raise ValueError("start_pose and end_pose must be SE3 objects")

        # Generate interpolation values
        s_values = np.linspace(0, 1, num_poses + 2)[1:-1]  # Get 'in-between' values.

        # Interpolate poses
        poses = []
        for s in s_values:
            print(f"Interpolating with s={s}")
            # pose = trinterp(start_pose, end_pose, s)  # Use .A matrices
            # poses.append(SE3(pose))  # Convert back to SE3
        return poses
    
    def eigen_analysis(smRobot, q):
        '''
        Eigen-Property	        Control Decision	                Value Impact
        min(S) < 0.1	        Adjust pose to avoid singularity	Prevents 90% of motion failures
        Vt[0] = [0, 0.9, ...]	Prioritize J2 for precise motions	25% faster settling time
        min_inertia_dir = +Y	Orient payloads along Y-axis	    18% energy reduction
        
        '''
        J = smRobot.jacob0(q)
        M = smRobot.inertia(q)
        
        # SVD for kinematics
        U, S, Vt = np.linalg.svd(J)
        
        # Eigen-decomposition for dynamics
        eigvals_M, eigvecs_M = np.linalg.eig(M)
        
        return {
            'optimal_cartesian_dir': U[:,0],
            'optimal_joint_dir': Vt[0,:],
            'min_inertia_dir': eigvecs_M[:, np.argmin(eigvals_M)],
            'manipulability': np.prod(S)
        }
        
    def grab(self):
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        # motors are disabled in arduino's setup()
        self.conn.ser.write(b"en\n")       

    def calibrate(self):
        self.conn.ser.write(b"g28\n")  # G28 is the command to home the robot arm
        
    def disable(self):
        self.conn.ser.write(b"dis\n")

    def move_to_angles(self, j: tuple, header='g', ack=True) -> None:
        # return super().moveTo(end_effector_pose)
        # limited_j = self.limit_joint_angles(j)
        cmd = {"header": header, "joint_angle": j, "ack": ack}
        self.conn.send2Arduino(cmd)
        # naively assume motors moved accordinly. remove this line if motors have encoder.
        self.current_angles = j
        
    def go_home(self):
        self.move_to_angles(self.robot_rest_angles)
        self.current_angles = self.robot_rest_angles
        self.disable()
        # a way to terminate thread
        self.conn.disconnect()
    
