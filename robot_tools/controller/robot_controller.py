"""Robot Controller Module

This module provides high-level control functionality for the small robot arm,
including velocity control, motion patterns, and direct joint control.

The controller handles communication with the hardware through a serial connection
and provides methods for various motion patterns and approach pose calculations.

Classes:
    RobotController: Main controller class for the robot arm
"""

import time
import numpy as np
from numpy import ndarray
from spatialmath import SE3
import robot_tools.serial.serial_class as ser

class RobotController:
    """Controller for the small robot arm.
    
    This class provides methods to control the robot arm, including velocity control,
    motion patterns, and direct joint control. It handles communication with the
    Arduino hardware through a serial connection.
    
    Attributes:
        robot_rest_angles (tuple): Default rest position angles for the robot
        current_angles (tuple): Current joint angles of the robot
        robot: Robot model used for kinematics calculations
        conn (SerialPort): Serial connection to the Arduino
    """
    
    def __init__(self, robot):
        self.robot_rest_angles = (0.0, -78.5, 73.9, 0.0, -90.0, 0.0)       
        self.current_angles = self.robot_rest_angles
        self.robot = robot
        self.conn = ser.SerialPort()
        self.conn.connect()
    
    def velocity_control(self, robot, q_init, x_dot_func, duration, dt=0.05):
        """Control robot using velocity control in task space.
        
        This method implements Jacobian-based velocity control, converting desired
        end-effector velocities to joint velocities using the damped pseudoinverse
        of the Jacobian matrix.
        
        Args:
            robot: Robot model with jacobian method
            q_init (ndarray): Initial joint angles in radians
            x_dot_func (callable): Function that takes time and returns 6D velocity vector
            duration (float): Duration of motion in seconds
            dt (float): Time step for control loop in seconds
            
        Returns:
            ndarray: Final joint angles in degrees
        """
        q = np.array(q_init, dtype=np.float64)  # Ensure q is a float array
        n_steps = int(duration / dt)
        max_qdot = np.radians(30)  # 30°/sec max joint velocity. in accrod w/ firmware's MAX_SPEED.
        for i in range(n_steps):
            J = robot.jacob0(q)
            # Check manipulability (measure of singularity)
            manipulability = np.sqrt(np.linalg.det(J @ J.T))            
            if manipulability < 0.01:  # Near singularity
                # Use damped least squares with adaptive damping
                damping = 0.1 * (0.01 - manipulability) / 0.01
                J_plus = J.T @ np.linalg.inv(J @ J.T + damping**2 * np.eye(6))
            else:
                # Standard pseudoinverse
                J_plus = np.linalg.pinv(J)
            x_dot = x_dot_func(i * dt)  # Desired end-effector vel in base/world frame (6D)
            q_dot = J_plus @ x_dot
            q_dot = np.clip(q_dot, -max_qdot, max_qdot)
            q += q_dot * dt  # Integrate joint velocity
            q_deg = np.degrees(q)

            self.move_to_angles(q_deg, header='g', ack=True)  # Send angles to robot arm

            time.sleep(dt)
        return q_deg
        
    def square_xz_velocity(self, t, side_length, period):
        """Generate velocity vector for square motion in XZ plane.
        
        Args:
            t (float): Current time in seconds
            side_length (float): Length of square side in mm
            period (float): Time to complete full square in seconds
            
        Returns:
            ndarray: 6D velocity vector [vx, vy, vz, wx, wy, wz]
        """
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

    def figure_eight_velocity(self, t, radius=30, freq=1.0):
        """Generate velocity vector for figure-eight pattern in XY plane.
        
        Creates a figure-eight pattern by using sine for x-axis motion with frequency f
        and cosine for y-axis motion with frequency 2f.
        
        Args:
            t (float): Current time in seconds
            radius (float): Size of figure-eight pattern in mm
            freq (float): Frequency of pattern in Hz (cycles per second)
            
        Returns:
            ndarray: 6D velocity vector [vx, vy, vz, wx, wy, wz]
        """
        v = np.zeros(6)
        v[0] = -radius * 2 * np.pi * freq * np.sin(2 * np.pi * freq * t)      # x
        v[1] = radius * 2 * np.pi * freq * np.cos(4 * np.pi * freq * t)       # y
        return v
    
    def spiral_velocity(self, t, radius_rate=2.0, angular_speed=0.5):
        """Generate velocity vector for spiral motion in XY plane.
        
        Args:
            t (float): Current time in seconds
            radius_rate (float): Rate of spiral expansion in mm/s
            angular_speed (float): Angular velocity in rad/s
            
        Returns:
            ndarray: 6D velocity vector [vx, vy, vz, wx, wy, wz]
        """
        r = radius_rate * t
        v = np.zeros(6)
        v[0] = -r * angular_speed * np.sin(angular_speed * t) + radius_rate * np.cos(angular_speed * t)  # x
        v[1] = r * angular_speed * np.cos(angular_speed * t) + radius_rate * np.sin(angular_speed * t)   # y
        return v

    def zigzag_velocity(self, t, side_length=40, period=10.0):
        """Generate velocity vector for zigzag motion in XY plane.
        
        Args:
            t (float): Current time in seconds
            side_length (float): Length of zigzag segment in mm
            period (float): Time to complete one zigzag cycle in seconds
            
        Returns:
            ndarray: 6D velocity vector [vx, vy, vz, wx, wy, wz]
        """
        v = np.zeros(6)
        step = int((t / (period / 4)) % 2)
        v[0] = side_length / (period / 4) if step == 0 else -side_length / (period / 4)
        v[1] = 10 * np.sin(4 * np.pi * t / period)  # Adds jagged pattern
        return v


    def square_with_corners_velocity(self, t, side_length=80, edge_period=2.0, hover_duration=2.0,  circle_radius=20.0, circle_period=2.0):
        """
        Move along square edges with cosine-blended velocity.
        Pause (hover) at corners for N seconds.
        At each corner, trace a small circle (e.g., 20mm radius) before continuing.
        Produces velocity vector that follows a square path with:
        - cosine-blended edges,
        - circular motion at corners,
        - hover pauses.

        Total duration per corner segment = edge_period + hover_duration + circle_period
        Square has 4 corners, so total = 4 * (edge + hover + circle)
        """
        total_phase_time = edge_period + hover_duration + circle_period
        full_duration = total_phase_time * 4
        t = t % full_duration  # Loop over full square

        corner = int(t // total_phase_time)
        local_t = t % total_phase_time

        v = np.zeros(6)  # 6D vel

        # Phase 1: move along edge (cosine-blended)
        if local_t < edge_period:
            progress = local_t / edge_period
            blend = 0.5 * (1 - np.cos(np.pi * progress))  # cosine blend 0 → 1
            speed = side_length / edge_period

            if corner == 0:   # → along X
                v[0] = speed * blend
            elif corner == 1: # ↓ along Z
                v[2] = -speed * blend
            elif corner == 2: # ← along X
                v[0] = -speed * blend
            elif corner == 3: # ↑ along Z
                v[2] = speed * blend

        # Phase 2: hover
        elif local_t < edge_period + hover_duration:
            return np.zeros(6)

        # Phase 3: circle at corner (in XY plane)
        else:
            circle_t = local_t - (edge_period + hover_duration)
            omega = 2 * np.pi / circle_period
            blend = 0.5 * (1 - np.cos(2 * np.pi * circle_t / circle_period))  # circle blend

            v[0] = -circle_radius * omega * np.sin(omega * circle_t) * blend
            v[1] =  circle_radius * omega * np.cos(omega * circle_t) * blend

        return v
    
    def fourier_circle_velocity(self, t, radius=40, period=15.0, harmonics=1):
        ''' harmonics = 1 for circle
        Parameters:
            - t: current time (seconds)
            - radius: radius of the circle (mm)
            - period: time to complete one full circle (seconds)

        Returns:
            - x_dot: 6D end-effector velocity in base frame [vx, vy, vz, wx, wy, wz]
        '''
        omega = 2 * np.pi / period  # angular velocity (rad/s)
        speed = omega * radius      # linear  velocity (mm/s)
        x_dot = np.zeros(6)
        vx, vy = 0, 0

        for n in range(1, harmonics + 1):
            # Tangent velocity on the circle
            vx += -n * speed * np.sin(n * omega * t)
            vy +=  n * speed * np.cos(n * omega * t)

        x_dot[0] = vx   # x direction
        x_dot[1] = vy   # y direction
        # Z (x_dot[2]) remains 0 → fixed height
        return x_dot
    
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

    def eigen_analysis(self, smRobot, q):
        """Perform eigenvalue analysis of robot Jacobian and inertia matrices.
        
        This analysis helps identify optimal motion directions and potential singularities.
        
        Args:
            q (ndarray): Joint angles in radians
            
        Returns:
            dict: Dictionary containing analysis results including optimal directions
                 and manipulability measure
        """
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
    
        
    def compute_approach_pose(self, T_cup, approach_vec_cup, offset=50):
        """
        Compute an approach pose with a specified offset from the target.
        
        Args:
            T_cup: 4x4 transformation matrix of the cup/target
            approach_vec_cup: 3D vector defining approach direction in cup frame
            offset: distance in mm to offset from the target
        
        Returns:
            4x4 transformation matrix for the approach pose
        """
        # Extract rotation and position from target transformation
        R_cup = T_cup[0:3, 0:3]
        p_cup = T_cup[0:3, 3]

        # Transform approach vector to base frame
        approach_vec_base = R_cup @ approach_vec_cup
        
        # Calculate position with offset
        p_tcp = p_cup + offset * approach_vec_base
        
        # Calculate orientation (z-axis aligned with approach vector)
        z_tcp = -approach_vec_base  # Negative because we approach opposite to vector
        
        # Find perpendicular vectors to form coordinate frame
        world_up = np.array([0, 0, 1])
        x_tcp = np.cross(world_up, z_tcp)
        if np.linalg.norm(x_tcp) < 1e-3:
            # Handle singularity if approach is parallel to world up
            world_up = np.array([0, 1, 0])
            x_tcp = np.cross(world_up, z_tcp)
        x_tcp /= np.linalg.norm(x_tcp)
        y_tcp = np.cross(z_tcp, x_tcp)

        # Construct rotation matrix and transformation
        R_tcp = np.column_stack((x_tcp, y_tcp, z_tcp))
        T_tcp = np.eye(4)
        T_tcp[0:3, 0:3] = R_tcp
        T_tcp[0:3, 3] = p_tcp
        
        return T_tcp
        
    def grab(self):
        """Activate the gripper to grab an object."""
        self.conn.ser.write(b"eOn\n")

    def drop(self):
        """Deactivate the gripper to release an object."""
        self.conn.ser.write(b"eOff\n")

    def enable(self):
        """Enable all motors on the robot arm."""
        # motors are disabled in arduino's setup()
        self.conn.ser.write(b"en\n")       

    def calibrate(self):
        """Calibrate the robot by homing all axes."""
        self.conn.ser.write(b"g28\n")  # G28 is the command to home the robot arm
        
    def disable(self):
        """Disable all motors on the robot arm."""
        self.conn.ser.write(b"dis\n")

    def move_to_angles(self, j: tuple, header='g', ack=True) -> None:
        """Move the robot to specified joint angles.
        
        Args:
            j (tuple): Target joint angles in degrees
            header (str): Command header for Arduino protocol
            ack (bool): Whether to wait for acknowledgment from Arduino
        """
        # return super().moveTo(end_effector_pose)
        # limited_j = self.limit_joint_angles(j)
        cmd = {"header": header, "joint_angle": j, "ack": ack}
        self.conn.send2Arduino(cmd)
        # naively assume motors moved accordinly. remove this line if motors have encoder.
        self.current_angles = j
        
    def go_home(self):
        """Move the robot to its home/rest position and disconnect.
        
        This is typically called before shutting down the system.
        """
        print('gohome')
        self.move_to_angles(j=self.robot_rest_angles)
        # self.current_angles = self.robot_rest_angles
        # self.disable()
        # a way to terminate thread
        self.conn.disconnect()
    
