from abc import ABC, abstractmethod
from typing import List, Tuple
import numpy as np
from numpy import ndarray
from spatialmath import SE3
import serial_class as ser
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# robot arm independent, inherited by robot arm dependent classes.
class RobotArm(ABC):
    def __init__(self):
        # init arm independent params
        self.conn = ser.SerialPort()
        self.conn.connect()        
    
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

    
    def T2Pose(self, T, seq="xyz", degrees=True) -> tuple:
        """
        Converts a 4x4 transformation matrix to a pose (position and orientation).

        Args:
            T: 4x4 NumPy array representing the transformation matrix.
            euler_seq: Euler angle sequence (e.g., "xyz", "zyz").
            degrees: If True, returns Euler angles in degrees; otherwise, in radians.

        Returns:
            A tuple: (position, euler_angles)
            position: a 3 element numpy array.
            euler_angles: a 3 element numpy array.
        """
        # position = T[:3, 3]  # Extract position (translation)
        # rotation_matrix = T[:3, :3]  # Extract rotation matrix

        # rotation = R.from_matrix(rotation_matrix)
        # euler_angles = rotation.as_euler(seq=seq, degrees=degrees)

        # return (position, euler_angles)
    
        SE3_T = SE3(T)
        position = SE3_T.t  # Extract position as a list
        zyz_euler = SE3_T.eul(
            unit="deg"
        )  # Extract ZYZ Euler angles in radians

        return (np.round(position, 4), np.round(zyz_euler, 4))


    def pose2T(self, pose: tuple, seq="xyz") -> ndarray:
        """
        Args: position (x,y,z) + 3 rotation angles in ZYZ order in degree
        Returns: transformation matrix from pose

        The three rotations can either be in a global frame of reference (extrinsic) or in a body centred frame of reference (intrinsic),
        which is attached to, and moves with, the object under rotation [1].
        Robots with spherical wrists (where the last three joint axes intersect at a point)
        a.k.a. a4=a5=a6=0 (std dh tbl) often use "ZYZ" to represent the orientation of the wrist.
        NOTE: uppercase 'ZYZ' for intrinsic rotation; lowercase->fixed angles sequence
        """
        # avoid naming roll, pitch and yaw with zyz coz of misleading
        # x, y, z, psi, theta, phi = pose
        # r = R.from_euler(seq=seq, angles=[psi, theta, phi], degrees=True)

        # Translation matrix
        # translation_matrix = np.eye(4)
        # translation_matrix[:3, 3] = [x, y, z]

        # Homogeneous transformation matrix
        # T = np.eye(4)
        # T[:3, :3] = r.as_matrix()  # rotation_matrix
        # T[:3, 3] = [x, y, z]

        # Alternatively, you can multiply the translation and rotation matrix.
        # T = translation_matrix @ np.eye(4)
        # T[:3,:3] = rotation_matrix

        # return T
    
        x, y, z = pose[:3]
        alpha, beta, gamma = pose[3:6]
        # SO3.eul always zyz
        T = SE3(x, y, z) * SE3.Eul(alpha, beta, gamma, unit='deg')
        return T.A

    def get_ti2i_1(self, i, theta=None) -> ndarray:
        """
        todo: when theta will be none? examin intput theta's procision by checking its number of decimal points.
        Creates a DH transformation matrix using NumPy.

        Args:
            th: Theta (joint angle or variable). muse be in rad coz np.sin/cos... take rad.
            alfa: Alpha (twist angle).
            ai: ai (link length).
            di: di (link offset).
            theta: If None, returns a rounded matrix (approximating symbolic).
            tolerance: Tolerance for rounding (used if theta is None).

        Returns:
            NumPy array representing the DH transformation matrix.
        """
        # fill in dh tbl wrt robot arms' dh params

        # array idx starts frm 0, so i-1
        # alfa, ai, di, th = symbols("alfa, ai, di, th")
        alfa, ai, di = self.dhTbl[i - 1, :]
        # th = f"q{i}" if theta is None else theta
        th = theta

        # the reason for using sympy's Matrix is that we need to apply it with sympy's simplify func
        # to eliminate sth likes 1.xxxxxe-14 * sin(qx)
        # Ti_2_i-1=Tzi-1(thetai)TZr(di)TXq(ai)TXp(alphai)
        # this matrix is transformation matrix for std dh table
        # m = Matrix(
        #     [
        #         [cos(th), -sin(th) * cos(alfa), sin(th) * sin(alfa), ai * cos(th)],
        #         [sin(th), cos(th) * cos(alfa), -cos(th) * sin(alfa), ai * sin(th)],
        #         [0, sin(alfa), cos(alfa), di],
        #         [0, 0, 0, 1],
        #     ]
        # )

        """ standard DH table's T_i_to_i-1 coz smallrobotarm is uing std DH tbl.
            the m matrix will be difference if using craig dh table. see ntu 3-3
            Tzi-1(th_i)@Tzr(di)@Txq(ai)@Txp(alpha_i)
        """
        m = np.array(
            [
                [
                    np.cos(th),
                    -np.sin(th) * np.cos(alfa),
                    np.sin(th) * np.sin(alfa),
                    ai * np.cos(th),
                ],
                [
                    np.sin(th),
                    np.cos(th) * np.cos(alfa),
                    -np.cos(th) * np.sin(alfa),
                    ai * np.sin(th),
                ],
                [0, np.sin(alfa), np.cos(alfa), di],
                [0, 0, 0, 1],
            ]
        )
        # todo: find out when theta will be None and why?
        if theta is None:
            # m = nsimplify(m, tolerance=1e-10, rational=True)
            # # print(f't{i}-{i-1}: {m}')
            # return np.array(m)
            # ------------------------------------------------------------
            # For symbolic-like behavior (approximation with rationals)
            # NumPy doesn't have a direct equivalent to nsimplify with rational=True
            # This part requires more advanced techniques or a different library if you need exact rational approximations.
            # Here we approximate with floats and round to a certain tolerance.
            decimals = int(-np.log10(1e-8))  # calculates the number of decimals needed
            # 6-10 decimals is common for robotics, we use 8 here.
            m = np.round(m, decimals)  # approximates the symbolic nsimplify tolerance.
            return m
        else:
            # Matrix objects have a numpy method that returns a numpy.ndarray
            # float32 maintains only approximately 7 decimal digits fo precision internally.
            return m.astype(np.float64)
    
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
        
    def go_home(self):
        self.move_to_angles(self.robot_rest_angles)
        self.disable()
        # a way to terminate thread
        self.conn.disconnect()
    
    def plot_frame_coordinates(self, smRobot, my_j, ik_sol):
        # Plot the frame coordinate with customization
        smRobot.plot(np.radians(my_j), backend='pyplot', block=False, jointaxes=True)
        # Get the current axes from robot.plot()          
        ax = plt.gca()                        
        fkine_all_T = smRobot.fkine_all(ik_sol.q)
        # Plot coordinate frames for each link
        for i, t in enumerate(fkine_all_T):
            T_arr = t.A
            smRobot.trplot(T_arr, ax=ax, width=2, length=20, color=('r','g','b'))
                            
        # Rotate the view for better visibility (optional)
        # ax.view_init(elev=30, azim=45)  # Adjust as needed
        plt.draw()
        plt.pause(0.1)  # w/o this line frame coordiantes won't be displayed.
        
        
    # interface
    @abstractmethod
    def ik(self, T_06:ndarray)->tuple:
        raise NotImplementedError("Subclasses must implement ik()")

    @abstractmethod
    def fk(self, Jfk):
        pass

    # @abstractmethod
    # def moveTo(self, end_effector_pose):
    #     pass