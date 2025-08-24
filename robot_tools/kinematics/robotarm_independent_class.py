from abc import ABC, abstractmethod
# from typing import List, Tuple
import numpy as np
from numpy import ndarray
from spatialmath import SE3
# from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# robot arm independent, inherited by robot arm dependent classes.
class RobotArm(ABC):
    def __init__(self):
        # init arm independent params
        pass
    
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

        # return (np.round(position, 4), np.round(zyz_euler, 4))
        return tuple(np.round(np.concatenate([position, zyz_euler]), 4).tolist())

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
        # Homogeneous transformation matrix
        # T = np.eye(4)
        # T[:3, :3] = r.as_matrix()  # rotation_matrix
        # T[:3, 3] = [x, y, z]
        # return T
    
        x, y, z = pose[:3]
        alpha, beta, gamma = pose[3:6]
        # SO3.eul always zyz
        T = SE3(x, y, z) * SE3.Eul(alpha, beta, gamma, unit='deg')
        return T.A

    def get_t_0n(self, q, i_th_frame):
        # Compute transformation matrices T_0^i for each joint
        T = np.eye(4)
        # T_list = [T.copy()]  # T_0^0 = I
        
        for i in range(i_th_frame):
            alpha, a, d = self.dhTbl[i]
            theta = q[i] + self.th_offsets[i]  # Add offset
            
            # Standard DH transformation matrix
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha), np.sin(alpha)
            
            Ti = np.array([
                [ct, -st*ca, st*sa, a*ct],
                [st, ct*ca, -ct*sa, a*st],
                [0, sa, ca, d],
                [0, 0, 0, 1]
            ])
            
            T = T @ Ti
            # T_list.append(T.copy())  # T_0^(i+1)
        return T
         
    # interface
    @abstractmethod
    def ik(self, T_06:ndarray)->tuple:
        raise NotImplementedError("Subclasses must implement ik()")

    @abstractmethod
    def fk(self, Jfk):
        pass
    '''   
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
'''        
   
