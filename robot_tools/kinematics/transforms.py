import numpy as np
from numpy import ndarray
from spatialmath import SE3

class TransformationUtils:
    """Utility class for handling coordinate transformations and pose conversions."""
    
    @staticmethod
    def T2Pose(T: ndarray, seq="xyz", degrees=True) -> tuple:
        """
        Converts a 4x4 transformation matrix to a pose (position and orientation).

        Args:
            T: 4x4 NumPy array representing the transformation matrix.
            seq: Euler angle sequence (e.g., "xyz", "zyz").
            degrees: If True, returns Euler angles in degrees; otherwise, in radians.

        Returns:
            A tuple containing position and euler angles (x,y,z,rx,ry,rz)
        """
        SE3_T = SE3(T)
        position = SE3_T.t
        zyz_euler = SE3_T.eul(unit="deg" if degrees else "rad")
        return tuple(np.round(np.concatenate([position, zyz_euler]), 4).tolist())

    @staticmethod
    def pose2T(pose: tuple, seq="xyz") -> ndarray:
        """
        Convert position and orientation to transformation matrix.

        Args: 
            pose: tuple containing (x,y,z) position and 3 rotation angles in ZYZ order
            seq: Euler angle sequence
        Returns: 
            4x4 transformation matrix
        """
        x, y, z = pose[:3]
        alpha, beta, gamma = pose[3:6]
        T = SE3(x, y, z) * SE3.Eul(alpha, beta, gamma, unit='deg')
        return T.A

    @staticmethod
    def dh_transform(theta: float, alpha: float, a: float, d: float) -> ndarray:
        """
        Creates a DH transformation matrix using standard DH parameters.

        Args:
            theta: Joint angle in radians
            alpha: Link twist angle in radians
            a: Link length
            d: Link offset

        Returns:
            4x4 transformation matrix
        """
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ], dtype=np.float64)