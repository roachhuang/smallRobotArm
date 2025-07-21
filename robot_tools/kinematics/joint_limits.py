from typing import Tuple, List
import numpy as np

class JointLimits:
    """Class for handling robot joint limits and angle constraints."""
    
    def __init__(self, max_limits: Tuple[float, ...], min_limits: Tuple[float, ...], offsets: Tuple[float, ...] = None):
        """
        Initialize joint limits handler.

        Args:
            max_limits: Maximum angle limits for each joint in degrees
            min_limits: Minimum angle limits for each joint in degrees
            offsets: Optional joint angle offsets in radians
        """
        if len(max_limits) != len(min_limits):
            raise ValueError("Max and min limit lists must have the same length")
            
        self.max_limits = max_limits
        self.min_limits = min_limits
        self.offsets = offsets if offsets is not None else tuple([0.0] * len(max_limits))
        
    def limit_joint_angles(self, angles: List[float]) -> List[float]:
        """
        Limits joint angles to specified max/min values.
        
        Args:
            angles: List of joint angles to be limited
            
        Returns:
            List of limited joint angles
        """
        if len(angles) != len(self.max_limits):
            raise ValueError("Angle and limit lists must have the same length")
            
        return [
            max(min(a, max_val), min_val)
            for a, max_val, min_val in zip(angles, self.max_limits, self.min_limits)
        ]
        
    def apply_offsets(self, angles: List[float], to_radians: bool = True) -> List[float]:
        """
        Apply joint angle offsets.
        
        Args:
            angles: List of joint angles
            to_radians: If True, converts result to radians
            
        Returns:
            List of angles with offsets applied
        """
        result = [a + o for a, o in zip(angles, self.offsets)]
        if to_radians:
            return [np.deg2rad(a) for a in result]
        return result