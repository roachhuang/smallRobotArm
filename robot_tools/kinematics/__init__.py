"""
Kinematics Module for Small Robot Arm

This module provides forward and inverse kinematics for the robot arm.
"""

from .robotarm_class import SmallRbtArm
from .std_dh_table import std_dh_tbl, std_dh_params
import modern_robotics as mr
__all__ = ['mr', 'SmallRbtArm', 'std_dh_tbl', 'std_dh_params']