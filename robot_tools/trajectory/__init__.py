"""
Trajectory Planning Module for Small Robot Arm

This module provides trajectory planning functionality for the robot arm.
"""

from .traj_plan_lfpb import plan_traj_with_lfpb
from .interpolation import Interp
from .equations import eq1, eq2, eq3, eq4, eq5, eq6, eq7
from .motion_patterns import *
__all__ = ['plan_traj_with_lfpb', 'Interp', 'eq1', 'eq2', 'eq3', 'eq4', 'eq5', 'eq6', 'eq7', 'fourier_circle_velocity', 'square_xz_velocity', 'figure_eight_velocity', 'spiral_velocity', 'zigzag_velocity', 'wave']