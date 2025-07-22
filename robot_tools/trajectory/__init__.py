"""
Trajectory Planning Module for Small Robot Arm

This module provides trajectory planning functionality for the robot arm.
"""

from .plan_traj import planTraj
from .equations import eq1, eq2, eq3, eq4, eq5, eq6, eq7

__all__ = ['planTraj', 'eq1', 'eq2', 'eq3', 'eq4', 'eq5', 'eq6', 'eq7']