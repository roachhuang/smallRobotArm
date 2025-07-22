"""
Miscellaneous Utilities for Small Robot Arm

This module provides helper functions and utilities for the robot arm.
"""

from .signal_handler import setup_signal_handler, GracefulExitHandler

__all__ = ['setup_signal_handler', 'GracefulExitHandler']