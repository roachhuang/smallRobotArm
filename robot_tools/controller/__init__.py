"""Controller Package

This package provides various controller implementations for the small robot arm.

Classes:
    BaseController: Base controller with common functionality
    VelocityController: Velocity-based control methods
    PositionController: Position-based control methods
    RobotController: Legacy combined controller (for backward compatibility)
"""

from .base_controller import BaseController
from .velocity_controller import VelocityController
from .position_controller import PositionController
from .optimal_motion_refactored import OptimalMotionAnalyzer
# Legacy import for backward compatibility
from .robot_controller import RobotController

__all__ = [
    'BaseController',
    'VelocityController', 
    'PositionController',
    'RobotController',
    'OptimalMotionAnalyzer',
]