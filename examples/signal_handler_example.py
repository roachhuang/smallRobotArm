"""
Example script demonstrating how to use the GracefulExitHandler.

This script shows how to integrate the signal handler into any robot arm application
to ensure graceful shutdown when Ctrl+C is pressed.
"""

import time
import logging
import sys
import os

# Add the parent directory to the path so we can import the robot_tools package
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_tools.misc.signal_handler import setup_signal_handler
from robot_tools.kinematics.robotarm_class import SmallRbtArm
import robot_tools.controller.robot_controller as controller
from roboticstoolbox import DHRobot, RevoluteDH
import numpy as np

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

def custom_cleanup():
    """Example custom cleanup function that can be passed to the signal handler."""
    logger.info("Performing custom cleanup tasks...")
    # Add any custom cleanup code here
    time.sleep(1)
    logger.info("Custom cleanup completed")

def main():
    logger.info("Starting example application")
    
    # Create a simple robot model for demonstration
    a1, a2, a3 = 47.0, 110.0, 26.0
    d1, d4, d6 = 133.0, 117.50, 28.0
    
    std_dh_table = [
        RevoluteDH(d=d1, a=a1, alpha=-np.pi / 2),
        RevoluteDH(d=0, a=a2, alpha=0, offset=-np.pi / 2),
        RevoluteDH(d=0, a=a3, alpha=-np.pi / 2),
        RevoluteDH(d=d4, a=0, alpha=np.pi / 2),
        RevoluteDH(d=0, a=0, alpha=-np.pi / 2),
        RevoluteDH(d=d6, a=0, alpha=0),
    ]
    
    # Create robot model
    robot_model = DHRobot(std_dh_table, name="smallRobotArm")
    
    # Create robot arm instance
    std_dh_params = np.array([
        [np.radians(-90), a1, d1],
        [0, a2, 0],
        [np.radians(-90), a3, 0],
        [np.radians(90), 0, d4],
        [np.radians(-90), 0, 0],
        [0, 0, d6],
    ])
    
    robot_arm = SmallRbtArm(std_dh_params)
    robot_controller = controller.RobotController(robot_model)
    robot_arm.controller = robot_controller
    
    # Set up the signal handler with our robot arm and custom cleanup function
    setup_signal_handler(robot_arm, custom_cleanup)
    
    logger.info("Signal handler registered. Press Ctrl+C to test graceful shutdown.")
    
    try:
        # Simulate a long-running process
        while True:
            logger.info("Robot is running... Press Ctrl+C to exit")
            time.sleep(2)
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())