"""
Signal handler module for graceful shutdown of robot arm applications.

This module provides a reusable signal handler for Ctrl+C (SIGINT) that ensures
the robot arm returns to a safe position before shutting down the application.
"""

import signal
import os
import logging
from typing import Optional, Any, Callable

logger = logging.getLogger(__name__)

class GracefulExitHandler:
    """
    A class to handle graceful exit when Ctrl+C is pressed.
    
    This handler ensures that:
    1. The robot arm returns to a safe home position
    2. Any active threads are properly joined
    3. The program exits cleanly
    
    Usage:
        handler = GracefulExitHandler(robot_arm)
        handler.register()
        
        # Your program code here
        
        # Optional: Unregister if needed
        handler.unregister()
    """
    
    def __init__(self, robot_arm=None, custom_cleanup_func: Optional[Callable] = None):
        """
        Initialize the handler with a robot arm instance.
        
        Args:
            robot_arm: The robot arm instance that has a controller with go_home method
            custom_cleanup_func: Optional custom cleanup function to call before exit
        """
        self.robot_arm = robot_arm
        self.custom_cleanup_func = custom_cleanup_func
        self.original_handler = None
    
    def register(self):
        """Register this handler for SIGINT (Ctrl+C)."""
        self.original_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._handler)
        logger.debug("Graceful exit handler registered")
    
    def unregister(self):
        """Restore the original signal handler."""
        if self.original_handler:
            signal.signal(signal.SIGINT, self.original_handler)
            logger.debug("Original signal handler restored")
    
    def _handler(self, signum: int, frame: Any):
        """
        Signal handler to gracefully exit the program on Ctrl+C.
        
        Args:
            signum: Signal number
            frame: Current stack frame
        """
        print("\nSignal received, exiting gracefully...")
        
        # Call custom cleanup function if provided
        if self.custom_cleanup_func:
            try:
                self.custom_cleanup_func()
            except Exception as e:
                logger.error(f"Error in custom cleanup function: {e}")
        
        # Handle robot arm cleanup
        if self.robot_arm is not None:
            try:
                self.robot_arm.go_home()
                # Move robot to home position
                # if hasattr(self.robot_arm, 'controller') and hasattr(self.robot_arm.controller, 'go_home'):
                #     self.robot_arm.go_home()
                # else:
                #     logger.warning("Robot arm has no go_home method")
            except Exception as e:
                logger.warning(f"Could not go home: {e}")
            
            # Join any threads
            try:
                # Try to get the thread from the controller's connection
                t = getattr(getattr(self.robot_arm, 'conn', None), 't', None)
                if t is not None and hasattr(t, 'is_alive') and t.is_alive():
                    t.join(timeout=2)
            except Exception as e:
                logger.warning(f"Could not join thread: {e}")
        
        # Force exit the program
        os._exit(0)


def setup_signal_handler(robot_arm=None, custom_cleanup_func=None):
    """
    Convenience function to set up a graceful exit handler.
    
    Args:
        robot_arm: The robot arm instance
        custom_cleanup_func: Optional custom cleanup function
        
    Returns:
        The handler instance
    """
    handler = GracefulExitHandler(robot_arm, custom_cleanup_func)
    handler.register()
    return handler