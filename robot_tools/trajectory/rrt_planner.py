"""RRT Path Planner Module

This module provides RRT-based path planning for the small robot arm using OMPL.
Install OMPL with: pip install ompl

Classes:
    RRTPlanner: RRT path planner for collision-free motion
"""

import numpy as np
from typing import List, Tuple, Optional

try:
    from ompl import base as ob
    from ompl import geometric as og
    OMPL_AVAILABLE = True
except ImportError:
    OMPL_AVAILABLE = False
    print("OMPL not available. Install with: pip install ompl")


class RRTPlanner:
    """RRT path planner for the robot arm."""
    
    def __init__(self, robot, joint_limits=None):
        """Initialize RRT planner.
        
        Args:
            robot: Robot model for collision checking
            joint_limits: Joint limits as [(min, max), ...] for each joint
        """
        self.robot = robot
        
        if joint_limits is None:
            # Default joint limits (radians)
            self.joint_limits = [(-np.pi, np.pi)] * 6
        else:
            self.joint_limits = joint_limits
            
        if not OMPL_AVAILABLE:
            raise ImportError("OMPL is required for RRT planning. Install with: pip install ompl")
    
    def is_state_valid(self, state, obstacles=None):
        """Check if a joint configuration is collision-free.
        
        Args:
            state: Joint configuration
            obstacles: List of obstacle definitions
            
        Returns:
            bool: True if state is valid
        """
        # Extract joint angles
        q = [state[i] for i in range(6)]
        
        # Check joint limits
        for i, (q_min, q_max) in enumerate(self.joint_limits):
            if not (q_min <= q[i] <= q_max):
                return False
        
        # Check self-collision (simplified)
        if self._check_self_collision(q):
            return False
            
        # Check obstacle collision
        if obstacles and self._check_obstacle_collision(q, obstacles):
            return False
            
        return True
    
    def _check_self_collision(self, q):
        """Simple self-collision check."""
        # Implement basic self-collision detection
        # For now, just check if joints are too close to extreme positions
        return False  # Placeholder
    
    def _check_obstacle_collision(self, q, obstacles):
        """Check collision with obstacles."""
        # Get end-effector position
        T = self.robot.fk(np.degrees(q))
        ee_pos = T[:3, 3]
        
        # Check against spherical obstacles
        for obstacle in obstacles:
            if obstacle['type'] == 'sphere':
                center = np.array(obstacle['center'])
                radius = obstacle['radius']
                if np.linalg.norm(ee_pos - center) < radius:
                    return True
        
        return False
    
    def plan_rrt_star(self, start_q, goal_q, obstacles=None, planning_time=1.0):
        """Plan path using RRT*.
        
        Args:
            start_q: Start joint configuration (degrees)
            goal_q: Goal joint configuration (degrees)
            obstacles: List of obstacles
            planning_time: Planning time limit (seconds)
            
        Returns:
            List of joint configurations or None if no path found
        """
        if not OMPL_AVAILABLE:
            return self._fallback_planning(start_q, goal_q)
        
        # Convert to radians
        start_rad = np.radians(start_q)
        goal_rad = np.radians(goal_q)
        
        # Create state space
        space = ob.RealVectorStateSpace(6)
        bounds = ob.RealVectorBounds(6)
        
        for i, (q_min, q_max) in enumerate(self.joint_limits):
            bounds.setLow(i, q_min)
            bounds.setHigh(i, q_max)
        
        space.setBounds(bounds)
        
        # Create simple setup
        ss = og.SimpleSetup(space)
        
        # Set state validity checker
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(
            lambda state: self.is_state_valid(state, obstacles)
        ))
        
        # Create start and goal states
        start = ob.State(space)
        goal = ob.State(space)
        
        for i in range(6):
            start[i] = start_rad[i]
            goal[i] = goal_rad[i]
        
        ss.setStartAndGoalStates(start, goal)
        
        # Use RRT*
        planner = og.RRTstar(ss.getSpaceInformation())
        ss.setPlanner(planner)
        
        # Solve
        solved = ss.solve(planning_time)
        
        if solved:
            # Extract path
            path = ss.getSolutionPath()
            path.interpolate(50)  # Interpolate for smoother path
            
            # Convert to list of joint angles (degrees)
            waypoints = []
            for i in range(path.getStateCount()):
                state = path.getState(i)
                q_rad = [state[j] for j in range(6)]
                q_deg = np.degrees(q_rad)
                waypoints.append(q_deg.tolist())
            
            return waypoints
        
        return None
    
    def _fallback_planning(self, start_q, goal_q):
        """Fallback to simple linear interpolation if OMPL not available."""
        print("Using fallback linear interpolation (no collision checking)")
        
        start = np.array(start_q)
        goal = np.array(goal_q)
        
        # Simple linear interpolation
        n_steps = 20
        waypoints = []
        
        for i in range(n_steps + 1):
            alpha = i / n_steps
            q = start + alpha * (goal - start)
            waypoints.append(q.tolist())
        
        return waypoints


# Convenience function for easy integration
def plan_collision_free_path(robot, start_angles, goal_angles, obstacles=None):
    """Plan a collision-free path between two joint configurations.
    
    Args:
        robot: Robot model
        start_angles: Start joint angles (degrees)
        goal_angles: Goal joint angles (degrees)
        obstacles: List of obstacle definitions
        
    Returns:
        List of waypoints or None if no path found
    """
    planner = RRTPlanner(robot)
    return planner.plan_rrt_star(start_angles, goal_angles, obstacles)


# Example obstacle definitions
EXAMPLE_OBSTACLES = [
    {
        'type': 'sphere',
        'center': [200, 100, 150],  # mm
        'radius': 50  # mm
    },
    {
        'type': 'sphere', 
        'center': [0, 200, 100],
        'radius': 30
    }
]