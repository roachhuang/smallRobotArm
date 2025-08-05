"""RRT Planning Example

This example demonstrates how to use RRT path planning with obstacle avoidance.
"""

import numpy as np
from robot_tools.kinematics import SmallRbtArm, std_dh_params
from robot_tools.trajectory import Interp
from robot_tools.trajectory.rrt_planner import EXAMPLE_OBSTACLES

def main():
    # Create robot model
    robot = SmallRbtArm(std_dh_params)
    
    # Create interpolator with robot model
    interp = Interp(robot)
    
    # Define start and goal configurations
    start_angles = [0, -45, 30, 0, -90, 0]  # degrees
    goal_angles = [45, -30, 60, 0, -45, 0]  # degrees
    
    print("=== RRT Path Planning Example ===")
    print(f"Start: {start_angles}")
    print(f"Goal: {goal_angles}")
    
    # Plan without obstacles (should use simple linear path)
    print("\n1. Planning without obstacles:")
    path_simple = interp.plan_simple_path(start_angles, goal_angles)
    print(f"Generated {len(path_simple)} waypoints")
    
    # Plan with obstacles (will use RRT if OMPL available)
    print("\n2. Planning with obstacles:")
    obstacles = EXAMPLE_OBSTACLES
    print(f"Obstacles: {len(obstacles)} spherical obstacles")
    
    path_rrt = interp.plan_rrt_path(start_angles, goal_angles, obstacles)
    
    if path_rrt:
        print(f"RRT generated {len(path_rrt)} waypoints")
        print("First few waypoints:")
        for i, waypoint in enumerate(path_rrt[:3]):
            print(f"  {i}: {[round(x, 1) for x in waypoint]}")
    else:
        print("RRT planning failed")
    
    # Example of using the path with a controller
    print("\n3. Executing planned path:")
    print("# Example usage with controller:")
    print("# controller = PositionController(robot)")
    print("# controller.follow_joint_trajectory(path_rrt)")

if __name__ == "__main__":
    main()