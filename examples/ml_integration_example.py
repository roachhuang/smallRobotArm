"""ML Integration Example

This example shows how to integrate ML capabilities into your robot arm project.
"""

import numpy as np
import cv2
from robot_tools.kinematics import SmallRbtArm, std_dh_params
from robot_tools.controller import RobotController
from robot_tools.ml import MLVision, detect_and_plan_grasp

def main():
    # Initialize robot
    robot = SmallRbtArm(std_dh_params)
    controller = RobotController(robot)
    
    print("=== ML Integration Demo ===")
    
    # 1. Vision-based object detection
    print("\n1. Object Detection Demo")
    
    # Simulate camera image (replace with actual camera feed)
    dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Create ML vision system
    vision = MLVision()
    
    # Detect objects
    objects = vision.detect_objects(dummy_image)
    print(f"Detected {len(objects)} objects")
    
    for i, obj in enumerate(objects):
        print(f"  Object {i}: {obj['class']} (confidence: {obj['confidence']:.2f})")
    
    # 2. Grasp planning
    print("\n2. Grasp Planning Demo")
    
    if objects:
        grasp_candidates = vision.predict_grasp_poses(objects)
        print(f"Generated {len(grasp_candidates)} grasp candidates")
        
        for i, grasp in enumerate(grasp_candidates[:3]):  # Show top 3
            print(f"  Grasp {i}: pos={grasp['position']}, confidence={grasp['confidence']:.2f}")
    
    # 3. ML-enhanced pick and place
    print("\n3. ML-Enhanced Pick and Place")
    
    def ml_pick_and_place(image):
        """ML-powered pick and place pipeline."""
        # Detect objects and plan grasps
        grasp_candidates = detect_and_plan_grasp(image)
        
        if not grasp_candidates:
            print("No objects detected for grasping")
            return
        
        # Select best grasp
        best_grasp = grasp_candidates[0]
        print(f"Selected grasp for {best_grasp['object_class']}")
        
        # Convert to robot coordinates and execute
        # (This would need proper camera-to-robot calibration)
        target_pose = [
            best_grasp['position'][0],
            best_grasp['position'][1], 
            best_grasp['position'][2],
            0, 90, 0  # Simple orientation
        ]
        
        print(f"Moving to grasp pose: {target_pose}")
        # controller.move_to_pose(target_pose)  # Uncomment for real execution
        # controller.grab()
        
        return True
    
    # Demo the ML pipeline
    success = ml_pick_and_place(dummy_image)
    if success:
        print("ML pick and place completed successfully!")
    
    # 4. Trajectory optimization demo
    print("\n4. Trajectory Optimization Demo")
    
    try:
        from robot_tools.ml.trajectory_rl import TrajectoryOptimizer
        
        optimizer = TrajectoryOptimizer(controller)
        
        # Generate optimized trajectory
        start_pose = [0, -45, 30, 0, -90, 0]
        target_pose = [45, -30, 60, 0, -45, 0]
        
        trajectory = optimizer.optimize_trajectory(start_pose, target_pose)
        print(f"Generated optimized trajectory with {len(trajectory)} waypoints")
        
        # Show first few waypoints
        for i, waypoint in enumerate(trajectory[:3]):
            print(f"  Waypoint {i}: {[round(x, 1) for x in waypoint]}")
            
    except ImportError:
        print("RL optimization not available (install stable-baselines3)")
    
    print("\n=== ML Integration Demo Complete ===")
    print("\nTo enable full ML capabilities, install:")
    print("  pip install ultralytics stable-baselines3 torch torchvision")

if __name__ == "__main__":
    main()