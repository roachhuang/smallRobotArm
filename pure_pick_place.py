#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import sleep
import sys

from roboticstoolbox import DHRobot

from robot_tools.controller.position_controller import PositionController
from robot_tools.kinematics import SmallRbtArm
from robot_tools.kinematics import std_dh_tbl, std_dh_params
from robot_tools.misc.signal_handler import setup_signal_handler

# ====== CONFIG ======
URDF_PATH = "my_robot.urdf"
GRIPPER_OFFSET = [50.0, 0, 0]  # mm
# GRIPPER_RPY = [180, 0, 0]        # top approach 
# APPROACH_AXIS = [0, 0, -1]         # Industrial standard: approach along tool -Z

GRIPPER_RPY = [0, 90, 0]          # deg
APPROACH_AXIS = [1, 0, 0]         # Industrial standard: approach along tool -Z

APPROACH_DIST = 50.0               # mm
RETREAT_DIST = 50.0               # mm
RETREAT_AXIS = [0, 0, 1]           # Retreat along tool +Z
CLEARANCE_HEIGHT = 100.0           # mm above work surface

# ====== UTILS ======
def make_pose(position, rpy_deg):
    """Create 4x4 homogeneous transform from pos (mm) + RPY (degrees)."""
    rot = R.from_euler('xyz', rpy_deg, degrees=True).as_matrix()
    pose = np.eye(4)
    pose[:3, :3] = rot
    pose[:3, 3] = position
    return pose

def grasp_pose(object_pose, offset_pose):
    """Object pose × offset → gripper pose in world (X_world_gripper)."""
    return object_pose @ offset_pose

def offset_along_gripper_z(pose, dist):
    """Move along pose's +Z axis (pose is a 4x4 transform)."""
    new_pose = np.copy(pose)
    new_pose[:3, 3] += new_pose[:3, 2] * dist
    return new_pose

def create_approach_pose(grasp_pose, distance):
    """
    Industrial standard: approach along tool Z-axis (toward grasp pose).
    
    Args:
        grasp_pose: 4x4 homogeneous transform of grasp pose (mm)
        distance: positive distance to move toward along tool Z-axis (mm)
    """
    approach_pose = grasp_pose.copy()
    tool_z_axis = grasp_pose[:3, 2]  # Gripper Z-axis
    approach_pose[:3, 3] = grasp_pose[:3, 3] - tool_z_axis * distance  # Move toward object
    print("DEBUG: tool Z-axis:", np.round(tool_z_axis, 3))
    if abs(tool_z_axis[2]) < 0.9:  # Warn if not side approach
        print("WARNING: Tool Z-axis may not be perpendicular to object Z. Check GRIPPER_RPY.")
    return approach_pose

# ====== MAIN PLANNER ======
def plan_pick_and_place(object_pose, place_pose, robot):
    offset_pose = make_pose(GRIPPER_OFFSET, GRIPPER_RPY)

    # --- PICK ---
    grasp = grasp_pose(object_pose, offset_pose)
    # check_workspace_bounds(grasp)
    approach = create_approach_pose(grasp, APPROACH_DIST)  # Approach toward grasp
    # check_workspace_bounds(approach)
    retreat = offset_along_gripper_z(grasp, RETREAT_DIST)
    # check_workspace_bounds(retreat)

    # --- PLACE ---
    place = grasp_pose(place_pose, offset_pose)
    # check_workspace_bounds(place)
    pre_place = create_approach_pose(place, APPROACH_DIST)
    # check_workspace_bounds(pre_place)
    post_place = offset_along_gripper_z(place, RETREAT_DIST)
    # check_workspace_bounds(post_place)

    print("DEBUG: approach pos (world, mm):", np.round(approach[:3, 3], 3))
    print("DEBUG: grasp pos (world, mm):", np.round(grasp[:3, 3], 3))
    print("DEBUG: tool Z-axis:", np.round(approach[:3, 2], 3))

    waypoints = [approach, grasp, retreat, pre_place, place, post_place]
    joint_paths = []
    for pose in waypoints:
        joints = robot.ik(pose)
        joint_paths.append(joints)
        
    return joint_paths

    # smooth_joints = []
    # for i in range(len(joint_paths)-1):
    #     smooth_joints.extend(interpolate_joints(joint_paths[i], joint_paths[i+1])[:-1])
    # smooth_joints.append(joint_paths[-1])

    # return smooth_joints

# ====== DEMO EXECUTION ======
if __name__ == "__main__":
    custom_robot = SmallRbtArm(std_dh_params)
    controller = PositionController(custom_robot)
    setup_signal_handler(controller)
    sleep(1)
    controller.enable()
    sleep(1)

    # Example poses (mm and degrees)
    X_object = custom_robot.pose2T((210, 0.0, 80, 0, 0, 0))
    X_place  = custom_robot.pose2T((210, 100, 80, 0, 0, 0))

    # Plan industrial-style sequence
    joint_seq = plan_pick_and_place(X_object, X_place, robot=custom_robot)
    
    if joint_seq is None:
        sys.exit("Path planning failed")

    # Execute with industrial timing
    waypoint_names = [
        "Above Object", "Approach", "Grasp", "Retreat", 
        "Lift", "Above Place", "Pre-Place", "Place",
        "Post-Place", "Final Lift"
    ]
    
    for i, (name, q) in enumerate(zip(waypoint_names, joint_seq)):
        print(f"{i+1}/{len(joint_seq)}: {name} - {np.round(q, 3)}")
        controller.move_to_angles(q)
        
        # Industrial pause at critical points
        if name in ("Grasp", "Place"):
            print("  >> Executing end effector operation <<")
            # Add your gripper control here
            sleep(0.5)  # Simulate operation time

    print("Operation complete")
    sleep(3)
    controller.go_home()