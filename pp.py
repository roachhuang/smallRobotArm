#!/usr/bin/env python3
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
from time import sleep
import sys

from robot_tools.controller.position_controller import PositionController
from robot_tools.kinematics import SmallRbtArm
from robot_tools.kinematics import std_dh_tbl, std_dh_params
from robot_tools.misc.signal_handler import setup_signal_handler

# ====== CONFIG ======
URDF_PATH = "smallrobot_updated.urdf"  # Reference only, not used directly
GRIPPER_OFFSET = [0.0, 0.0, 50.0]     # mm (matches std_dh_params)
GRIPPER_RPY = [90,0, 0]               # deg; set to choose approach direction (e.g., [180, 0, 0] for downward Z)
APPROACH_DIST = 50.0                  # mm
RETREAT_DIST = 50.0                   # mm
WORKSPACE_BOUNDS = [[-500, 500], [-500, 500], [0, 1000]]  # mm

# Industrial standard: approach along tool Z-axis, direction set by GRIPPER_RPY

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
    Industrial standard: approach along tool Z-axis (away from grasp pose).

    Args:
        grasp_pose: 4x4 homogeneous transform of grasp pose (mm)
        distance: positive distance to move away along tool Z-axis (mm)
    """
    approach_pose = grasp_pose.copy()
    tool_z_axis = grasp_pose[:3, 2]  # Z column of rotation matrix
    approach_pose[:3, 3] = grasp_pose[:3, 3] + tool_z_axis * distance
    print("DEBUG: tool Z-axis:", np.round(tool_z_axis, 3))
    return approach_pose

def interpolate_joints(q1, q2, steps=10):
    """Interpolate joint angles for smooth motion."""
    t = np.linspace(0, 1, steps)
    interp = interp1d([0, 1], np.vstack([q1, q2]), axis=0)
    return interp(t)

def check_workspace_bounds(pose):
    """Check if pose is within workspace bounds (mm)."""
    pos = pose[:3, 3]
    if not all(WORKSPACE_BOUNDS[i][0] <= pos[i] <= WORKSPACE_BOUNDS[i][1] for i in range(3)):
        raise ValueError(f"Pose {pos} outside workspace bounds {WORKSPACE_BOUNDS}")

# ====== MAIN PLANNER ======
def plan_pick_and_place(object_pose, place_pose, robot):
    """Generate IK joint sequences for pick-and-place with industrial standard approach."""
    # Poses in mm (from pose2T), no scaling needed
    offset_pose = make_pose(GRIPPER_OFFSET, GRIPPER_RPY)

    # --- PICK ---
    grasp = grasp_pose(object_pose, offset_pose)
    check_workspace_bounds(grasp)
    approach = create_approach_pose(grasp, APPROACH_DIST)
    check_workspace_bounds(approach)
    retreat = offset_along_gripper_z(grasp, RETREAT_DIST)
    check_workspace_bounds(retreat)

    # --- PLACE ---
    place = grasp_pose(place_pose, offset_pose)
    check_workspace_bounds(place)
    pre_place = create_approach_pose(place, APPROACH_DIST)
    check_workspace_bounds(pre_place)
    post_place = offset_along_gripper_z(place, RETREAT_DIST)
    check_workspace_bounds(post_place)

    print("DEBUG: approach pos (world, mm):", np.round(approach[:3, 3], 3))
    print("DEBUG: grasp    pos (world, mm):", np.round(grasp[:3, 3], 3))
    print("DEBUG: pre_place pos (world, mm):", np.round(pre_place[:3, 3], 3))
    print("DEBUG: place    pos (world, mm):", np.round(place[:3, 3], 3))

    waypoints = [approach, grasp, retreat, pre_place, place, post_place]
    joint_paths = []
    for pose in waypoints:
        joints = robot.ik(pose)  # Assumes ik handles mm
        joint_paths.append(joints)
    return joint_paths
    
    # smooth_joints = []
    # for i in range(len(joint_paths)-1):
    #     smooth_joints.extend(interpolate_joints(joint_paths[i], joint_paths[i+1])[:-1])
    # smooth_joints.append(joint_paths[-1])

    # return smooth_joints

# ====== DEMO ======
if __name__ == "__main__":
    custom_robot = SmallRbtArm(std_dh_params)  # DH params in mm
    controller = PositionController(custom_robot)
    setup_signal_handler(controller)
    sleep(1)
    controller.enable()
    sleep(1)

    controller.move_to_angles([0]*6)
    sleep(3)
    
    X_object = custom_robot.pose2T((250, 0.0, 52, 0, 0, 0))  # mm, RPY deg
    X_place  = custom_robot.pose2T((180, 180, 52, 0, 0, 90))  # mm, RPY deg

    joint_seq = plan_pick_and_place(X_object, X_place, robot=custom_robot)

    for i, q in enumerate(joint_seq):
        print(f"Waypoint {i+1}: {np.round(q, 3)}")
        controller.move_to_angles(q)

    sleep(3)
    controller.go_home()