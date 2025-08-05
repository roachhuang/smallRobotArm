
from spatialmath import SE3
import numpy as np
from .rrt_planner import plan_collision_free_path

class Interp:
    def __init__(self, robot=None):
        self.robot = robot
    
    def plan_rrt_path(self, start_angles, goal_angles, obstacles=None):
        """Plan collision-free path using RRT*."""
        if self.robot is None:
            print("Warning: No robot model provided. Using simple linear path.")
            return self.generate_linear_path_in_js(start_angles, goal_angles, steps=20)
            
        path = plan_collision_free_path(self.robot, start_angles, goal_angles, obstacles)
        
        if path is None:
            print("RRT planning failed. Using linear interpolation.")
            return self.generate_linear_path_in_js(start_angles, goal_angles, steps=20)
            
        return path
    
    def plan_simple_path(self, start_angles, goal_angles, obstacles=None):
        """Simple straight-line path planning in joint space."""
        if obstacles is None:
            return self.generate_linear_path_in_js(start_angles, goal_angles, steps=20)
        else:
            return self.plan_rrt_path(start_angles, goal_angles, obstacles)
    
    def interpolate_poses(self, start_pose: SE3, end_pose: SE3, num_poses=5):
        s_values = np.linspace(0, 1, num_poses + 2)[1:-1]
        poses = [start_pose.interp(end_pose, s) for s in s_values]
        return poses
    
    # in joint space
    def generate_linear_path_in_js(self, start, end, steps):
        """Generates joint-space straight-line path from start to end."""
        start = np.array(start)
        end = np.array(end)
        path = [start + (end - start) * t / (steps - 1) for t in range(steps)]
        return path
    
    # in cartesian space
    def generate_linear_pose_path(self, start_pose: SE3, end_pose: SE3, steps: int):
        path = [start_pose.interp(end_pose, s / (steps - 1)) for s in range(steps)]
        return path

    # cartesian space
    def smooth_pose_path(self, path:list[SE3], alpha=0.3):
        '''
        For each step, it interpolates between the previous smoothed pose and the current input pose using the given alpha.
        This creates a low-pass filter effect in SE3 space.
        '''
        smoothed = [path[0]]
        for i in range(1, len(path)):
            smooth_pose = smoothed[-1].interp(path[i], alpha)
            smoothed.append(smooth_pose)
        return smoothed
    
    def fourier_xy_velocity(self, t):
        # Example Fourier synthesis for x(t) and y(t)
        T = 15.0  # Period
        A_list = [
            [0, 0, 40],  # A_n for x — 3rd harmonic cosine only
            [0, 20, 0],  # A_n for y — 2nd harmonic cosine only
        ]
        B_list = [
            [0, 0, 0],   # B_n for x
            [0, 0, 0],   # B_n for y
        ]

        x_dot = self.general_fourier_velocity(t, A_list[0], B_list[0], T)
        y_dot = self.general_fourier_velocity(t, A_list[1], B_list[1], T)

        return np.array([x_dot, y_dot, 0, 0, 0, 0])  # Cartesian velocity vector (6D)

    
    def general_fourier_velocity(self, t, A, B, T):
        omega = 2 * np.pi / T
        result = 0
        for n in range(1, len(A) + 1):
            result += A[n-1] * np.cos(n * omega * t) + B[n-1] * np.sin(n * omega * t)
        return result
