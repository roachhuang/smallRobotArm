
from spatialmath import SE3
import numpy as np

class Interp:
    def __init__(self):
        pass
    
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