"""Reinforcement Learning for Trajectory Optimization

This module provides RL-based trajectory optimization for the robot arm.
"""

import numpy as np
import gym
from gym import spaces
from typing import Dict, Any

try:
    import stable_baselines3 as sb3
    SB3_AVAILABLE = True
except ImportError:
    SB3_AVAILABLE = False


class RobotArmEnv(gym.Env):
    """Gym environment for robot arm trajectory optimization."""
    
    def __init__(self, robot_controller):
        super().__init__()
        
        self.controller = robot_controller
        self.target_pose = None
        self.max_steps = 100
        self.current_step = 0
        
        # Action space: joint velocities
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(6,), dtype=np.float32
        )
        
        # Observation space: current joints + target pose + manipulability
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(13,), dtype=np.float32
        )
    
    def reset(self):
        """Reset environment to random start position."""
        # Random start position within joint limits
        start_angles = np.random.uniform(-90, 90, 6)
        self.controller.move_to_angles(start_angles)
        
        # Random target pose
        self.target_pose = np.random.uniform(
            [-200, -200, 100, -180, -90, -180],
            [200, 200, 300, 180, 90, 180]
        )
        
        self.current_step = 0
        return self._get_observation()
    
    def step(self, action):
        """Execute action and return next state."""
        # Scale action to reasonable joint velocities
        joint_velocities = action * np.radians(10)  # Max 10 deg/s
        
        # Apply action
        self.controller.joint_space_vel_ctrl(
            lambda t: joint_velocities, 
            duration=0.1
        )
        
        # Get new observation
        obs = self._get_observation()
        
        # Calculate reward
        reward = self._calculate_reward()
        
        # Check if done
        self.current_step += 1
        done = (self.current_step >= self.max_steps or 
                self._is_at_target())
        
        return obs, reward, done, {}
    
    def _get_observation(self):
        """Get current observation."""
        current_angles = np.array(self.controller.current_angles)
        target_pose = np.array(self.target_pose)
        
        # Add manipulability measure
        q_rad = np.radians(current_angles)
        try:
            manipulability = self.controller.eigen_analysis(q_rad)['manipulability']
        except:
            manipulability = 0.1
        
        return np.concatenate([
            current_angles / 180.0,  # Normalize joint angles
            target_pose / 200.0,     # Normalize target pose
            [manipulability]
        ]).astype(np.float32)
    
    def _calculate_reward(self):
        """Calculate reward for current state."""
        # Distance to target reward
        current_pose = self._get_current_pose()
        distance = np.linalg.norm(np.array(current_pose[:3]) - np.array(self.target_pose[:3]))
        distance_reward = -distance / 1000.0  # Scale to reasonable range
        
        # Manipulability reward (avoid singularities)
        q_rad = np.radians(self.controller.current_angles)
        try:
            manipulability = self.controller.eigen_analysis(q_rad)['manipulability']
            manip_reward = manipulability * 10
        except:
            manip_reward = -10  # Penalty for singularity
        
        # Smoothness reward (penalize large joint movements)
        smoothness_reward = -np.sum(np.abs(np.array(self.controller.current_angles))) / 1000.0
        
        return distance_reward + manip_reward + smoothness_reward
    
    def _get_current_pose(self):
        """Get current end-effector pose."""
        q_rad = np.radians(self.controller.current_angles)
        T = self.controller.robot.fk(np.degrees(q_rad))
        return self.controller.robot.T2Pose(T)
    
    def _is_at_target(self, tolerance=10.0):
        """Check if robot is at target pose."""
        current_pose = self._get_current_pose()
        distance = np.linalg.norm(np.array(current_pose[:3]) - np.array(self.target_pose[:3]))
        return distance < tolerance


class TrajectoryOptimizer:
    """RL-based trajectory optimizer."""
    
    def __init__(self, robot_controller):
        self.controller = robot_controller
        self.env = RobotArmEnv(robot_controller)
        self.model = None
        
        if SB3_AVAILABLE:
            self.model = sb3.PPO('MlpPolicy', self.env, verbose=1)
    
    def train(self, total_timesteps=10000):
        """Train the RL model."""
        if not SB3_AVAILABLE:
            print("stable-baselines3 not available. Install with: pip install stable-baselines3")
            return
        
        print(f"Training RL model for {total_timesteps} timesteps...")
        self.model.learn(total_timesteps=total_timesteps)
        print("Training completed!")
    
    def optimize_trajectory(self, start_pose, target_pose):
        """Generate optimized trajectory using trained model."""
        if not SB3_AVAILABLE or self.model is None:
            print("Model not available or not trained. Using fallback.")
            return self._fallback_trajectory(start_pose, target_pose)
        
        # Set environment target
        self.env.target_pose = target_pose
        
        # Generate trajectory
        obs = self.env.reset()
        trajectory = []
        
        for _ in range(100):  # Max steps
            action, _ = self.model.predict(obs, deterministic=True)
            obs, reward, done, _ = self.env.step(action)
            
            # Record current joint angles
            trajectory.append(self.controller.current_angles.copy())
            
            if done:
                break
        
        return trajectory
    
    def _fallback_trajectory(self, start_pose, target_pose):
        """Simple linear interpolation fallback."""
        steps = 20
        trajectory = []
        
        for i in range(steps + 1):
            alpha = i / steps
            interpolated = [
                start + alpha * (target - start)
                for start, target in zip(start_pose, target_pose)
            ]
            trajectory.append(interpolated)
        
        return trajectory
    
    def save_model(self, path):
        """Save trained model."""
        if self.model:
            self.model.save(path)
    
    def load_model(self, path):
        """Load trained model."""
        if SB3_AVAILABLE:
            self.model = sb3.PPO.load(path, env=self.env)