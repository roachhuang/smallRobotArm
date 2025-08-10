"""Motion Pattern Generators

Collection of motion pattern generators for robot trajectories.
"""

import numpy as np
from numpy import ndarray


class MotionPatterns:
    """Collection of motion pattern generators for robot trajectories."""
    
    @staticmethod
    def fourier_circle(t: float, radius: float = 40, period: float = 15.0, harmonics: int = 1) -> ndarray:
        """Generate velocity vector for circular motion."""
        omega = 2 * np.pi / period
        speed = omega * radius
        x_dot = np.zeros(6)
        vx, vy = 0, 0

        for n in range(1, harmonics + 1):
            vx += -n * speed * np.sin(n * omega * t)
            vy += n * speed * np.cos(n * omega * t)

        x_dot[0] = vx
        x_dot[1] = vy
        return x_dot
    
    @staticmethod
    def figure8(t: float, radius: float = 30, freq: float = 1.0) -> ndarray:
        """Generate velocity vector for figure-eight pattern in XY plane."""
        v = np.zeros(6)
        v[0] = -radius * 2 * np.pi * freq * np.sin(2 * np.pi * freq * t)
        v[1] = radius * 2 * np.pi * freq * np.cos(4 * np.pi * freq * t)
        return v
    
    @staticmethod
    def spiral(t: float, radius_rate: float = 2.0, angular_speed: float = 0.5) -> ndarray:
        """Generate velocity vector for spiral motion in XY plane."""
        r = radius_rate * t
        v = np.zeros(6)
        v[0] = -r * angular_speed * np.sin(angular_speed * t) + radius_rate * np.cos(angular_speed * t)
        v[1] = r * angular_speed * np.cos(angular_speed * t) + radius_rate * np.sin(angular_speed * t)
        return v
    
    @staticmethod
    def zigzag(t: float, side_length: float = 40, period: float = 10.0) -> ndarray:
        """Generate velocity vector for zigzag motion in XY plane."""
        v = np.zeros(6)
        step = int((t / (period / 4)) % 2)
        v[0] = side_length / (period / 4) if step == 0 else -side_length / (period / 4)
        v[1] = 10 * np.sin(4 * np.pi * t / period)
        return v
    
    @staticmethod
    def square_xz(t: float, side_length: float, period: float) -> ndarray:
        """Generate velocity vector for square motion in XZ plane."""
        v = np.zeros(6)
        edge_time = period / 4
        speed = side_length / edge_time
        phase = int(t / edge_time) % 4
        
        if phase == 0: v[0] = speed
        elif phase == 1: v[2] = -speed
        elif phase == 2: v[0] = -speed
        elif phase == 3: v[2] = speed
        return v
    
    @staticmethod
    def wave(t: float) -> ndarray:
        """Generate joint space wave motion."""
        return np.array([
            0.2 * np.sin(t), 0.1 * np.cos(t),
            0, 0, 0, 0
        ])
        
    @staticmethod
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

    @staticmethod
    def general_fourier_velocity(self, t, A, B, T):
        omega = 2 * np.pi / T
        result = 0
        for n in range(1, len(A) + 1):
            result += A[n-1] * np.cos(n * omega * t) + B[n-1] * np.sin(n * omega * t)
        return result
    
# Export functions for direct use
fourier_circle = MotionPatterns.fourier_circle
figure8 = MotionPatterns.figure8
spiral = MotionPatterns.spiral
zigzag = MotionPatterns.zigzag
square_xz = MotionPatterns.square_xz
wave = MotionPatterns.wave