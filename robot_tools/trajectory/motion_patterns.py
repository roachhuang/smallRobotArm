"""Motion Pattern Generators

This module provides velocity functions for various motion patterns
used in robotic trajectory generation.
"""

import numpy as np

def square_xz(t, side_length, period)->np.ndarray:
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

def figure8(t, radius=30, freq=1.0):
    """Generate velocity vector for figure-eight pattern in XY plane."""
    v = np.zeros(6)
    v[0] = -radius * 2 * np.pi * freq * np.sin(2 * np.pi * freq * t)
    v[1] = radius * 2 * np.pi * freq * np.cos(4 * np.pi * freq * t)
    return v

def fourier_circle(t, radius=40, period=15.0, harmonics=1):
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

def spiral(t, radius_rate=2.0, angular_speed=0.5):
    """Generate velocity vector for spiral motion in XY plane."""
    r = radius_rate * t
    v = np.zeros(6)
    v[0] = -r * angular_speed * np.sin(angular_speed * t) + radius_rate * np.cos(angular_speed * t)
    v[1] = r * angular_speed * np.cos(angular_speed * t) + radius_rate * np.sin(angular_speed * t)
    return v

def zigzag(t, side_length=40, period=10.0):
    """Generate velocity vector for zigzag motion in XY plane."""
    v = np.zeros(6)
    step = int((t / (period / 4)) % 2)
    v[0] = side_length / (period / 4) if step == 0 else -side_length / (period / 4)
    v[1] = 10 * np.sin(4 * np.pi * t / period)
    return v

############## for joint space
def wave(t):
    return np.array([
        0.2 * np.sin(t), 0.1 * np.cos(t),
        0, 0, 0, 0
    ])
