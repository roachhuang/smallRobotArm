"""Trajectory Equations Module

This module provides the mathematical equations used for trajectory planning
with Linear Functions with Parabolic Blends (LFPB). Each equation represents
a different segment of the trajectory.

Functions:
    eq1: Initial acceleration segment (parabolic blend)
    eq2: First constant velocity segment (linear)
    eq3: First transition blend (parabolic)
    eq4: Second constant velocity segment (linear)
    eq5: Second transition blend (parabolic)
    eq6: Third constant velocity segment (linear)
    eq7: Final deceleration segment (parabolic blend)
"""

def eq1(t, v0, a0):
    """Initial acceleration segment equation (0 to 0.5s).
    
    Implements the parabolic blend for the initial acceleration from standstill.
    
    Args:
        t (float): Current time in seconds
        v0 (float): Initial velocity
        a0 (float): Initial acceleration
        
    Returns:
        float: Position increment from the initial position
    """
    dt = t - 0
    # Parabolic equation: d(t) = d0 + v0*t + 0.5*a*t^2
    return v0 * dt + 0.5 * a0 * dt**2


def eq2(t, v1):
    """First constant velocity segment equation (0.5 to 1.75s).
    
    Implements the linear segment after initial acceleration.
    
    Args:
        t (float): Current time in seconds
        v1 (float): Constant velocity
        
    Returns:
        float: Position increment from the reference position
    """
    dt = t - 0.25
    # Linear equation: d(t) = d0 + v*t
    return v1 * dt


def eq3(t, ts1, v1, acc1):
    """First transition blend equation (1.75 to 2.25s).
    
    Implements the parabolic blend at the first via point.
    
    Args:
        t (float): Current time in seconds
        ts1 (float): Time at first via point
        v1 (float): Velocity before the blend
        acc1 (float): Acceleration during the blend
        
    Returns:
        float: Position increment from the reference position
    """
    dt1 = t - 0.25
    dt2 = t - (ts1 - 0.25)
    return v1 * dt1 + 0.5 * acc1 * dt2**2


def eq4(dt, v2):
    """Second constant velocity segment equation (2.25 to 5.75s).
    
    Implements the linear segment after the first via point.
    
    Args:
        dt (float): Time delta from reference point
        v2 (float): Constant velocity
        
    Returns:
        float: Position increment from the reference position
    """
    return v2 * dt


def eq5(t, ts, v2, acc2):
    """Second transition blend equation (5.75 to 6.25s).
    
    Implements the parabolic blend at the second via point.
    
    Args:
        t (float): Current time in seconds
        ts (list): List of time points
        v2 (float): Velocity before the blend
        acc2 (float): Acceleration during the blend
        
    Returns:
        float: Position increment from the reference position
    """
    dt1 = t - ts[1]
    dt2 = t - (ts[2] - 0.25)
    return v2 * dt1 + 1 / 2 * acc2 * dt2**2


def eq6(dt, v3):
    """Third constant velocity segment equation (6.25 to 8.5s).
    
    Implements the linear segment after the second via point.
    
    Args:
        dt (float): Time delta from reference point
        v3 (float): Constant velocity
        
    Returns:
        float: Position increment from the reference position
    """
    return v3 * dt

def eq7(t, ts, v3, acc3, totalPoints):
    """Final deceleration segment equation (8.5 to 9.0s).
    
    Implements the parabolic blend for the final deceleration to stop.
    
    Args:
        t (float): Current time in seconds
        ts (list): List of time points
        v3 (float): Velocity before deceleration
        acc3 (float): Deceleration rate
        totalPoints (int): Total number of waypoints
        
    Returns:
        float: Position increment from the reference position
    """
    dt1 = t - ts[2]
    dt2 = t - (ts[totalPoints - 1] - 0.5)
    return v3 * dt1 + 1 / 2 * acc3 * dt2**2

'''
 # in 0, 0.5s. col[1~3]: x, y, z
    def eq1(t, col):
        dt = t - 0
        v0 = v[0, col]
        a0 = a[0, col]
        # col+1 coz time on the 1st col. parabloic (二次多項式)
        # Xeq1(t) = x0+v0*dt+1/2*at^2, here we replace d with theta
        return p[0, col + 1] + v0 * dt + 1 / 2 * a0 * dt**2

    # in 0.5, 1.75
    def eq2(t, col):
        # Xeq2(t)=x0+v1*dt.
        dt = t - 0.25
        v1 = v[1, col]
        # linear segment.
        return p[0, col + 1] + v1 * dt

    # in 1.75, 2.25
    def eq3(t, col):
        v1 = v[1, col]
        a1 = a[1, col]
        dt1 = t - 0.25
        dt2 = t - (ts[1] - 0.25)
        return p[0, col + 1] + v1 * dt1 + 1 / 2 * a1 * dt2**2

    # in 2.25, 5.75
    def eq4(t, col):
        dt = t - ts[1]
        v2 = v[2, col]
        return p[1, col + 1] + v2 * dt

    # 5.75, 6.25
    def eq5(t, col):
        dt1 = t - ts[1]
        dt2 = t - (ts[2] - 0.25)
        v2 = v[2, col]
        a2 = a[2, col]
        return p[1, col + 1] + v2 * dt1 + 1 / 2 * a2 * dt2**2

    # 6.25, 8.5
    def eq6(t, col):
        dt = t - ts[2]
        v3 = v[3, col]
        return p[2, col + 1] + v3 * dt

    # col - 0:x, 1:y, 2:theta
    # 8.5 ~9s
    def eq7(t, col):
        dt1 = t - ts[2]
        dt2 = t - (ts[totalPoints - 1] - 0.5)
        v3 = v[3, col]
        a3 = a[3, col]
        return p[2, col + 1] + v3 * dt1 + 1 / 2 * a3 * dt2**2
'''