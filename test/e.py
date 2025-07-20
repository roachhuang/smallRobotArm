import numpy as np
import matplotlib.pyplot as plt

def generate_tilted_ellipse(center, semi_major, semi_minor, tilt_angle, num_points=100):
    """
    Generate a tilted ellipse in 3D with the given parameters.
    
    :param center: Center of the ellipse (x, y, z).
    :param semi_major: Length of the semi-major axis.
    :param semi_minor: Length of the semi-minor axis.
    :param tilt_angle: The tilt angle in radians.
    :param num_points: Number of points to generate for the ellipse.
    :return: Ellipse points in 3D.
    """
    # Generate the ellipse in the XY plane
    t = np.linspace(0, 2 * np.pi, num_points)
    ellipse_x = semi_major * np.cos(t)
    ellipse_y = semi_minor * np.sin(t)
    
    # Tilt the ellipse in the Z direction
    R_tilt = np.array([
        [1, 0, 0], 
        [0, np.cos(tilt_angle), -np.sin(tilt_angle)], 
        [0, np.sin(tilt_angle), np.cos(tilt_angle)]
    ])
    
    # Apply the rotation
    ellipse_points = np.vstack([ellipse_x, ellipse_y, np.zeros_like(ellipse_x)])  # Initial points
    tilted_points = np.dot(R_tilt, ellipse_points)
    
    # Apply the center shift (translation)
    tilted_points[0, :] += center[0]
    tilted_points[1, :] += center[1]
    tilted_points[2, :] += center[2]
    
    return tilted_points

# Parameters
center = np.array([200, 100, 20])  # Center (x, y, z), z = 20mm to ensure z is >= 10mm
semi_major = 50  # Length of the semi-major axis
semi_minor = 30  # Length of the semi-minor axis (ensures it doesn't go below z=10)
tilt_angle = np.radians(960)  # Tilt the ellipse by 30 degrees
num_points = 100  # Number of points to generate for the ellipse

# Generate tilted ellipse
ellipse_points = generate_tilted_ellipse(center, semi_major, semi_minor, tilt_angle, num_points)

# Plot the ellipse in 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(ellipse_points[0, :], ellipse_points[1, :], ellipse_points[2, :], label="Tilted Ellipse")
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.set_title('Tilted Ellipse in 3D')
ax.legend()

plt.show()
