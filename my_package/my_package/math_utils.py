import numpy as np
import math

def z_rotation_matrix(z):
    return np.array([
        [np.cos(z), -np.sin(z)],
        [np.sin(z), np.cos(z)]
    ])


def euler_to_quaternion(x, y, z):
    """
    Convert Euler angles (roll, pitch, yaw) to a quaternion (w, x, y, z).
    Euler angles must be in radians.
    
    Args:
    roll (float): Rotation around the x-axis (in radians)
    pitch (float): Rotation around the y-axis (in radians)
    yaw (float): Rotation around the z-axis (in radians)
    
    Returns:
    tuple: Quaternion (w, x, y, z)
    """
    
    # Compute half angles
    roll_half = x / 2.0
    pitch_half = y / 2.0
    yaw_half = z / 2.0

    # Compute trigonometric functions of the half angles
    cr = math.cos(roll_half)
    sr = math.sin(roll_half)
    cp = math.cos(pitch_half)
    sp = math.sin(pitch_half)
    cy = math.cos(yaw_half)
    sy = math.sin(yaw_half)

    # Compute quaternion
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)

