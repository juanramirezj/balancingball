# This script calculates the inverse kinematics for a 3RRS manipulator
# copilot suggested this code
import numpy as np

def compute_platform_center(normal_vector, height):
    """Compute the center position of the platform based on normal vector and height."""
    nx, ny, nz = normal_vector
    center_x = nx * height
    center_y = ny * height
    center_z = nz * height
    return center_x, center_y, center_z

def inverse_kinematics(l1, l2, servo_distance, platform_radius, normal_vector, height):
    """Calculate servo angles for the 3RRS manipulator based on platform orientation."""
    
    center_x, center_y, center_z = compute_platform_center(normal_vector, height)
    
    # Compute the equilateral triangle servo positions in the XY plane
    angle_offset = np.radians([0, 120, 240])  # Equilateral triangle angles
    base_positions = np.array([
        [servo_distance * np.cos(a), servo_distance * np.sin(a), 0]  # Servos at z=0
        for a in angle_offset
    ])

    angles = []
    for base_x, base_y, base_z in base_positions:
        dx, dy, dz = center_x - base_x, center_y - base_y, center_z - base_z
        d = np.sqrt(dx**2 + dy**2 + dz**2)
        
        if d > (l1 + l2):
            raise ValueError("Target position is out of the reachable workspace.")

        # Law of cosines to find joint angles
        cos_theta2 = (dx**2 + dy**2 + dz**2 - l1**2 - l2**2) / (2 * l1 * l2)
        theta2 = np.arccos(np.clip(cos_theta2, -1, 1))  # Clip to avoid domain errors
        theta1 = np.arctan2(dy, dx) - np.arctan2(l2 * np.sin(theta2), l1 + l2 * np.cos(theta2))

        angles.append((np.degrees(theta1), np.degrees(theta2)))

    return angles

# Example values (in mm)
l1 = 98  # Length of first link
l2 = 120  # Length of second link
servo_distance = 92  # Distance of servo from center
platform_radius = 65  # Radius of platform
normal_vector = (0, 0, 1)  # Example normal vector at the platform center
height = 190  # Height of the center of the platform

servo_angles = inverse_kinematics(l1, l2, servo_distance, platform_radius, normal_vector, height)
print("Servo angles:", servo_angles)
