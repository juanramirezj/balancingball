#chatpgt prompt:
#create a python program that calculate the position of three servos for a 3rrs parallel manipulator
import numpy as np

def rotation_z(angle):
    """Rotation matrix around Z-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

def inverse_kinematics_3rrs(platform_pos, l1, l2):
    """
    Computes servo angles for a 3RRS manipulator.
    
    platform_pos: [x, y, z] position of the platform center
    l1: length of first link (from base to elbow)
    l2: length of second link (elbow to platform)
    Returns angles for each of the 3 servos in radians.
    """

    # Base joint positions (equilateral triangle)
    base_radius = 92  # mm
    B = np.array([
        [base_radius, 0, 0],
        [base_radius * np.cos(2*np.pi/3), base_radius * np.sin(2*np.pi/3), 0],
        [base_radius * np.cos(4*np.pi/3), base_radius * np.sin(4*np.pi/3), 0],
    ])

    # Platform joint offsets (same triangle, smaller radius)
    platform_radius = 65  # mm
    P_offsets = np.array([
        [platform_radius, 0, 0],
        [platform_radius * np.cos(2*np.pi/3), platform_radius * np.sin(2*np.pi/3), 0],
        [platform_radius * np.cos(4*np.pi/3), platform_radius * np.sin(4*np.pi/3), 0],
    ])

    # Target joint positions on platform in world space
    P_world = P_offsets + platform_pos

    angles = []
    for i in range(3):
        vec = P_world[i] - B[i]
        dist = np.linalg.norm(vec)
        
        # Check if the position is reachable
        if dist > l1 + l2 or dist < abs(l1 - l2):
            raise ValueError(f"Leg {i+1}: Position unreachable.")

        # Law of cosines to find angle at base servo
        # Assume motion in plane formed by vector and base
        # Project to 2D for simplicity
        dx, dz = np.linalg.norm(vec[:2]), vec[2]
        d = np.sqrt(dx**2 + dz**2)
        
        angle = np.arccos((l1**2 + d**2 - l2**2) / (2 * l1 * d))  # angle between l1 and d
        base_angle = np.arctan2(dz, dx)  # angle of vector in plane
        
        total_angle = base_angle - angle
        angles.append(total_angle)

    return angles


# Platform center at (0, 0, 150mm)
platform_position = np.array([0, 0, 0])  # mm
while True:
    height = input("Enter the height of the platform (in mm): ")
   
    platform_position[2] = height # mm
    # Link lengths
    l1 = 98  # mm
    l2 = 120  # mm

    angles = inverse_kinematics_3rrs(platform_position, l1, l2)
    for i, angle in enumerate(angles, start=1):
        print(f"Servo {i} angle: {np.degrees(angle):.2f}°")

