# input h, roll and pitch

import numpy as np

def get_rotation_matrix(angle_x, angle_y):
    """
    Calculates the combined rotation matrix for tilting the platform.
    Angles should be in radians.
    """
    # Rotation around X-axis (Pitch/Tilt X)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(angle_x), -np.sin(angle_x)],
        [0, np.sin(angle_x), np.cos(angle_x)]
    ])
    
    # Rotation around Y-axis (Roll/Tilt Y)
    Ry = np.array([
        [np.cos(angle_y), 0, np.sin(angle_y)],
        [0, 1, 0],
        [-np.sin(angle_y), 0, np.cos(angle_y)]
    ])
    
    # Combined rotation (Rz is usually fixed at 0 for 3RRS platforms)
    return np.dot(Ry, Rx)

def inverse_kinematics_3rrs(height, tilt_x_deg, tilt_y_deg, l1, l2):
    """
    Computes servo angles for a 3RRS manipulator given height and tilt.
    """
    # Convert angles to radians
    rx = np.radians(tilt_x_deg)
    ry = np.radians(tilt_y_deg)
    
    # Platform center position
    platform_center = np.array([0, 0, height])

    # Base joint positions (Static)
    base_radius = 92
    B = np.array([
        [base_radius, 0, 0],
        [base_radius * np.cos(2*np.pi/3), base_radius * np.sin(2*np.pi/3), 0],
        [base_radius * np.cos(4*np.pi/3), base_radius * np.sin(4*np.pi/3), 0],
    ])

    # Platform joint offsets (Relative to center)
    p_radius = 65
    P_offsets = np.array([
        [p_radius, 0, 0],
        [p_radius * np.cos(2*np.pi/3), p_radius * np.sin(2*np.pi/3), 0],
        [p_radius * np.cos(4*np.pi/3), p_radius * np.sin(4*np.pi/3), 0],
    ])

    # 1. Apply Rotation to platform joints
    R = get_rotation_matrix(rx, ry)
    
    angles = []
    for i in range(3):
        # 2. Calculate the rotated world position of each platform joint
        P_i = platform_center + np.dot(R, P_offsets[i])
        
        # 3. Vector from base joint to platform joint
        vec = P_i - B[i]
        
        # Distance calculation
        dist_sq = np.sum(vec**2)
        dist = np.sqrt(dist_sq)
        
        if dist > l1 + l2 or dist < abs(l1 - l2):
            raise ValueError(f"Position unreachable for Leg {i+1} at requested tilt/height.")

        # 4. Plane Geometry: Solve for servo angle
        # Project the vector into the vertical plane of the servo arm
        # For a standard 3RRS, the servo moves in a radial plane from the center
        dx = np.sqrt(vec[0]**2 + vec[1]**2)
        # Note: If base joints are rotated, you'd need to project onto the arm's specific plane
        # Here we assume base hinges are perpendicular to the radius to center
        
        # Law of Cosines
        cos_alpha = (l1**2 + dist_sq - l2**2) / (2 * l1 * dist)
        alpha = np.arccos(np.clip(cos_alpha, -1, 1))
        
        beta = np.arctan2(vec[2], dx) # Angle of the vector relative to horizontal
        
        # Total angle (Upward from horizontal)
        servo_angle = beta + alpha
        angles.append(np.degrees(servo_angle))

    return angles

# --- Main Loop ---
L1, L2 = 98, 120

print("--- 3RRS Kinematics Controller ---")
while True:
    try:
        h = float(input("\nEnter Height (mm): "))
        ax = float(input("Enter Tilt X (degrees): "))
        ay = float(input("Enter Tilt Y (degrees): "))

        servo_angles = inverse_kinematics_3rrs(h, ax, ay, L1, L2)
        
        print("-" * 30)
        for i, ang in enumerate(servo_angles, 1):
            print(f"Servo {i}: {ang:.2f}°")
            
    except ValueError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        break