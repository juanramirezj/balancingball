import numpy as np

def inverse_kinematics_rrs(roll_deg, pitch_deg, z, base_radius=92, platform_radius=65, upper_arm=120, lower_arm=98):
    """
    Calculates the inverse kinematics for a 3-DOF RRS parallel manipulator.
    Args:
        roll_deg: Platform tilt angle around X axis (degrees)
        pitch_deg: Platform tilt angle around Y axis (degrees)
        z: Height of the center of the platform (mm)
        base_radius: Radius of the base (mm)
        platform_radius: Radius of the moving platform (mm)
        upper_arm: Length of the upper arm (mm)
        lower_arm: Length of the lower arm (mm)
    Returns:
        List of three joint angles (degrees) for the actuators [theta1, theta2, theta3]
    """
    # Convert angles to radians
    roll = np.deg2rad(roll_deg)
    pitch = np.deg2rad(pitch_deg)

    # Angles for the three base joints (120 degrees apart)
    base_angles = [0, 120, 240]
    joint_angles = []

    # Rotation matrix for platform orientation
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    R = Ry @ Rx

    for i in range(3):
        # Base joint position
        theta = np.deg2rad(base_angles[i])
        base_x = base_radius * np.cos(theta)
        base_y = base_radius * np.sin(theta)

        # Platform joint position (relative to platform center)
        plat_theta = np.deg2rad(base_angles[i])
        local_plat = np.array([
            platform_radius * np.cos(plat_theta),
            platform_radius * np.sin(plat_theta),
            0
        ])
        # Rotate and translate platform joint
        plat_pos = R @ local_plat + np.array([0, 0, z])
        plat_x, plat_y, plat_z = plat_pos

        # Vector from base to platform joint
        dx = plat_x - base_x
        dy = plat_y - base_y
        dz = plat_z

        # Distance between base and platform joint
        d = np.sqrt(dx**2 + dy**2 + dz**2)

        # Law of cosines to find angle at base joint
        if d > (upper_arm + lower_arm) or d < abs(upper_arm - lower_arm):
            raise ValueError("Position unreachable for leg {}".format(i+1))

        # Project onto the plane of the arm
        a = upper_arm
        b = d
        c = lower_arm

        # Angle at base joint (shoulder)
        cos_angle = (a**2 + b**2 - c**2) / (2 * a * b)
        cos_angle = np.clip(cos_angle, -1, 1)
        angle = np.arccos(cos_angle)

        # Angle from base to platform in XY
        phi = np.arctan2(dz, np.sqrt(dx**2 + dy**2))

        # Total angle for actuator
        theta_deg = np.rad2deg(phi + angle)
        joint_angles.append(theta_deg)

    return joint_angles

if __name__ == "__main__":
    # Example usage: roll=5 deg, pitch=10 deg, z=-300 mm
    roll, pitch, z = 0, 0, 170
    try:
        angles = inverse_kinematics_rrs(roll, pitch, z)
        print("Joint angles (degrees):", angles)
    except ValueError as e:
        print("Error:", e)