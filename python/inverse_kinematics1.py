import math
import numpy as np

def deg(x):
    return math.degrees(x)

def rrs_inverse_kinematics(Pz, L1, L2, R_base, R_platform, tilt_deg=(0, 0)):
    """
    Calculate inverse kinematics for 3-RRS parallel manipulator.
    
    Parameters:
        Pz: Height of the platform center [mm]
        L1: Lower link length (servo arm) [mm]
        L2: Upper link length [mm]
        R_base: Radius of the base joint circle [mm]
        R_platform: Radius of the platform joint circle [mm]
        tilt_deg: (theta, phi) in degrees; rotations around X and Y axis

    Returns:
        list of servo angles [θ1, θ2, θ3] in degrees
    """
    print("Parameters for inverse kinematics:   Pz:", Pz, "L1:", L1, "L2:", L2, "R_base:", R_base, "R_platform:", R_platform, "tilt_deg:", tilt_deg)
    theta_x, phi_y = map(math.radians, tilt_deg)

    # Rotation matrix for platform tilt
    R = np.array([
        [1, 0, 0],
        [0, math.cos(theta_x), -math.sin(theta_x)],
        [0, math.sin(theta_x), math.cos(theta_x)]
    ]) @ np.array([
        [math.cos(phi_y), 0, math.sin(phi_y)],
        [0, 1, 0],
        [-math.sin(phi_y), 0, math.cos(phi_y)]
    ])

    # Platform joint positions (local frame)
    angle_offsets = [0, 120, 240]
    platform_joints = []
    for angle_deg in angle_offsets:
        rad = math.radians(angle_deg)
        x = R_platform * math.cos(rad)
        y = R_platform * math.sin(rad)
        z = 0
        platform_joints.append(np.array([x, y, z]))

    # Base joint positions (global frame)
    base_joints = []
    for angle_deg in angle_offsets:
        rad = math.radians(angle_deg)
        x = R_base * math.cos(rad)
        y = R_base * math.sin(rad)
        z = 0
        base_joints.append(np.array([x, y, 0]))

    servo_angles = []

    for i in range(3):
        B = base_joints[i]
        P = platform_joints[i]

        # Position of platform joint in global frame
        P_global = R @ P + np.array([0, 0, Pz])

        # Vector from base joint to platform joint
        D = P_global - B
        dx, dy, dz = D

        # Project to plane of lower arm rotation
        # Assume rotation plane is vertical (YZ)
        # Use Law of Cosines to compute angle θ
        a = L1
        b = L2
        c = np.linalg.norm(D)

        if c > (a + b):
            raise ValueError(f"Target is out of reach for leg {i+1}.")

        # Law of cosines
        cos_theta = (a**2 + c**2 - b**2) / (2 * a * c)
        theta = math.acos(np.clip(cos_theta, -1.0, 1.0))

        # Compute elevation angle from servo axis to platform joint
        servo_plane_angle = math.atan2(dz, np.linalg.norm([dx, dy]))
        servo_angle = theta + servo_plane_angle

        servo_angles.append(180-deg(servo_angle))

    return servo_angles


# Example usage:
if __name__ == "__main__":
    # Manipulator dimensions in mm
    L1 = 98       # lower link (servo arm)
    L2 = 116      # upper link
    R_base = 92   # base radius
    R_platform = 65  # platform radius
    Pz = 110      # desired platform height
    # self.L = [0.065, 0.116, 0.098, 0.092]  # in meters#
    # Optional tilt (theta_x, p_
    print("Inverse Kinematics Results:")
    tilt_deg = (0, 0)  # No tilt
    angles = rrs_inverse_kinematics(Pz=110, L1=98, L2=116, R_base=92, R_platform=65, tilt_deg=(0,0))
    print(f"Servo angles inverse_kinematics1: {angles}\n\n")
    
    angles = rrs_inverse_kinematics(Pz=175, L1=98, L2=116, R_base=92, R_platform=65, tilt_deg=(0,0))
    print(f"Servo angles inverse_kinematics1: {angles}\n\n")
    
    
    
    #tilt_deg = (5, 5)  # Example tilt
    #angles_tilted = rrs_inverse_kinematics(Pz, L1, L2, R_base, R_platform, tilt_deg)
    #print(f"Servo angles with tilt inverse_kinematics2: {tilt_deg}: {angles_tilted}")
