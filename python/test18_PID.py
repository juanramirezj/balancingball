import numpy as np
import time
from st3215 import ST3215
from class_OpenMV_cam import Camera

# ===============================
# CONFIGURATION
# ===============================
L1 = 100.0  # Servo arm
L2 = 114.0  # Rod link
BASE_RADIUS = 92.0
PLAT_RADIUS = 65.0
H = 150.0   # Adjusted Height (Ensure L1+L2 > dist)

CENTER_X = 176 
CENTER_Y = 122 

MAX_ANGLE = 10.0 # degrees
TARGET_DT = 0.02  # 50 Hz

# Tuned PD Gains (Start low and increase Kd first)
KP = 0.08   
KD = 0.04   

# ===============================
# KINEMATICS ENGINE
# ===============================

def solve_ik(h, ax_deg, ay_deg):
    ax, ay = np.radians(ax_deg), np.radians(ay_deg)
    
    # Rotation Matrix (Pitch/Roll)
    Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
    R = Ry @ Rx

    # Base and Platform joint positions (120 deg apart)
    angles = np.radians([0, 120, 240])
    B = np.array([[BASE_RADIUS * np.cos(a), BASE_RADIUS * np.sin(a), 0] for a in angles])
    P_local = np.array([[PLAT_RADIUS * np.cos(a), PLAT_RADIUS * np.sin(a), 0] for a in angles])
    
    target_positions = []
    for i in range(3):
        # Platform joint in world space
        Pi = np.array([0, 0, h]) + R @ P_local[i]
        
        # Vector from base joint to platform joint
        vec = Pi - B[i]
        
        # Local 2D coordinates in the plane of the servo arm
        # x_loc is the radial distance from base joint
        # We use the angle of the base joint to define the plane
        angle_base = angles[i]
        x_loc = (Pi[0] - B[i][0]) * np.cos(angle_base) + (Pi[1] - B[i][1]) * np.sin(angle_base)
        z_loc = Pi[2] # vertical

        dist_sq = x_loc**2 + z_loc**2
        dist = np.sqrt(dist_sq)

        # Law of Cosines to find servo angle
        cos_alpha = (L1**2 + dist_sq - L2**2) / (2 * L1 * dist)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        
        theta_rel = np.arccos(cos_alpha)
        beta = np.arctan2(z_loc, x_loc)
        
        angle_rad = beta + theta_rel # Elbow out configuration
        angle_deg = np.degrees(angle_rad)
        
        # Mapping: Assuming 0 deg is horizontal, 90 is vertical
        # Adjust 1100/90 factor based on your specific servo calibration
        pos = int(angle_deg * 1100 / 90)
        target_positions.append(pos)

    return target_positions

# ===============================
# MAIN CONTROL LOOP
# ===============================

servo = ST3215('/dev/ttyS0')
ids = [1, 2, 3] # Ensure these match your physical IDs
cam = Camera()

prev_error_x = 0
prev_error_y = 0
prev_time = time.time()

print("System Ready. Balancing...")

while True:
    t_start = time.time()
    dt = t_start - prev_time
    prev_time = t_start

    ball_x, ball_y = cam.find_ball()

    if ball_x != -1:
        # 1. Calculate Error (Normalized or pixels)
        err_x = ball_x - CENTER_X
        err_y = ball_y - CENTER_Y

        # 2. Calculate Derivative (velocity)
        d_x = (err_x - prev_error_x) / dt
        d_y = (err_y - prev_error_y) / dt
        
        prev_error_x = err_x
        prev_error_y = err_y

        # 3. PD Control Output (Tilt angles)
        # Note: You may need to swap signs (+/-) or swap err_x/err_y 
        # depending on how your camera is mounted relative to Servo 1.
        tilt_x = (err_x * KP) + (d_x * KD)
        tilt_y = (err_y * KP) + (d_y * KD)

        # 4. Limit Tilt for safety
        tilt_x = np.clip(tilt_x, -MAX_ANGLE, MAX_ANGLE)
        tilt_y = np.clip(tilt_y, -MAX_ANGLE, MAX_ANGLE)

        # 5. Inverse Kinematics & Move
        try:
            target_steps = solve_ik(H, tilt_x, tilt_y)
            for i in range(3):
                servo.MoveTo(ids[i], target_steps[i], speed=0, acc=0) # Max speed/acc
        except ValueError:
            print("Calculation Error: Out of bounds")

    # Maintain Loop Rate
    elapsed = time.time() - t_start
    if elapsed < TARGET_DT:
        time.sleep(TARGET_DT - elapsed)