import numpy as np
import time
from st3215 import ST3215
from class_OpenMV_cam import Camera

# ===============================
# CONFIGURATION
# ===============================

H = 165.0
CENTER_X = 176 # 164.0
CENTER_Y = 122 # 114.0

MAX_ANGLE = 8.0 #5   # degrees safety limit
LOOP_DT = 0.005 #0.02  #0.005   #200 Hz  # 0.02     # 50 Hz control loop

# PID gains (you will tune these)
Kd = 0.006
Kp = 0.03
Kv = 0.003   # velocity gain

# ===============================
# PID CLASS
# ===============================

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        return (self.kp * error +
                self.ki * self.integral +
                self.kd * derivative)
        
        
# --- KINEMATICS CONFIGURATION ---
L1 = 100.0  # Link 1 (mm)
L2 = 114.0 # Link 2 (mm)
BASE_RADIUS = 92.0
PLAT_RADIUS = 65.0

# --- SERVO MAPPING FUNCTION ---
def angulo_to_position(angulo):
    """
    As per your definition: 90 degrees = 1100 steps.
    Assuming 0 degrees (horizontal) = 0 steps.
    """
    return int(angulo * 1100 / 90)

def get_rotation_matrix(ax_deg, ay_deg):
    ax, ay = np.radians(ax_deg), np.radians(ay_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
    return np.dot(Ry, Rx)

def solve_ik(h, ax_deg, ay_deg):
    platform_center = np.array([0.0, 0.0, h])

    # --- Base joints (120° apart) ---
    B = np.array([
        [BASE_RADIUS, 0, 0],
        [BASE_RADIUS * np.cos(2*np.pi/3), BASE_RADIUS * np.sin(2*np.pi/3), 0],
        [BASE_RADIUS * np.cos(4*np.pi/3), BASE_RADIUS * np.sin(4*np.pi/3), 0],
    ])

    # --- Platform joints (120° apart) ---
    P_offsets = np.array([
        [PLAT_RADIUS, 0, 0],
        [PLAT_RADIUS * np.cos(2*np.pi/3), PLAT_RADIUS * np.sin(2*np.pi/3), 0],
        [PLAT_RADIUS * np.cos(4*np.pi/3), PLAT_RADIUS * np.sin(4*np.pi/3), 0],
    ])

    R = get_rotation_matrix(ax_deg, ay_deg)

    target_positions = []

    for i in range(3):

        # Platform joint in world coordinates
        Pi = platform_center + R @ P_offsets[i]

        # Vector from base joint to platform joint
        vec = Pi - B[i]

        # --- Build servo radial direction (unit vector in XY plane) ---
        radial_dir = B[i][:2] / np.linalg.norm(B[i][:2])
        radial_3d = np.array([radial_dir[0], radial_dir[1], 0])

        # --- Project vec into servo plane ---
        x_local = np.dot(vec, radial_3d)      # radial direction
        z_local = vec[2]                      # vertical

        dist = np.sqrt(x_local**2 + z_local**2)

        if dist > (L1 + L2) or dist < abs(L1 - L2):
            raise ValueError(f"Leg {i+1} unreachable!")

        # --- 2-link planar IK ---
        cos_alpha = (L1**2 + dist**2 - L2**2) / (2 * L1 * dist)
        cos_alpha = np.clip(cos_alpha, -1.0, 1.0)
        alpha = np.arccos(cos_alpha)

        beta = np.arctan2(z_local, x_local)

        theta = beta - alpha   # <-- CORRECT branch

        angle_deg = np.degrees(theta)
        angle_deg = max(0, min(90, angle_deg))

        pos = angulo_to_position(angle_deg)
        target_positions.append(pos)

    return target_positions

def compute_home_height():
    # servo angle = 90 degrees
    theta = np.radians(90)

    # elbow position for leg 1
    B = np.array([BASE_RADIUS, 0, 0])

    elbow = B + np.array([
        L1 * np.cos(theta),  # radial
        0,
        L1 * np.sin(theta)
    ])

    # platform joint radial offset difference
    radial_offset = BASE_RADIUS - PLAT_RADIUS

    # horizontal distance from elbow to platform joint
    dx = radial_offset

    # compute vertical height from law of cosines
    dz = np.sqrt(L2**2 - dx**2)

    return elbow[2] + dz


def compute_jacobian(h, ax_deg, ay_deg, delta=0.01):
    """
    Numerical Jacobian:
    Returns 3x2 matrix:
        d(theta_i)/d(ax)
        d(theta_i)/d(ay)
    Units: degrees per degree
    """

    # Current angles
    theta0 = np.array(solve_ik(h, ax_deg, ay_deg)) * 90 / 1100  # convert back to degrees

    # Perturb AX
    theta_ax = np.array(solve_ik(h, ax_deg + delta, ay_deg)) * 90 / 1100

    # Perturb AY
    theta_ay = np.array(solve_ik(h, ax_deg, ay_deg + delta)) * 90 / 1100

    # Numerical derivatives
    dtheta_dax = (theta_ax - theta0) / delta
    dtheta_day = (theta_ay - theta0) / delta

    # Build Jacobian (3x2)
    J = np.column_stack((dtheta_dax, dtheta_day))

    return J

# ===============================
# INITIALIZE HARDWARE
# ===============================

servo = ST3215('/dev/ttyS0')
ids = servo.ListServos()

cam = Camera()
cam.center_x = CENTER_X
cam.center_y = CENTER_Y


# ===============================
# PLATFORM AXES (AUTO CALIBRATED)
# ===============================
# CAMERA → PLATFORM calibration matrix
CAL = np.array([
    [ 0.035, -0.020],
    [-0.010, -0.030]
])

# --- measured servo positions in camera coordinates ---
S1 = np.array([99.0, 96.0])     # servo 1
S2 = np.array([227.0, 61.0])    # servo 2
S3 = np.array([184.0, 187.0])   # servo 3

# normalize
axis_ay = np.array([-77.0, -26.0])   # toward servo 1
axis_ax = np.array([-34.5, 19.5])    # midpoint of servo1 & servo3


axis_ax /= np.linalg.norm(axis_ax)
axis_ay /= np.linalg.norm(axis_ay)

print("Ball balancing started...")

# ===============================
# MAIN LOOP
# ===============================

# ===============================
# MAIN LOOP
# ===============================

ax = 0.0
ay = 0.0

prev_error_x = 0
prev_error_y = 0
prev_vel_x = 0
prev_vel_y = 0

TARGET_DT = 0.02   # 50 Hz (your real speed)
prev_time = time.time()



while True:
    loop_start = time.time()

    dt = loop_start - prev_time
    dt = min(max(dt, 0.001), 0.02)
    prev_time = loop_start

    ball_x, ball_y = cam.find_ball()
    
    print(f"Ball position: ({ball_x}, {ball_y})")

    if ball_x == -1:
        continue

    # ----------------------------
    # 1. Compute current error
    # ----------------------------
    # --- camera error ---
    ex = ball_x
    ey = -ball_y   # keep Y inversion if needed

    e = np.array([ex, ey])
    
    tilt = -Kp * CAL @ e

    ax = tilt[0]
    ay = tilt[1]

    ax = np.clip(ax, -MAX_ANGLE, MAX_ANGLE)
    ay = np.clip(ay, -MAX_ANGLE, MAX_ANGLE)

    print(f"Error: {e}, ax: {ax:.2f}, ay: {ay:.2f}")

    positions = solve_ik(H, ax, ay)

    for i in range(3):
        servo.MoveTo(ids[i], positions[i],
                     wait=False, speed=2400, acc=50)

    # ----------------------------
    # Maintain 50 Hz
    # ----------------------------
    elapsed = time.time() - loop_start
    sleep_time = TARGET_DT - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)