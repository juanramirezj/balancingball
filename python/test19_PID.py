import numpy as np
import time
from st3215 import ST3215
from class_OpenMV_cam import Camera

# ===============================
# CONFIGURATION
# ===============================

H = 180.0
CENTER_X = 164.0
CENTER_Y = 114.0

MAX_ANGLE = 16.0   # degrees safety limit
LOOP_DT = 0.005 # 0.02  #0.005   #200 Hz  # 0.02     # 50 Hz control loop

# PID gains (you will tune these)
Kp = 0.015 #0.0145
Ki = 0.003
Kd = 0.08
Kv = 0.03   # velocity gain
Ki_pos = 0.0015  

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

def friction_boost(angle, threshold=0.5, boost=1.2):
    if abs(angle) < threshold:
        return angle + np.sign(angle) * boost
    return angle

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

pid_x = PID(Kp, Ki, Kd)
pid_y = PID(Kp, Ki, Kd)

print("Ball balancing started...")

# ===============================
# MAIN LOOP
# ===============================

ax = 0.0
ay = 0.0

prev_ax = 0.0
prev_ay = 0.0

prev_x = 0
prev_y = 0

vel_x = 0
vel_y = 0

# ---- identified inverse Jacobian ----
Jinv = 8.0 *np.array([
    [-0.054,  0.027],
    [-0.121, -0.486]
])


CENTER_OFFSET_X = -62
CENTER_OFFSET_Y = 10

prev_x, prev_y = cam.find_ball()
vel_x = vel_y = 0.0


CAM_ROT = np.radians(90)   # ← WILL CALIBRATE

R_cam2plat = np.array([
    [np.cos(CAM_ROT), -np.sin(CAM_ROT)],
    [np.sin(CAM_ROT),  np.cos(CAM_ROT)]
])

int_error = np.array([0.0, 0.0])

while True:
    start_time = time.time()

    ball_x, ball_y = cam.find_ball()

    if ball_x == -1:
        time.sleep(LOOP_DT)
        continue

    alpha = 0.8   # smoothing (0.7–0.9 works well)

    raw_vx = (ball_x - prev_x) / LOOP_DT
    raw_vy = (ball_y - prev_y) / LOOP_DT

    vel_x = alpha * vel_x + (1 - alpha) * raw_vx
    vel_y = alpha * vel_y + (1 - alpha) * raw_vy

    prev_x = ball_x
    prev_y = ball_y

    # Error (ball relative to center)
    error_x = ball_x - CENTER_OFFSET_X
    error_y = ball_y - CENTER_OFFSET_Y
    
    # rotation between camera and platform (degrees)


    e_platform = R_cam2plat @ np.array([error_x, error_y])
    E_MAX = 60.0
    e_platform = E_MAX * np.tanh(e_platform / E_MAX)

    int_error += e_platform * LOOP_DT  
    INT_LIM = 200
    int_error = np.clip(int_error, -INT_LIM, INT_LIM)  
    
    v_platform = R_cam2plat @ np.array([vel_x, vel_y])
    VEL_MAX = 150
    v_platform = np.clip(v_platform, -VEL_MAX, VEL_MAX)
    
    # nonlinear compression
    E_MAX = 60.0   # pixels (tune 40–80)


    
    
    # --- PD control with velocity feedback ---
    dist = np.linalg.norm(e_platform)
    Kv_eff = Kv * (1 / (1 + 0.01 * dist))
    
    
    u = (
        Kp * (Jinv @ e_platform)
        + Ki_pos * (Jinv @ int_error)
        - Kv_eff * (Jinv @ v_platform)
    )
    
    #print("Error (platform): ({:.2f}, {:.2f}), Velocity (platform): ({:.2f}, {:.2f}), Control output (ax, ay): ({:.2f}, {:.2f})".format(e_platform[0], e_platform[1], v_platform[0], v_platform[1], u[0], u[1]))
    
    #ax += u[0] * LOOP_DT
    #ay += u[1] * LOOP_DT
    
    ax = u[0]
    ay = u[1]
       
    MAX_RATE = 120 * LOOP_DT   # deg/sec

    ax = np.clip(ax, prev_ax - MAX_RATE, prev_ax + MAX_RATE)
    ay = np.clip(ay, prev_ay - MAX_RATE, prev_ay + MAX_RATE)

    prev_ax = ax
    prev_ay = ay

    tilt_mag = np.sqrt(ax**2 + ay**2)

    if tilt_mag > MAX_ANGLE:
        scale = MAX_ANGLE / tilt_mag
        ax *= scale
        ay *= scale

    # Inverse kinematics
    positions = solve_ik(H, ax, ay)

    #print("Camera position: ({:.2f}, {:.2f}), Error: ({:.2f}, {:.2f}), Velocity: ({:.2f}, {:.2f}), Angles: ({:.2f}, {:.2f}), Servo positions: ({:.2f}, {:.2f}, {:.2f}), Tilt magniture: {:.2f}".format(ball_x, ball_y, error_x, error_y, vel_x, vel_y, ax, ay, positions[0], positions[1], positions[2], np.sqrt(ax**2 + ay**2) ))
        
        # Move servos
    for i in range(3):
        servo.MoveTo(ids[i], positions[i],
                     wait=False, speed=2400, acc=50)

    # Maintain loop timing
    elapsed = time.time() - start_time
    sleep_time = LOOP_DT - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)