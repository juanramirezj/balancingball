import numpy as np
import time
from st3215 import ST3215
from class_OpenMV_cam import Camera
import os
import yaml

# ===============================
# CONFIGURATION
# ===============================

H = 180 #180.0
CENTER_X = 164.0
CENTER_Y = 114.0

MAX_ANGLE = 16.0   # degrees safety limit
LOOP_DT = 0.005 # 0.02  #0.005   #200 Hz  # 0.02     # 50 Hz control loop

# PID gains (you will tune these)
Kp = 0.06 #0.0145
Ki = 0.003
Kd = 0.08
Kv = 0.03   # velocity gain
Ki_pos = 0.01  

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


CALIB_FILE = "coords.yaml"


def save_calibration(Jinv):
    data = {
        "Jinv": Jinv.tolist()
    }

    with open(CALIB_FILE, "w") as f:
        yaml.dump(data, f)

    print("Calibration saved to coords.yaml")


def load_calibration():
    with open(CALIB_FILE, "r") as f:
        data = yaml.safe_load(f)

    Jinv = np.array(data["Jinv"])
    print("Calibration loaded from coords.yaml")
    return Jinv

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

def measure_ball_average(samples=20):
    xs, ys = [], []

    for _ in range(samples):
        x, y = cam.find_ball()
        if x != -1:
            xs.append(x)
            ys.append(y)
        time.sleep(0.02)

    return np.mean(xs), np.mean(ys)


def calibrate_mapping():

    print("Calibration starting...")
     
    # --- baseline ---
    ax = 0
    ay = 0
    move_platform(ax, ay)
    input("Place ball near center and press Enter...")
    x0, y0 = measure_ball_average()

    STEP = 8.0   # degrees tilt

    # ---------- AX TEST ----------
    print("Tilting AX...")
    move_platform(STEP, 0)
    time.sleep(10)
    #time.sleep(1.5)

    x1, y1 = measure_ball_average()

    dx1 = x1 - x0
    dy1 = y1 - y0

    # return to flat
    move_platform(0,0)
    time.sleep(1)

    # ---------- AY TEST ----------
    print("Tilting AY...")
    move_platform(0, STEP)
    time.sleep(10)
    x2, y2 = measure_ball_average()

    dx2 = x2 - x0
    dy2 = y2 - y0

    move_platform(0,0)

    # Build matrix
    M = np.array([
        [dx1, dx2],
        [dy1, dy2]
    ])

    print("Measured mapping M:")
    print(M)


    Jinv = STEP * np.linalg.inv(M)

    print("Computed inverse mapping:")
    print(Jinv)

    return Jinv

def validate_calibration(Jinv):

    test_error = np.array([30.0, 0.0])  # pretend ball right

    u = Jinv @ test_error

    print("TEST RESPONSE for +X error:", u)

    print("""
EXPECTED:
Ball right (+X) → platform tilts LEFT
(ax should push ball toward center)
""")
    
    
def move_platform(ax, ay):
    positions = solve_ik(H, ax, ay)
    for i in range(3):
        servo.MoveTo(ids[i], positions[i],
                     wait=False, speed=1500, acc=40)

def get_rotation_matrix(ax_deg, ay_deg):
    ax, ay = np.radians(ax_deg), np.radians(ay_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
    return np.dot(Ry, Rx)

def fix_axis_orientation(Jinv):

    candidates = [
        Jinv,
        np.array([[1,0],[0,-1]]) @ Jinv,
        np.array([[-1,0],[0,1]]) @ Jinv,
        -Jinv,
        Jinv[[1,0],:],     # swap axes
        -Jinv[[1,0],:]
    ]

    print("\nTesting axis orientations...\n")

    for i, J in enumerate(candidates):
        print(f"Candidate {i}:")
        print(J)

    choice = int(input("Select correct orientation index: "))

    return candidates[choice]


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

for sid in ids:
    servo.SetMode(sid, 0)        # position mode
    servo.SetAcceleration(sid, 200) #50
    servo.SetSpeed(sid, 2400)
    servo.StartServo(sid)
    
cam = Camera()
cam.center_x = CENTER_X
cam.center_y = CENTER_Y

pid_x = PID(Kp, Ki, Kd)
pid_y = PID(Kp, Ki, Kd)

print("Ball balancing started...")
# ---------------------------------
# Calibration manager
# ---------------------------------
if os.path.exists(CALIB_FILE):

    answer = input(
        "Calibration file found. Use this file (Y/N)? N=recalibrate: "
    ).strip().upper()

    if answer == "Y":
        Jinv = load_calibration()
    else:
        Jinv = calibrate_mapping()
        Jinv = fix_axis_orientation(Jinv)
        save_calibration(Jinv)

else:
    print("No calibration file found. Running calibration...")
    Jinv = calibrate_mapping()
    save_calibration(Jinv)

validate_calibration(Jinv)

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


#CENTER_OFFSET_X = -62
#CENTER_OFFSET_Y = 10
CENTER_OFFSET_X = 0
CENTER_OFFSET_Y = 0

prev_x, prev_y = cam.find_ball()
vel_x = vel_y = 0.0




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
    
    
    # --- errors directly in camera frame ---
    e_platform = np.array([error_x, error_y])
    v_platform = np.array([vel_x, vel_y])

    # nonlinear compression
    E_MAX = 60.0
    e_platform = E_MAX * np.tanh(e_platform / E_MAX)

    # integral
    int_error += e_platform * LOOP_DT
    int_error = np.clip(int_error, -200, 200)

    # adaptive damping
    dist = np.linalg.norm(e_platform)
    Kv_eff = Kv / (1 + 0.01 * dist)

    # calibrated control
    u = (
        Kp * (Jinv @ e_platform)
        + Ki_pos * (Jinv @ int_error)
        - Kv_eff * (Jinv @ v_platform)
    )

    # integrate tilt
    #ax += u[0] * LOOP_DT
    #ay += u[1] * LOOP_DT
        
    ax = u[0]
    ay = u[1]     

    MIN_TILT = 0.8  # degrees

    if np.linalg.norm(e_platform) > 5:
        if abs(ax) < MIN_TILT:
            ax = np.sign(ax) * MIN_TILT
        if abs(ay) < MIN_TILT:
            ay = np.sign(ay) * MIN_TILT
           
    MAX_RATE = 120 * LOOP_DT   # deg/sec

    #temporarily disabled
    #ax = np.clip(ax, prev_ax - MAX_RATE, prev_ax + MAX_RATE)
    #ay = np.clip(ay, prev_ay - MAX_RATE, prev_ay + MAX_RATE)

    prev_ax = ax
    prev_ay = ay

    tilt_mag = np.sqrt(ax**2 + ay**2)

    if tilt_mag > MAX_ANGLE:
        scale = MAX_ANGLE / tilt_mag
        ax *= scale
        ay *= scale

  
    # Inverse kinematics

    positions = solve_ik(H, ax, ay)

    
    #print("Error (platform): ({:.2f}, {:.2f}), Velocity (platform): ({:.2f}, {:.2f}), Control output (ax, ay): ({:.2f}, {:.2f})".format(e_platform[0], e_platform[1], v_platform[0], v_platform[1], u[0], u[1]))
    #print("Camera position: ({:.2f}, {:.2f}), Error: ({:.2f}, {:.2f}), Velocity: ({:.2f}, {:.2f}), Angles: ({:.2f}, {:.2f}), Servo positions: ({:.2f}, {:.2f}, {:.2f}), Tilt magniture: {:.2f}".format(ball_x, ball_y, error_x, error_y, vel_x, vel_y, ax, ay, positions[0], positions[1], positions[2], np.sqrt(ax**2 + ay**2) ))
        
    # Move servos
    for i in range(3):
        #servo.MoveTo(ids[i], positions[i], wait=False, speed=2400, acc=50)
        servo.WritePosition(ids[i], positions[i])

    # Maintain loop timing
    elapsed = time.time() - start_time
    sleep_time = LOOP_DT - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)