import numpy as np
import time
from st3215 import ST3215
from class_OpenMV_cam import Camera
import os
import yaml

# ===============================
# CONFIGURATION
# ===============================

H = 180.0

# find_ball() returns center-relative coordinates → setpoint is (0, 0)
SETPOINT_X = 0.0
SETPOINT_Y = 0.0

MAX_ANGLE = 16.0
LOOP_DT = 0.005  # 200 Hz

# Control gains
Kp     = 0.05   # slightly increased
Ki_pos = 0.000  # reduced — integral was causing windup
Kv     = 0.1

# ===============================
# KINEMATICS
# ===============================
L1 = 100.0
L2 = 114.0
BASE_RADIUS = 92.0
PLAT_RADIUS = 65.0

CALIB_FILE = "coords.yaml"


# ===============================
# CALIBRATION I/O
# ===============================

def save_calibration(Jinv):
    data = {"Jinv": Jinv.tolist()}
    with open(CALIB_FILE, "w") as f:
        yaml.dump(data, f)
    print("Calibration saved.")


def load_calibration():
    with open(CALIB_FILE, "r") as f:
        data = yaml.safe_load(f)
    Jinv = np.array(data["Jinv"])
    print("Calibration loaded.")
    return Jinv


# ===============================
# KINEMATICS HELPERS
# ===============================

def angulo_to_position(angulo):
    return int(angulo * 1100 / 90)


def get_rotation_matrix(ax_deg, ay_deg):
    ax, ay = np.radians(ax_deg), np.radians(ay_deg)
    Rx = np.array([[1, 0, 0],
                   [0,  np.cos(ax), -np.sin(ax)],
                   [0,  np.sin(ax),  np.cos(ax)]])
    Ry = np.array([[ np.cos(ay), 0, np.sin(ay)],
                   [0,           1, 0          ],
                   [-np.sin(ay), 0, np.cos(ay)]])
    return np.dot(Ry, Rx)


def solve_ik(h, ax_deg, ay_deg):
    platform_center = np.array([0.0, 0.0, h])

    B = np.array([
        [BASE_RADIUS, 0, 0],
        [BASE_RADIUS * np.cos(2*np.pi/3), BASE_RADIUS * np.sin(2*np.pi/3), 0],
        [BASE_RADIUS * np.cos(4*np.pi/3), BASE_RADIUS * np.sin(4*np.pi/3), 0],
    ])
    P_offsets = np.array([
        [PLAT_RADIUS, 0, 0],
        [PLAT_RADIUS * np.cos(2*np.pi/3), PLAT_RADIUS * np.sin(2*np.pi/3), 0],
        [PLAT_RADIUS * np.cos(4*np.pi/3), PLAT_RADIUS * np.sin(4*np.pi/3), 0],
    ])

    R = get_rotation_matrix(ax_deg, ay_deg)
    target_positions = []

    for i in range(3):
        Pi  = platform_center + R @ P_offsets[i]
        vec = Pi - B[i]

        radial_dir = B[i][:2] / np.linalg.norm(B[i][:2])
        radial_3d  = np.array([radial_dir[0], radial_dir[1], 0])

        x_local = np.dot(vec, radial_3d)
        z_local = vec[2]
        dist    = np.sqrt(x_local**2 + z_local**2)

        if dist > (L1 + L2) or dist < abs(L1 - L2):
            raise ValueError(f"Leg {i+1} unreachable!")

        cos_alpha = np.clip((L1**2 + dist**2 - L2**2) / (2 * L1 * dist), -1.0, 1.0)
        alpha     = np.arccos(cos_alpha)
        beta      = np.arctan2(z_local, x_local)
        theta     = beta - alpha

        angle_deg = np.clip(np.degrees(theta), 0, 90)
        target_positions.append(angulo_to_position(angle_deg))

    return target_positions


def move_platform(ax, ay):
    positions = solve_ik(H, ax, ay)
    for i in range(3):
        servo.MoveTo(ids[i], positions[i], wait=False, speed=1500, acc=40)


# ===============================
# CALIBRATION
# ===============================

def measure_ball_average(samples=30, timeout=5.0):
    """
    Collect ball readings. Waits up to `timeout` seconds for the ball
    to settle (velocity < threshold) before measuring.
    """
    # Wait for ball to settle
    print("  Waiting for ball to settle...", end="", flush=True)
    t0 = time.time()
    prev_x, prev_y = None, None
    while time.time() - t0 < timeout:
        x, y = cam.find_ball()
        if x != -1:
            if prev_x is not None:
                vx = abs(x - prev_x) / 0.05
                vy = abs(y - prev_y) / 0.05
                if vx < 20 and vy < 20:  # pixels/sec threshold
                    print(" settled.")
                    break
            prev_x, prev_y = x, y
        time.sleep(0.05)
    else:
        print(" timeout — measuring anyway.")

    # Now collect samples
    xs, ys = [], []
    for _ in range(samples):
        x, y = cam.find_ball()
        if x != -1:
            xs.append(x)
            ys.append(y)
        time.sleep(0.03)

    if not xs:
        raise RuntimeError("Ball not detected during calibration!")

    return np.mean(xs), np.mean(ys)


def calibrate_mapping():
    print("\n=== CALIBRATION ===")
    print("Keep the ball on the platform throughout. It will be held by gravity.")
    print("The platform will tilt and we measure where the ball rolls to.\n")

    move_platform(0, 0)
    input("Place ball near CENTER and press Enter...")
    x0, y0 = measure_ball_average()
    print(f"Baseline: ({x0:.1f}, {y0:.1f})  ← should be near (0,0)\n")

    STEP = 5.0  # smaller step → less likely ball rolls off edge

    # --- AX ---
    print(f"Tilting AX = +{STEP}°  (hold ~3 sec)")
    move_platform(STEP, 0)
    x1, y1 = measure_ball_average(timeout=6.0)
    dx1, dy1 = x1 - x0, y1 - y0
    print(f"  AX response: dx={dx1:.1f}, dy={dy1:.1f}")

    move_platform(0, 0)
    time.sleep(1.5)

    # --- AY ---
    print(f"Tilting AY = +{STEP}°  (hold ~3 sec)")
    move_platform(0, STEP)
    x2, y2 = measure_ball_average(timeout=6.0)
    dx2, dy2 = x2 - x0, y2 - y0
    print(f"  AY response: dx={dx2:.1f}, dy={dy2:.1f}")

    move_platform(0, 0)
    time.sleep(1.0)

    # Sanity check — each tilt should move the ball at least 10px
    if abs(dx1) < 8 and abs(dy1) < 8:
        print("\nWARNING: AX tilt produced almost no ball movement!")
        print("The ball may have rolled off, or servo IDs / axes may be swapped.")
    if abs(dx2) < 8 and abs(dy2) < 8:
        print("\nWARNING: AY tilt produced almost no ball movement!")

    M = np.array([[dx1, dx2],
                  [dy1, dy2]])
    print(f"\nMapping matrix M:\n{M}")

    cond = np.linalg.cond(M)
    print(f"Condition number: {cond:.1f}  (good < 10, bad > 50)")
    if cond > 30:
        print("WARNING: Matrix is poorly conditioned. Axes may be nearly parallel.")
        print("Consider rotating the camera or remounting servos 120° apart.\n")

    Jinv = STEP * np.linalg.inv(M)
    print(f"\nJinv:\n{Jinv}")
    return Jinv


def pick_orientation(Jinv):
    """
    Instead of blind candidates, derive the correct sign automatically
    from the physics: tilting platform toward the ball should push
    ball back toward center.

    We check: for a ball at +X (right), does ax go negative (tilt left)?
    If not, we flip the appropriate sign.
    """
    test = np.array([30.0, 0.0])
    u = Jinv @ test

    print(f"\nSign check: ball 30px RIGHT → ax={u[0]:.3f}, ay={u[1]:.3f}")
    print("ax must be NEGATIVE to push ball left (back to center).")

    if u[0] > 0:
        print("→ Flipping X row sign.")
        Jinv[0, :] = -Jinv[0, :]

    # Re-check Y: ball at +Y (down in camera) should give negative ay
    test_y = np.array([0.0, 30.0])
    u_y = Jinv @ test_y
    print(f"Sign check: ball 30px DOWN  → ax={u_y[0]:.3f}, ay={u_y[1]:.3f}")
    print("ay must be NEGATIVE to push ball up (back to center).")

    if u_y[1] > 0:
        print("→ Flipping Y row sign.")
        Jinv[1, :] = -Jinv[1, :]

    u_final = Jinv @ np.array([30.0, 0.0])
    print(f"\nFinal check: ball 30px RIGHT → ax={u_final[0]:.3f}  ← should be negative ✓")
    return Jinv


# ===============================
# INITIALIZE HARDWARE
# ===============================

servo = ST3215('/dev/ttyS0')
ids   = servo.ListServos()

for sid in ids:
    servo.SetMode(sid, 0)
    servo.SetAcceleration(sid, 200)
    servo.SetSpeed(sid, 2400)
    servo.StartServo(sid)

cam = Camera()
cam.center_x = 164.0
cam.center_y = 114.0

print("Ball balancing controller starting...")

# ===============================
# CALIBRATION MANAGER
# ===============================

if os.path.exists(CALIB_FILE):
    answer = input("Calibration file found. Use it? (Y=yes, N=recalibrate): ").strip().upper()
    if answer == "Y":
        Jinv = load_calibration()
        # Always re-validate sign on load
        Jinv = pick_orientation(Jinv)
    else:
        Jinv = calibrate_mapping()
        Jinv = pick_orientation(Jinv)
        save_calibration(Jinv)
else:
    Jinv = calibrate_mapping()
    Jinv = pick_orientation(Jinv)
    save_calibration(Jinv)


# ===============================
# MAIN CONTROL LOOP
# ===============================

ax, ay        = 0.0, 0.0
int_error     = np.array([0.0, 0.0])
vel_x = vel_y = 0.0
alpha_vel     = 0.85   # slightly less smoothing for faster response

prev_x, prev_y = SETPOINT_X, SETPOINT_Y

print("\nControl loop running. Press Ctrl+C to stop.\n")

while True:
    start_time = time.time()

    ball_x, ball_y = cam.find_ball()

    if ball_x == -1:
        # Ball lost — flatten platform gently to avoid runaway
        ax *= 0.9
        ay *= 0.9
        positions = solve_ik(H, ax, ay)
        for i in range(3):
            servo.WritePosition(ids[i], positions[i])
        time.sleep(LOOP_DT)
        continue

    # Velocity (pixels/sec), smoothed
    raw_vx = (ball_x - prev_x) / LOOP_DT
    raw_vy = (ball_y - prev_y) / LOOP_DT
    vel_x  = alpha_vel * vel_x + (1 - alpha_vel) * raw_vx
    vel_y  = alpha_vel * vel_y + (1 - alpha_vel) * raw_vy
    prev_x, prev_y = ball_x, ball_y

    # Error
    error_x = ball_x - SETPOINT_X
    error_y = ball_y - SETPOINT_Y

    e_platform = np.array([error_x, error_y])
    v_platform = np.array([vel_x,   vel_y  ])

    dist = np.linalg.norm(e_platform)

    # Soft saturation on position error only
    E_MAX   = 80.0
    e_sat   = E_MAX * np.tanh(e_platform / E_MAX)

    # Integral (small, just for steady-state bias)
    int_error += e_sat * LOOP_DT
    int_error  = np.clip(int_error, -200, 200)

    # Adaptive damping — more damping close to center
    Kv_eff = Kv * (1.0 + 0.005 * dist)

    # Control output
    u = (  Kp     * (Jinv @ e_sat)
         + Ki_pos * (Jinv @ int_error)
         - Kv_eff * (Jinv @ v_platform) )

    ax, ay = u[0], u[1]

    # NO MIN_TILT CLAMP — it was locking the controller at ±0.5 constantly.
    # The IK and servos handle small angles fine.

    # Safety: clamp total tilt magnitude
    tilt_mag = np.sqrt(ax**2 + ay**2)
    if tilt_mag > MAX_ANGLE:
        scale = MAX_ANGLE / tilt_mag
        ax   *= scale
        ay   *= scale

    positions = solve_ik(H, ax, ay)

    print(f"ball=({ball_x:.1f},{ball_y:.1f}) "
          f"err=({error_x:.1f},{error_y:.1f}) "
          f"vel=({vel_x:.0f},{vel_y:.0f}) "
          f"ax={ax:.3f} ay={ay:.3f} "
          f"|e|={dist:.1f}")

    for i in range(3):
        servo.WritePosition(ids[i], positions[i])

    elapsed = time.time() - start_time
    sleep_time = LOOP_DT - elapsed
    if sleep_time > 0:
        time.sleep(sleep_time)