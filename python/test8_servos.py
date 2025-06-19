import math
import time

# Constants
BASE_RADIUS = 92      # mm
ARM1_LENGTH = 98      # mm
ARM2_LENGTH = 120     # mm
PLATFORM_RADIUS = 65  # mm
PLATFORM_HEIGHT = 190 # mm
SERVO_ANGLES = [0, 120, 240]  # Degrees

# PID gains
Kp = 0.04
Ki = 0.001
Kd = 0.02

# PID state
pid_state = {
    'x': {'integral': 0, 'prev_error': 0},
    'y': {'integral': 0, 'prev_error': 0},
}
last_time = time.time()

def move_servo(angle, id):
    print(f"Servo {id}: Move to {angle:.2f}°")

def pid_control(posx, posy):
    global last_time, pid_state

    current_time = time.time()
    dt = current_time - last_time
    if dt <= 0:
        dt = 0.01  # avoid division by zero

    # PID for X axis (pitch)
    error_x = -posx
    pid_state['x']['integral'] += error_x * dt
    derivative_x = (error_x - pid_state['x']['prev_error']) / dt
    tilt_x = (Kp * error_x +
              Ki * pid_state['x']['integral'] +
              Kd * derivative_x)
    pid_state['x']['prev_error'] = error_x

    # PID for Y axis (roll)
    error_y = -posy
    pid_state['y']['integral'] += error_y * dt
    derivative_y = (error_y - pid_state['y']['prev_error']) / dt
    tilt_y = (Kp * error_y +
              Ki * pid_state['y']['integral'] +
              Kd * derivative_y)
    pid_state['y']['prev_error'] = error_y

    last_time = current_time

    # Convert tilt angles to Z offsets at each joint
    dz = PLATFORM_HEIGHT
    dx = dz * math.tan(tilt_x)
    dy = dz * math.tan(tilt_y)

    for i in range(3):
        angle_deg = SERVO_ANGLES[i]
        angle_rad = math.radians(angle_deg)

        # Platform point
        px = PLATFORM_RADIUS * math.cos(angle_rad)
        py = PLATFORM_RADIUS * math.sin(angle_rad)

        # Z offset due to tilt
        dz_i = dx * math.cos(angle_rad) + dy * math.sin(angle_rad)
        z_target = dz + dz_i

        # Horizontal distance
        bx = BASE_RADIUS * math.cos(angle_rad)
        by = BASE_RADIUS * math.sin(angle_rad)
        dx_i = px - bx
        dy_i = py - by
        d_xy = math.sqrt(dx_i**2 + dy_i**2)
        d_total = math.sqrt(d_xy**2 + z_target**2)

        # Inverse Kinematics
        a = ARM1_LENGTH
        b = ARM2_LENGTH
        c = d_total

        try:
            cos_theta = (a**2 + c**2 - b**2) / (2 * a * c)
            theta = math.acos(max(-1, min(1, cos_theta)))
            alpha = math.atan2(z_target, d_xy)
            servo_angle = math.degrees(theta + alpha)
        except ValueError:
            servo_angle = 90  # default if unreachable
            print(f"Servo {i+1} target unreachable.")

        move_servo(servo_angle, i + 1)

# Example loop
if __name__ == "__main__":
    import random

    for _ in range(100):
        # Simulate random ball movement
        posx = random.uniform(-15, 15)
        posy = random.uniform(-15, 15)
        pid_control(posx, posy)
        time.sleep(0.05)  # simulate camera update rate (20Hz)
