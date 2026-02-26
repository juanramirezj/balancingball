import numpy as np
import time
from st3215 import ST3215

# --- KINEMATICS SETTINGS ---
L1, L2 = 98, 120  # Link lengths in mm
BASE_RADIUS = 92
PLAT_RADIUS = 65

# --- SERVO SETTINGS ---
# Adjust these based on how you mounted your servo horns
SERVO_CENTER_STEP = 2048  # Usually the middle of the 4096 range
STEPS_PER_DEGREE = 4096 / 360.0 

def get_rotation_matrix(ax_deg, ay_deg):
    ax, ay = np.radians(ax_deg), np.radians(ay_deg)
    Rx = np.array([[1, 0, 0], [0, np.cos(ax), -np.sin(ax)], [0, np.sin(ax), np.cos(ax)]])
    Ry = np.array([[np.cos(ay), 0, np.sin(ay)], [0, 1, 0], [-np.sin(ay), 0, np.cos(ay)]])
    return np.dot(Ry, Rx)

def calculate_servos(h, ax, ay):
    platform_center = np.array([0, 0, h])
    
    # Base positions
    B = np.array([
        [BASE_RADIUS, 0, 0],
        [BASE_RADIUS * np.cos(2*np.pi/3), BASE_RADIUS * np.sin(2*np.pi/3), 0],
        [BASE_RADIUS * np.cos(4*np.pi/3), BASE_RADIUS * np.sin(4*np.pi/3), 0],
    ])
    # Platform offsets
    P_offsets = np.array([
        [PLAT_RADIUS, 0, 0],
        [PLAT_RADIUS * np.cos(2*np.pi/3), PLAT_RADIUS * np.sin(2*np.pi/3), 0],
        [PLAT_RADIUS * np.cos(4*np.pi/3), PLAT_RADIUS * np.sin(4*np.pi/3), 0],
    ])

    R = get_rotation_matrix(ax, ay)
    steps = []

    for i in range(3):
        Pi = platform_center + np.dot(R, P_offsets[i])
        vec = Pi - B[i]
        dist_sq = np.sum(vec**2)
        dist = np.sqrt(dist_sq)

        if dist > (L1 + L2) or dist < abs(L1 - L2):
            raise ValueError(f"Position unreachable for Leg {i+1}")

        # Solve for the angle (in plane)
        dx = np.sqrt(vec[0]**2 + vec[1]**2)
        alpha = np.arccos((L1**2 + dist_sq - L2**2) / (2 * L1 * dist))
        beta = np.arctan2(vec[2], dx)
        
        angle_deg = np.degrees(beta + alpha)
        
        # Map degrees to servo steps
        # Assuming 0 degrees is horizontal (horizontal = SERVO_CENTER_STEP)
        servo_step = int(SERVO_CENTER_STEP + (angle_deg * STEPS_PER_DEGREE))
        steps.append(servo_step)
        
    return steps

# --- HARDWARE INITIALIZATION ---
try:
    servo_hw = ST3215('/dev/ttyS0')
    ids = servo_hw.ListServos()
    if len(ids) < 3:
        print(f"Error: Found only {len(ids)} servos. Need 3.")
        exit()
    print(f"Connected to servos: {ids}")
except Exception as e:
    print(f"Connection failed: {e}")
    exit()

# --- MAIN LOOP ---
print("\n3RRS Controller Ready.")
print("Enter Height (mm), Tilt X (deg), Tilt Y (deg)")

while True:
    user_input = input("\nInput [h, ax, ay] or 'q': ")
    if user_input.lower() == 'q':
        break
    
    try:
        h, ax, ay = map(float, user_input.split(','))
        
        # Calculate target steps
        target_steps = calculate_servos(h, ax, ay)
        
        # Move Servos
        for i in range(3):
            # Applying move to IDs found in the list
            servo_hw.MoveTo(ids[i], target_steps[i], wait=False, speed=2400, acc=50)
            print(f"Servo {ids[i]} -> Step {target_steps[i]}")

    except ValueError as e:
        print(f"Input Error: {e}")
    except Exception as e:
        print(f"Runtime Error: {e}")