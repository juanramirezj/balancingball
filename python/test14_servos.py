import numpy as np
import time
from st3215 import ST3215

# --- KINEMATICS CONFIGURATION ---
L1 = 98.0  # Link 1 (mm)
L2 = 120.0 # Link 2 (mm)
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
    platform_center = np.array([0, 0, h])
    
    # Base positions (120 deg apart)
    B = np.array([
        [BASE_RADIUS, 0, 0],
        [BASE_RADIUS * np.cos(2*np.pi/3), BASE_RADIUS * np.sin(2*np.pi/3), 0],
        [BASE_RADIUS * np.cos(4*np.pi/3), BASE_RADIUS * np.sin(4*np.pi/3), 0],
    ])
    
    # Platform joint offsets
    P_offsets = np.array([
        [PLAT_RADIUS, 0, 0],
        [PLAT_RADIUS * np.cos(2*np.pi/3), PLAT_RADIUS * np.sin(2*np.pi/3), 0],
        [PLAT_RADIUS * np.cos(4*np.pi/3), PLAT_RADIUS * np.sin(4*np.pi/3), 0],
    ])

    R = get_rotation_matrix(ax_deg, ay_deg)
    target_positions = []

    for i in range(3):
        # Rotate platform joints and add height
        Pi = platform_center + np.dot(R, P_offsets[i])
        
        # Vector from base joint to platform joint
        vec = Pi - B[i]
        dist_sq = np.sum(vec**2)
        dist = np.sqrt(dist_sq)

        if dist > (L1 + L2) or dist < abs(L1 - L2):
            raise ValueError(f"Leg {i+1} unreachable!")

        # 2D Projection for the servo plane
        dx = np.sqrt(vec[0]**2 + vec[1]**2)
        
        # Law of Cosines for the 'elbow' angle
        alpha = np.arccos((L1**2 + dist_sq - L2**2) / (2 * L1 * dist))
        # Base vector angle
        beta = np.arctan2(vec[2], dx)
        
        angle_deg = np.degrees(beta + alpha)
        
        # Convert degrees to servo steps using your formula
        pos = angulo_to_position(angle_deg)
        target_positions.append(pos)
        
    return target_positions

# --- INITIALIZE HARDWARE ---
try:
    servo = ST3215('/dev/ttyS0')
    ids = servo.ListServos()
    if len(ids) < 3:
        print(f"Only found {len(ids)} servos. Check connections.")
        exit()
except Exception as e:
    print(f"Hardware error: {e}")
    exit()

# --- MAIN CONTROL LOOP ---
print("Enter parameters: Height(mm), AngleX(deg), AngleY(deg)")
while True:
    entry = input("\nInput (e.g. 150, 10, -5) or 'q' to quit: ")
    if entry.lower() == 'q':
        break
    
    try:
        h, ax, ay = map(float, entry.split(','))
        
        # Calculate steps
        positions = solve_ik(h, ax, ay)
        
        print(f"Moving to: H={h}mm, AX={ax}°, AY={ay}°")
        for i in range(3):
            print(f"  Servo {ids[i]} -> Position: {positions[i]}")
            #servo.MoveTo(ids[i], positions[i], wait=False, speed=2400, acc=50)
            
    except ValueError as e:
        print(f"Error: {e}. Ensure 3 numbers are entered.")
    except Exception as e:
        print(f"Unexpected error: {e}")