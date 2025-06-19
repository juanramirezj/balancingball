#program to control three servos of a 3RRS parallel manipulator
from st3215 import ST3215
import numpy as np
import time

class BBrobot:
    
    #Create robot structure
    def __init__(self):
        self.servo = ST3215('/dev/ttyS0')
        self.ids = self.servo.ListServos()
        if len(self.ids) == 0:
            print('No servos found during initialization!')
            exit()
        
        # Link lengths
        self.l1 = 98  # mm
        self.l2 = 120  # mm
        self.base_radius = 92  # mm
            
    def print_status(self):
        ids = self.ids
        servo = self.servo
        
        if len(ids) == 0:
            print('No servos found')
            exit()
        else:
            print('Found servos:', ids)

        print('Servo IDs:', ids)
        print('Servo Count:', len(ids))
        for i in range(len(ids)):
            print('Servo ID:', ids[i])
            print('Servo Position:', servo.ReadPosition(ids[i]))
            print('Servo Temperature:', servo.ReadTemperature(ids[i]))
            print('Servo Voltage:', servo.ReadVoltage(ids[i]))
            print('Servo Current:', servo.ReadCurrent(ids[i]))
            print('Servo Speed:', servo.ReadSpeed(ids[i]))
            print('Servo Load:', servo.ReadLoad(ids[i]))
            print('Servo Status:', servo.ReadStatus(ids[i]))
            print('----------------')
            
    def angle_to_pos(self, angle):
        """Convert degrees to servo position."""
        v = angle * 1100 / 90
        if v < 500:
            v = 500
        elif v > 1100:
            v = 1100
        return int(v)
    
    def control_rotate(self, angle):
        ids = self.ids
        servo = self.servo
        for j in range(len(ids)):
            servo.MoveTo(ids[j], self.angle_to_pos(angle[j]), wait=False, speed=2400, acc = 50)
            
    def rotation_z(angle):
        #Rotation matrix around Z-axis.
        c, s = np.cos(angle), np.sin(angle)
        return np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])
        
    def inverse_kinematics_3rrs(self, platform_pos ):

        # Computes servo angles for a 3RRS manipulator.
        
        # platform_pos: [x, y, z] position of the platform center
        # l1: length of first link (from base to elbow)
        # l2: length of second link (elbow to platform)
        # Returns angles for each of the 3 servos in radians.


        # Base joint positions (equilateral triangle)
        l1 = self.l1
        l2 = self.l2
        base_radius = self.base_radius
        
        base_radius = 92  # mm
        B = np.array([
            [base_radius, 0, 0],
            [base_radius * np.cos(2*np.pi/3), base_radius * np.sin(2*np.pi/3), 0],
            [base_radius * np.cos(4*np.pi/3), base_radius * np.sin(4*np.pi/3), 0],
        ])

        # Platform joint offsets (same triangle, smaller radius)
        platform_radius = 65  # mm
        P_offsets = np.array([
            [platform_radius, 0, 0],
            [platform_radius * np.cos(2*np.pi/3), platform_radius * np.sin(2*np.pi/3), 0],
            [platform_radius * np.cos(4*np.pi/3), platform_radius * np.sin(4*np.pi/3), 0],
        ])

        # Target joint positions on platform in world space
        P_world = P_offsets + platform_pos

        angles = []
        for i in range(3):
            vec = P_world[i] - B[i]
            dist = np.linalg.norm(vec)
            
            # Check if the position is reachable
            if dist > l1 + l2 or dist < abs(l1 - l2):
                raise ValueError(f"Leg {i+1}: Position unreachable.")

            # Law of cosines to find angle at base servo
            # Assume motion in plane formed by vector and base
            # Project to 2D for simplicity
            dx, dz = np.linalg.norm(vec[:2]), vec[2]
            d = np.sqrt(dx**2 + dz**2)
            
            angle = np.arccos((l1**2 + d**2 - l2**2) / (2 * l1 * d))  # angle between l1 and d
            base_angle = np.arctan2(dz, dx)  # angle of vector in plane
            
            total_angle = base_angle - angle
            angles.append(total_angle)
            
            # convert to degrees
            angles[i] = np.degrees(angles[i])

        return angles    
            
    def test_inverse_kinematics(self, platform_height):
        platform_position = np.array([0,0,0])              
        platform_position[2] = platform_height # mm      
        angles = self.inverse_kinematics_3rrs(platform_position)
        print(f"Testing inverse kinematics for platform height {platform_height} mm:")
        for i, angle in enumerate(angles, start=1):
            print(f"Servo {i} angle: {angle} °")
        self.control_rotate(angles)
   
# Main program
Robot = BBrobot()
Robot.print_status()
a = [60,60,60]  # Example angles for servos
Robot.control_rotate(a)
Robot.test_inverse_kinematics(200)  # Example height for testing inverse kinematics