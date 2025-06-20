import math
import time
import numpy as np
from st3215 import ST3215

class BBrobot:
    #Create robot structure
    def __init__(self):
        self.servo = ST3215('/dev/ttyS0')
        servo = self.servo
        self.ids = self.servo.ListServos()
        ids = self.ids
        
        if len(ids) == 0:
            print('No servos found during initialization!')
            exit()       

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
            
         
        #range for moving servos
        self.min_position = 400
        self.max_position = 1100      
        
        #Link length L = [bottom, bottom link, top link, ceiling]
        #L = [base servo arm length, middle link lenght, upper link lenght, 
        # platform radius (distance from plantform center to a spherical joint)]
        #self.L = [0.098, 0.120, 0.092, 0.065]  # in meters
        
        self.L = [65,116,98,92] #in millimeters
        #L = [platform radius, upper link length, lower link length, base radius]
        # Initial posture (theta, phi, pz)
        self.ini_pos = [0, 0, 110]
        self.pz_max = 220 #0.0732
        self.pz_min = 532
        self.phi_max = 0.8 # 1rad = 57°


    def angulo_to_position(self,angulo):
        valor = int(angulo * 1100 / 90) 
        if valor < 400:
            valor = 400
        elif valor > 1100:
            valor = 1100
        
        return valor
    
    def position_to_angulo(self,position):
        valor = int(position * 90 / 1100)
        if valor < 0:
            valor = 0
        elif valor > 90:
            valor = 90
        
        return valor

    #Method to prepare the robot
    def set_up(self):
        #Turn on servo torque
        servo = self.servo
        ids = self.ids
        position = [45, 45, 45] #degrees
        for j in range(len(ids)):
            servo.MoveTo(ids[j], self.angulo_to_position(position[j]), wait=False, speed=2400, acc = 50)
        
    
    #Method to tidy up robots
    def clean_up(self):
        #Turn off servo torque
        servo = self.servo
        ids = self.ids
        position = [45, 45, 45] #degrees
        for j in range(len(ids)):
            servo.MoveTo(ids[j], self.angulo_to_position(position[j]), wait=False, speed=2400, acc = 50)

    def deg(self,x):
        return math.degrees(x)

    #New method for inverse kinematics
    def rrs_inverse_kinematics(self,Pz, L1, L2, R_base, R_platform, tilt_deg=(0, 0)):
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
        # print("Parameters for inverse kinematics:   Pz:", Pz, "L1:", L1, "L2:", L2, "R_base:", R_base, "R_platform:", R_platform, "tilt_deg:", tilt_deg)
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

            servo_angles.append(180-self.deg(servo_angle))

        return servo_angles


   
    #Method to achieve posture (theta, phi, Pz) after t seconds
    def control_t_posture(self, pos, t):
        ids = self.ids
        servo = self.servo
        
        theta = pos[0]
        phi = pos[1]
        #Operation constraints
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        """
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min 
        """
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]
        #angles = self.kinema_inv(n, Pz)
        #L1 = lower link
        #L2 = upper link
        #def rrs_inverse_kinematics(self,Pz, L1, L2, R_base, R_platform, tilt_deg=(0, 0)):
        angles = self.rrs_inverse_kinematics(Pz=Pz, L1=self.L[2], L2=self.L[1], R_base=self.L[3], R_platform=self.L[0], tilt_deg= (theta, phi))
        # print("Posture angles:", pos)
        # print("Calculated angles:", angles)
        for j in range(len(ids)):
            print(f"Moving servo #{j} ID: {ids[j]} to angle: {angles[j]}", end ="" )
            servo.MoveTo(ids[j], self.angulo_to_position(angles[j]), wait=False, speed=2400, acc = 50)
            time.sleep(t)
        print(f"\nServo 1= {self.position_to_angulo(servo.ReadPosition(ids[0]))}, Servo 2={self.position_to_angulo(servo.ReadPosition(ids[1]))}, Servo 3={self.position_to_angulo(servo.ReadPosition(ids[2]))}")
        
        time.sleep(t)
    
    def Initialize_posture(self):
        # Initial posture (theta, phi, pz)

        t = 1        
        
        # pos = [0,0,110]
        # self.L = [65,116,98,92]
        #pos = [0, 0, 0.0632]
        #self.L = [0.07, 0.105, 0.120, 0.09]  # in meters
        #40 es la base de los servos
        #40 es la distance del brazo chico
        #65 e la distancia del brazo grande
        # self.L = [0.04, 0.04, 0.065, 0.065]
        pos = self.ini_pos
        self.control_t_posture(pos, t)
        
        # pos = [0,0,-0.210]        
        # self.L = [0.065, 0.12, 0.098, 0.0092]  # in meters
        # self.control_t_posture(pos, t)
        



