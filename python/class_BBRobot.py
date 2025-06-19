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
        
        self.L = [0.092, 0.098, 0.12, 0.065]  # in meters
        self.L = [65,116,98,92] #in millimeters
        #L = [platform radius, upper link length, lower link length, base radius]
        # Initial posture (theta, phi, pz)
        self.ini_pos = [0, 0, 0.110]
        self.pz_max = 0.220 #0.0732
        self.pz_min = 0.0532
        self.phi_max = 20


    #Method to prepare the robot
    def set_up(self):
        #Turn on servo torque
        servo = self.servo
        ids = self.ids
        position = [420, 420, 420]
        for j in range(len(ids)):
            servo.MoveTo(ids[j], position[j], wait=False, speed=2400, acc = 50)
        
    
    #Method to tidy up robots
    def clean_up(self):
        #Turn off servo torque
        servo = self.servo
        ids = self.ids
        position = [420, 420, 420]
        for j in range(len(ids)):
            servo.MoveTo(ids[j], position[j], wait=False, speed=2400, acc = 50)

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
        print("Parameters for inverse kinematics:   Pz:", Pz, "L1:", L1, "L2:", L2, "R_base:", R_base, "R_platform:", R_platform, "tilt_deg:", tilt_deg)
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


    #Method to calculate inverse kinematics
    def kinema_inv(self, n, Pz):
        print("Inverse kinematics calculation")
        print("Parameters:  n:", n, "Pz:", Pz)
        L = self.L
        print("L=", L)
        # Servo reference height Pmz derive (+- inversion at Pmz)
        A = (L[0]+L[1])/Pz
        B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
        C = A**2+1
        D = 2*(A*B-(L[0]+L[1]))
        E = B**2+(L[0]+L[1])**2-L[2]**2
        print("A:", A, "B:", B, "C:", C, "D:", D, "E:", E              )
        print("determinant:", D**2-4*C*E)
        Pmx = (-D+math.sqrt(D**2-4*C*E))/(2*C)
        Pmz = math.sqrt(L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2)

        #Derive servo a's angle
        a_m_x = (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(n[2])
        a_m_y = 0
        a_m_z = Pz + (L[3]/(math.sqrt(n[0]**2 + n[2]**2)))*(-n[0])
        A_m = [a_m_x, a_m_y, a_m_z]
        A = (L[0]-A_m[0])/A_m[2]
        B = (A_m[0]**2+A_m[1]**2+A_m[2]**2-L[2]**2-L[0]**2+L[1]**2)/(2*A_m[2])
        C = A**2+1
        D = 2*(A*B-L[0])
        E = B**2+L[0]**2-L[1]**2
        ax = (-D+math.sqrt(D**2-4*C*E))/(2*C)
        ay = 0
        az = math.sqrt(L[1]**2-ax**2+2*L[0]*ax-L[0]**2)
        if (a_m_z < Pmz):
            az = -az
        A_2 = [ax, ay, az]
        theta_a = 90 - math.degrees(math.atan2(A_2[0]-L[0], A_2[2]))

        #Derive servo b's angle
        b_m_x = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        b_m_y = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[2])
        b_m_z = Pz + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2+2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[1]+n[0])
        B_m = [b_m_x, b_m_y, b_m_z]

        A = -(B_m[0]+math.sqrt(3)*B_m[1]+2*L[0])/B_m[2]
        B = (B_m[0]**2+B_m[1]**2+B_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*B_m[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        x = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        y = math.sqrt(3)*x
        z = math.sqrt(L[1]**2-4*x**2-4*L[0]*x-L[0]**2)
        if (b_m_z < Pmz):
            z = -z
        B_2 = [x, y, z]
        theta_b = 90 - math.degrees(math.atan2(math.sqrt(B_2[0]**2+B_2[1]**2)-L[0], B_2[2]))

        # Servo c angle calculation
        c_m_x = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-n[2])
        c_m_y = (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(math.sqrt(3)*n[2])
        c_m_z = Pz + (L[3]/(math.sqrt(n[0]**2+3*n[1]**2+4*n[2]**2-2*math.sqrt(3)*n[0]*n[1])))*(-math.sqrt(3)*n[1]+n[0])
        C_m = [c_m_x, c_m_y, c_m_z]

        A = -(C_m[0]-math.sqrt(3)*C_m[1]+2*L[0])/C_m[2]
        B = (C_m[0]**2+C_m[1]**2+C_m[2]**2+L[1]**2-L[2]**2-L[0]**2)/(2*C_m[2])
        C = A**2+4
        D = 2*A*B+4*L[0]
        E = B**2+L[0]**2-L[1]**2
        x = (-D-math.sqrt(D**2-4*C*E))/(2*C)
        y = -math.sqrt(3)*x
        z = math.sqrt(L[1]**2-4*x**2-4*L[0]*x-L[0]**2)
        if (c_m_z < Pmz):
            z = -z
        C_2 = [x, y, z]
        theta_c = 90 - math.degrees(math.atan2(math.sqrt(C_2[0]**2+C_2[1]**2)-L[0], C_2[2]))
        thetas = [theta_a, theta_b, theta_c]
        return thetas

    #Method to achieve posture (theta, phi, Pz) after t seconds
    def control_t_posture(self, pos, t):
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
        print("Posture angles:", pos)
        print("Calculated angles:", angles)
        # self.s1.control_time_rotate(angles[0], t)
        # self.s2.control_time_rotate(angles[1], t)
        # self.s3.control_time_rotate(angles[2], t)
        time.sleep(t)
    
    def Initialize_posture(self):
        # Initial posture (theta, phi, pz)

        t = 1        
        
        pos = [0,0,110]
        self.L = [65,116,98,92]
        #pos = [0, 0, 0.0632]
        #self.L = [0.07, 0.105, 0.120, 0.09]  # in meters
        #40 es la base de los servos
        #40 es la distance del brazo chico
        #65 e la distancia del brazo grande
        # self.L = [0.04, 0.04, 0.065, 0.065]
        self.control_t_posture(pos, t)
        
        # pos = [0,0,-0.210]        
        # self.L = [0.065, 0.12, 0.098, 0.0092]  # in meters
        # self.control_t_posture(pos, t)
        



