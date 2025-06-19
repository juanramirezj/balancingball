from st3215 import ST3215
import time
import random
import math




# input values for servo positions 1,2,3 
min_position = 400
max_position = 1200
compensation12 = 110
compensation13 = 0



class BBrobot:
    #3 RRS Parallel manipulator
    def __init__(self):               
        # self.servo = ST3215('/dev/ttyS0')
        # self.ids = self.servo.ListServos()
        # if len(self.ids) == 0:
        #     print('No servos found')
        #  else:
        #     print('Found servos:', self.ids)
        
        #Link length L = [bottom, lower link, upper link, ceiling]
        # bottom = Base radius
        # Lower link = first arm length
        # Upper link = second arm length
        # ceiling = platform radius
        # values are in meters
        #self.L = [0.092, 0.098,  0.120, 0.180]
        self.L = [0.092, 0.098,  0.120, 0.065]
        
        #Initial posture (theta, phi, pz)
        self.ini_pos = [0, 0, 0.185] # theta, phi, pz
        #pz is the distance between the end effector and the ceiling
        #theta represents the azimuthal angle (rotation around the z-axis)
        #phi is the elevation angle (tilt from the vertical axis)
        self.pz_max = 10.0
        self.pz_min = -10.0
        self.phi_max = 180
            
    def list_servos(self):
        servo = self.servo
        ids = self.ids
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
    
    def kinema_inv(self, n, Pz):
        # the 3 RRS parallel manipulator is a 3 DOF manipulator
        # has 3 servos, the servo is connected to the end effector, and the end effector is connected to the ceiling
        # n is the normal vector of the ceiling
        # Pz is the vertical distance to the ceiling
        # n = [nx, ny, nz] 
        L = self.L
        # Servo reference height Pmz derive (+- inversion at Pmz)
        A = (L[0]+L[1])/Pz
        B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
        C = A**2+1
        D = 2*(A*B-(L[0]+L[1]))
        E = B**2+(L[0]+L[1])**2-L[2]**2
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
    #theta is the angle between the end effector and the ceiling
    #phi is the angle between the end effector and the bottom link
    #Pz is the distance between the end effector and the ceiling
    #t is the time to achieve the posture
    #pos = [theta, phi, Pz]
    def control_t_posture(self, pos, t):
        theta = pos[0]
        phi = pos[1]
        
        #Operation constraints
        if phi > self.phi_max:
            phi = self.phi_max
        Pz = pos[2]
        if Pz > self.pz_max:
            Pz = self.pz_max
        elif Pz < self.pz_min:
            Pz = self.pz_min 
        z = math.cos(math.radians(phi))
        r = math.sin(math.radians(phi))
        x = r*math.cos(math.radians(theta))
        y = r*math.sin(math.radians(theta))
        n = [x, y, z]
        angles = self.kinema_inv(n, Pz)
        
        #self.s1.control_time_rotate(angles[0], t)
        #self.s2.control_time_rotate(angles[1], t)
        #self.s3.control_time_rotate(angles[2], t)
        #time.sleep(t)
        print(f"pos: {pos}, theta: {theta}, phi: {phi}, Pz: {Pz}")        
        print(f"Servo angles: {angles}")

    def Initialize_posture(self):
        pos = self.ini_pos
        t = 1
        self.control_t_posture(pos, t)
        
robot = BBrobot()
# robot.list_servos()
robot.Initialize_posture()
print( robot.kinema_inv([0, 0, 1], 0.180) )



