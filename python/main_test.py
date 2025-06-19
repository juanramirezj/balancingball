import math
import time


class BBrobot:   #Method to calculate inverse kinematics
    
       #Create robot structure
    def __init__(self):
       
        #range for moving servos
        self.min_position = 400
        self.max_position = 1100      
        
        #Link length L = [bottom, bottom link, top link, ceiling]
        #L = [base servo arm length, middle link lenght, upper link lenght, 
        # platform radius (distance from plantform center to a spherical joint)]
        #self.L = [0.098, 0.120, 0.092, 0.065]  # in meters
        
        self.L = [0.092, 0.098, 0.12, 0.093]  # in meters
        # Initial posture (theta, phi, pz)
        self.ini_pos = [0, 0, 0.110]
        self.pz_max = 0.220 #0.0732
        self.pz_min = 0.0532
        self.phi_max = 20


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
        angles = self.kinema_inv(n, Pz)
        print("Calculated angles:", angles)
        # self.s1.control_time_rotate(angles[0], t)
        # self.s2.control_time_rotate(angles[1], t)
        # self.s3.control_time_rotate(angles[2], t)
        time.sleep(t)
    
    def Initialize_posture(self):
        # Initial posture (theta, phi, pz)

        t = 1        
        
        pos = [0,0,110]
        self.L = [65, 98, 116, 92 ]
        #pos = [0, 0, 0.0632]
        #self.L = [0.07, 0.105, 0.120, 0.09]  # in meters funciona
        #40 es la base de los servos
        #40 es la distance del brazo chico
        #65 e la distancia del brazo grande
        # self.L = [0.04, 0.04, 0.065, 0.065]
        self.control_t_posture(pos, t)
        
        # pos = [0,0,-0.100]        
        #self.L = [0.065, 0.116, 0.098, 0.092]  # in meters
        # self.control_t_posture(pos, t)
  
Robot = BBrobot()

Robot.Initialize_posture()  