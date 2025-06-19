#code for computing angles test
import math
import sys

def kinema_inv(n, Pz, L):
    
    A = (L[0]+L[1])/Pz
    B = (Pz**2+L[2]**2-(L[0]+L[1])**2-L[3]**2)/(2*Pz)
    C = A**2+1
    D = 2*(A*B-(L[0]+L[1]))
    E = B**2+(L[0]+L[1])**2-L[2]**2
    Pmx = (-D+math.sqrt(D**2-4*C*E))/(2*C)
    Pmz = math.sqrt(L[2]**2-Pmx**2+2*(L[0]+L[1])*Pmx-(L[0]+L[1])**2)

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
    
    return [theta_a, theta_b, theta_c]

class DeltaRobotKinematics:
    def __init__(self):
        # Joint limits in degrees
        self.THETA_MIN = -130
        self.THETA_MAX = 180
        
        # Platform limits
        self.MIN_HEIGHT = 0.03
        self.MAX_HEIGHT = 0.75
        self.MAX_TILT = 60
        
        self.L = None

    def calculate_normal_from_xyz(self, x, y, z, platform_height):
        rel_x = x
        rel_y = y
        rel_z = z - platform_height
        
        magnitude = math.sqrt(rel_x**2 + rel_y**2 + rel_z**2)
        if magnitude == 0:
            return [0, 0, 1]
            
        tilt_angle = math.degrees(math.acos(rel_z/magnitude))
        if tilt_angle > self.MAX_TILT:
            raise ValueError(f"Required tilt angle {tilt_angle:.1f}° exceeds maximum {self.MAX_TILT}°")
            
        return [rel_x/magnitude, rel_y/magnitude, rel_z/magnitude]

    def get_user_input_links(self):
        try:
            print("\nEnter link lengths in meters:")
            base_radius = float(input("Base radius (typical 0.04m): "))
            arm1_length = float(input("First arm length (typical 0.04m): "))
            arm2_length = float(input("Second arm length (typical 0.065m): "))
            platform_radius = float(input("Platform radius (typical 0.065m): "))
            
            if any(l <= 0 for l in [base_radius, arm1_length, arm2_length, platform_radius]):
                raise ValueError("All lengths must be positive!")
            
            self.L = [base_radius, arm1_length, arm2_length, platform_radius]
            return True
            
        except ValueError as e:
            print(f"Error: {e}")
            return False

    def get_target_position(self):
        try:
            print("\nEnter target position (in meters):")
            x = float(input("X coordinate: "))
            y = float(input("Y coordinate: "))
            z = float(input("Z coordinate: "))
            return x, y, z
        except ValueError as e:
            print(f"Error: Invalid input - {e}")
            return None

    def calculate_angles(self, x, y, z):
        try:
            platform_height = z/2
            
            if platform_height < self.MIN_HEIGHT or platform_height > self.MAX_HEIGHT:
                raise ValueError(f"Platform height {platform_height}m is outside valid range [{self.MIN_HEIGHT}, {self.MAX_HEIGHT}]m")
            
            n = self.calculate_normal_from_xyz(x, y, z, platform_height)
            
            angles = kinema_inv(n, platform_height, self.L)
            
            for i, angle in enumerate(['A', 'B', 'C']):
                if not self.THETA_MIN <= angles[i] <= self.THETA_MAX:
                    raise ValueError(f"Servo {angle} angle {angles[i]:.1f}° exceeds limits [{self.THETA_MIN}, {self.THETA_MAX}]°")

            return angles

        except ValueError as e:
            raise ValueError(f"Calculation failed: {e}")

def main():
    robot = DeltaRobotKinematics()
    
    print("Delta Robot Inverse Kinematics Calculator")
    while not robot.get_user_input_links():
        print("Please try again.")
    
    while True:
        result = robot.get_target_position()
        if not result:
            continue
        x, y, z = result
        
        try:
            angles = robot.calculate_angles(x, y, z)
            
            print("\nResults:")
            print(f"Target position: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
            print(f"Platform height: {z/2:.3f}m")
            print(f"Servo angles:")
            print(f"  Servo A: {angles[0]:.1f}°")
            print(f"  Servo B: {angles[1]:.1f}°")
            print(f"  Servo C: {angles[2]:.1f}°")
            
        except ValueError as e:
            print(f"\nError: {e}")
        
        if input("\nCalculate another position? (y/n): ").lower() != 'y':
            break
    
    print("Thank you for using the Delta Robot Calculator!")

if __name__ == "__main__":
    main()