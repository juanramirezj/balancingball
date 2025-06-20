import class_PID
import class_BBRobot
import class_OpenMV_cam
import threading
import time
import numpy as np

def cont_rob():
    global x,y 
    while True:
        x,y = camera.find_ball()
        #if x is not None and y is not None:
        #    print(f"Ball coordinates: x={x}, y={y}")


#Coefficients that determine the PID coefficients and the magnitude of phi
print("Initializing robot control...")
K_PID = [0.015, 0.0001, 0.0051] #0.015, 0.0001, 0.0051
k = 1
a = 1
#Instantiate the robot, camera and PID rule
print("Creating robot, camera and PID instances...")
Robot = class_BBRobot.BBrobot()
camera = class_OpenMV_cam.Camera()
pid = class_PID.PID(K_PID, k, a)

#ball coordinates
x = -1
y = -1
area = -1
goal = [0, 0]

print("Setting up robot and initializing posture...")
Robot.set_up()
Robot.Initialize_posture()

pz_ini = Robot.ini_pos[2]

try:
    rob_thread = threading.Thread(target=cont_rob)
    rob_thread.start()

    print("Starting main control loop...")
    while True:
        Current_value = [x, y]
        if x!= -1:
            theta, phi = pid.compute(goal, Current_value)
            pos = [theta, phi, pz_ini]
            Robot.control_t_posture(pos, 0.01)
            #print(f"Current position: {Current_value}, Target position: {goal} Theta: {theta}, Phi: {phi} Robot position: {pos} ")
        
finally:
    print("Stopping robot and cleaning up...")