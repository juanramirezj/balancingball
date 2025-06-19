import class_PID
import class_BBRobot
import class_OpenMV_cam
import threading
import time
import numpy as np

#Coefficients that determine the PID coefficients and the magnitude of phi
K_PID = [0.015, 0.0001, 0.0051] #0.015, 0.0001, 0.0051
k = 1
a = 1
#Instantiate the robot, camera and PID rule
Robot = class_BBRobot.BBrobot()
camera = class_OpenMV_cam.Camera()
pid = class_PID.PID(K_PID, k, a)

Robot.set_up()
Robot.Initialize_posture()
