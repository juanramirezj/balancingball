from st3215 import ST3215
import time
import random

print('ST3215 Servo Test Script')
print('------------------------')
print('This script will test the ST3215 servo motors by moving them to random positions.')
print('Make sure the servos are connected and powered on.')
print('------------------------')
print('Initializing ST3215 servo controller...')
      

servo = ST3215('/dev/ttyS0')
ids = servo.ListServos()

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


# input values for servo positions 1,2,3 
min_position = 300
max_position = 1000
while True:
    e = input(f"Enter position [servo1,servo2,servo3] ({min_position}-{max_position}) or 'q' to quit: ")
    if e == 'q':
        break
    try:
        position = list(map(int, e.split(',')))
        if len(position) != 3:
            print("Please enter 3 values for servo positions.")
            continue
        if any(p < min_position or p > max_position for p in position):
            print(f"Please enter values between {min_position} and {max_position}.")
            continue
        for j in range(len(ids)):
            servo.MoveTo(ids[j], position[j], wait=False, speed=2400, acc = 50)
    except ValueError:
        print("Invalid input. Please enter 3 integers separated by commas.")
        continue
# input values for servo positions 1,2,3
