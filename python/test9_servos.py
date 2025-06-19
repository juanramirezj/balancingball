900from st3215 import ST3215
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
min_position = 45
max_position = 90
compensation12 = 0
compensation13 = 0

def degrees_to_pos(degrees):
    """Convert degrees to servo position."""
    v = degrees * 1100/90
    if v < 500:
        v = 500
    elif v > 1100:
        v = 1100
    return int(v)  # Assuming 90 degrees is at 1100 position

while True:
    e = input(f"Enter position servos IN DEGREES ({min_position}-{max_position}) or 'q' to quit: ")
    if e == 'q':
        break

    e = int(e)
    if e < min_position or e > max_position:
        print(f"Please enter values between {min_position} and {max_position}.")
        continue

    position = [0, 0, 0]
    position[0] = e 
    position[1] = e + compensation12
    position[2] = e + compensation13
    for j in range(len(ids)):
        servo.MoveTo(ids[j], degrees_to_pos(position[j]), wait=False, speed=2400, acc = 50)
