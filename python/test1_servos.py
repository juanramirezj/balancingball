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


for i in range(1,10):
    new_position = random.randint(0, 2399)
    print(f"{i} - Moving servos to position {new_position}")
    
    for j in range(len(ids)):
        servo.MoveTo(ids[j], new_position, wait=False, speed=2400, acc = 50)
    time.sleep(1)