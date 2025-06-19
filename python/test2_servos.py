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


position = [
    500,600,700,800,900,1000,1100,1000,900,800,700,600,500]
for i in range(len(position)):
    print(f"Moving servos to position {position[i]}")
    
    for j in range(len(ids)):
        servo.MoveTo(ids[j], position[i], wait=False, speed=2400, acc = 50)
        # wait for a keystroke
    input("Press Enter to continue...")
    
            
    
    
    