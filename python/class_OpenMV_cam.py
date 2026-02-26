import serial

class Camera:
    def __init__(self, port='/dev/ttyACM0'):
        self.center_x = 0 # 167.0
        self.center_y = 0 # 115.0
        self.port = port
        self.baudrate = 115200
        self.timeout = 5
        
    def find_ball(self):
        x = -1
        y = -1
        try:
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    print("No data received from serial port.")
                else:
                    x_str, y_str = line.split(",")
                    x = float(x_str) - self.center_x
                    y = float(y_str) - self.center_y
        except:
            print(f"Error reading from serial port line: {line}")
        return x,y
        