import serial

class Camera:
    def __init__(self, port='/dev/ttyACM0'):
        self.center_x = 167.0
        self.center_y = 115.0
        self.port = port
        self.baudrate = 115200
        self.timeout = 5
        
    def find_ball(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=self.timeout) as ser:
                line = ser.readline().decode('utf-8').strip()
                if not line:
                    raise ValueError("No data received from serial port.")
                x_str, y_str = line.split()
                x = float(x_str) - self.center_x
                y = float(y_str) - self.center_y
                return x, y
        except Exception as e:
            raise ValueError(f"Error reading from serial port: {e}") from e
        