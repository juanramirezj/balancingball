import serial
import time

def read_serial_xy(port='/dev/ttyACM0', baudrate=115200, timeout=5):
    """
    Reads a line from the specified serial port and returns x, y values as floats.
    Assumes the data is in the format: "x,y\n"
    """
    center_x = 167.0
    center_y = 115.0
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        line = ser.readline().decode('utf-8').strip()
        #print(f"Received line: {line}")  # Debugging output
        if not line:
            raise ValueError("No data received from serial port.")
        try:
            x_str, y_str = line.split()
            x = float(x_str) - center_x
            y = float(y_str) - center_y
            return x, y
        except Exception as e:
            raise ValueError(f"Invalid data received: {line}") from e

# Example usage:
old_x = 0.0
old_y = 0.0
min_x = 640.0
min_y = 480.0
max_x = 0.0
max_y = 0.0

while True:
    x, y = read_serial_xy()
    max_x = max(x,max_x)
    max_y = max(y, max_y)
    min_x = min(min_x,x)
    min_y = min(min_y,y)
        
    if x != old_x or y != old_y:
        print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - x={x}, y={y} min_x={min_x}, min_y={min_y}, max_x={max_x}, max_y={max_y} ")
    old_x, old_y = x, y
    
print("End of program")