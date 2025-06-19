import serial

def main():
    port = '/dev/ttyACM0'
    baudrate = 115200  # Change if your device uses a different baudrate

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            print(f"Listening on {port} at {baudrate} baud...")
            while True:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    print(data.decode(errors='replace'), end='', flush=True)
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\nExiting.")

if __name__ == "__main__":
    main()