import serial
import threading

class Camera:
    def __init__(self, port='/dev/ttyACM0', baudrate=115200):
        self.center_x = 164.0
        self.center_y = 114.0

        self.port = port
        self.baudrate = baudrate

        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=0,          # non-blocking
            write_timeout=0
        )

        self.lock = threading.Lock()

        self.x = -1.0
        self.y = -1.0

        self.running = True

        # Start background reader thread
        self.thread = threading.Thread(target=self._reader)
        self.thread.daemon = True
        self.thread.start()

    # ==========================================
    # Background serial reader (FAST)
    # ==========================================
    def _reader(self):
        buffer = ""

        while self.running:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
                if not data:
                    continue

                buffer += data.decode('utf-8', errors='ignore')

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()

                    if "," in line:
                        try:
                            xs, ys = line.split(",")
                            x = float(xs) - self.center_x
                            y = float(ys) - self.center_y

                            with self.lock:
                                #self.x = x
                                #self.y = y
                                #Low pass filter
                                alpha = 0.6
                                self.x = alpha * x + (1 - alpha) * self.x
                                self.y = alpha * y + (1 - alpha) * self.y
                        except:
                            pass
            except:
                pass

    # ==========================================
    # Get latest ball position (NON-BLOCKING)
    # ==========================================
    def find_ball(self):
        with self.lock:
            return self.x, self.y

    # ==========================================
    # Clean shutdown
    # ==========================================
    def close(self):
        self.running = False
        self.thread.join()
        self.ser.close()