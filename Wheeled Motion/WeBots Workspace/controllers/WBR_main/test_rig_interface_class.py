import numpy as np
import serial
import threading
import time

class TestRig:
    def __init__(self, port="COM9", baud=115200):
        # ---- Connection ----
        self.ser = serial.Serial(port, baud, timeout=0.01)

        # Latest streamed data
        self.pos = np.zeros(2)
        self.pitch = 0.0
        self.yaw = 0.0
        self.timestamp = 0.0
        self.timestep = 1

        self.error_flag = False
        self.running = True

        # Start listener thread
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------
    # LISTENER THREAD – continuously receives streaming packets
    # ------------------------------------------------
    def _listen_loop(self):
        buffer = ""

        while self.running:
            try:
                # Read and decode incoming data
                incoming = self.ser.read(128).decode(errors='ignore')
                if not incoming:
                    continue

                buffer += incoming

                if "\n" in buffer:
                    
                    parts = buffer.rsplit("\n", 2)
                    
                    if len(parts) >= 2:
                        # Get latest complete line and update the buffer (trailer)
                        line_to_process = parts[-2]
                        buffer = parts[-1] 
                        
                        self._parse_packet(line_to_process)
                    
            except Exception:
                self.error_flag = True

    def _parse_packet(self, line):
        try:
            parts = line.strip().split(",")
            if len(parts) != 5:
                return

            e_right, e_left, pitch, yaw, ts = parts
            print(f"new pitch is {pitch}")

            self.pos = np.array([float(e_right), float(e_left)])
            self.pitch = float(pitch) + 90
            self.yaw = float(yaw)
            self.timestamp = float(ts)

        except:
            pass  # ignore malformed packets

    # ------------------------------------------------
    # API: SET WHEEL TORQUES
    # ------------------------------------------------
    def set_wheels_torque(self, torques):
        """
        Send torque command to ESP32.
        Expected packet format:
        T,torqueL,torqueR\n
        """
        print(f"sending torque command {torques}")
        tL, tR = torques
        msg = f"T,{tL:.2f},{tR:.2f}\n"
        self.ser.write(msg.encode())

    # ------------------------------------------------
    # API: READ FUNCTIONS (nonblocking)
    # ------------------------------------------------
    def read_pos(self):
        return self.pos.copy()

    def read_imu(self):
        conv = np.pi / 180
        return self.pitch * conv, self.yaw * conv

    # ------------------------------------------------
    # API: TIME + STEP
    # ------------------------------------------------
    def get_time(self):
        """Return real-world time in seconds."""
        return time.time()

    def step(self):
        """Real hardware has no simulation step."""
        pass

    # ------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------
    def close(self):
        self.running = False
        time.sleep(0.05)
        self.ser.close()