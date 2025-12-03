import numpy as np
import serial
import threading
import time

class RealRobotInterface:
    def __init__(self, port="/dev/ttyUSB0", baud=460800):
        # ---- Connection ----
        self.ser = serial.Serial(port, baud, timeout=0.01)

        # Latest streamed data
        self.pos = np.zeros(2)
        self.pitch = 0.0
        self.yaw = 0.0
        self.timestamp = 0.0

        self.error_flag = False
        self.running = True

        # Start listener thread
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------
    # LISTENER THREAD – continuously receives streaming packets
    # ------------------------------------------------
    def _listen_loop(self):
        """
        Expected ESP32 packet format (one line):
        ENC1,ENC2,PITCH,YAW,TIME\n
        Example:
        1234,1240,-0.03,0.14,12.712
        """
        buffer = ""

        while self.running:
            try:
                incoming = self.ser.read(128).decode(errors='ignore')
                if not incoming:
                    continue

                buffer += incoming

                # process full lines
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self._parse_packet(line)

            except Exception as e:
                print("Serial error:", e)
                self.error_flag = True

    def _parse_packet(self, line):
        try:
            parts = line.strip().split(",")
            if len(parts) != 5:
                return

            e1, e2, pitch, yaw, ts = parts

            self.pos = np.array([float(e1), float(e2)])
            self.pitch = float(pitch)
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
        tL, tR = torques
        msg = f"T,{tL:.5f},{tR:.5f}\n"
        self.ser.write(msg.encode())

    # ------------------------------------------------
    # API: READ FUNCTIONS (nonblocking)
    # ------------------------------------------------
    def read_pos(self):
        return self.pos.copy()

    def read_imu(self):
        return self.pitch, self.yaw

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