import serial
import time
import sys
import signal
import numpy as np # <-- REQUIRED for np.array

# --- Configuration ---
PORT_NAME = "COM9"
BAUD_RATE = 115200
TIMEOUT = 0.1 

# --- Global Flag for Exit ---
running = True

# --- Global State Container (Replaces 'self' variables) ---
# This is where the parsed data will be stored.
data_state = {
    'pos': np.zeros(2),
    'pitch': 0.0,
    'yaw': 0.0,
    'timestamp': 0.0
}
# ---------------------------------------------------------


def parse_packet(line, state_container):
    """
    Parses the incoming serial line and updates the state container.
    Expected format: e_right,e_left,yaw,pitch,ts
    """
    try:
        parts = line.strip().split(",")
        if len(parts) != 5:
            # Optionally print a debug message for malformed packets
            # print(f"Skipping malformed packet: {line}")
            return

        # Unpack and convert values
        e_right, e_left, yaw, pitch, ts = parts

        # Update the state container with the parsed data
        state_container['pos'] = np.array([float(e_right), float(e_left)])
        state_container['pitch'] = float(pitch)
        state_container['yaw'] = float(yaw)
        state_container['timestamp'] = float(ts)
        
        return True # Indicate successful parsing

    except ValueError:
        # Ignore packets where conversion to float fails
        return False
    except Exception as e:
        # Catch any other unexpected errors
        print(f"Parsing error: {e}")
        return False


def signal_handler(sig, frame):
    """Gracefully handles Ctrl+C (SIGINT) to stop the loop."""
    global running
    print("\n--- Interrupted by user (Ctrl+C). Exiting... ---")
    running = False

# Register the signal handler for Ctrl+C
signal.signal(signal.SIGINT, signal_handler)

def simple_serial_monitor():
    """Opens the serial port and continuously prints incoming lines."""
    global running
    ser = None
    
    try:
        ser = serial.Serial(port=PORT_NAME, baudrate=BAUD_RATE, timeout=TIMEOUT)
        print(f"--- Serial Monitor Started ---")
        print(f"Listening on {PORT_NAME} at {BAUD_RATE} bps. Press Ctrl+C to stop.")
        print(f"Current State: {data_state}")
        time.sleep(1) 
        
        while running:
            line = ser.readline()
            
            if line:
                try:
                    message = line.decode('utf-8').strip()
                    
                    # --- CALL THE NEW PARSER FUNCTION ---
                    if parse_packet(message, data_state):
                        # Print the latest state after a successful parse
                        print(f"Parsed: {data_state['pos']}, Pitch: {data_state['pitch']:.3f}, Yaw: {data_state['yaw']:.3f}")
                    else:
                         print(f"Raw: {message}")
                         
                except UnicodeDecodeError:
                    print(f"[{time.strftime('%H:%M:%S')}] Received undecodable bytes.")

            time.sleep(0.001)

    except serial.SerialException as e:
        print(f"\nFATAL ERROR: Could not open port {PORT_NAME}: {e}")
        print("Please check if the port is correct and not in use by another program (or use the correct baud rate).")
        running = False
        
    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    simple_serial_monitor()