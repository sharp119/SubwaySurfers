import serial
import time
from pynput.keyboard import Key, Controller

# Initialize keyboard controller
keyboard = Controller()

# Replace 'COM3' with your ESP32's COM port. On Linux/macOS, it might be '/dev/ttyUSB0' or similar.
SERIAL_PORT = 'COM5'  # Example for Windows
BAUD_RATE = 9600
TIMEOUT = 1  # seconds

# Define the mapping between touch events and keyboard keys
# You can customize these mappings as needed
KEY_MAPPING = {
    "Left touched": Key.left,
    "Right touched": Key.right,
    "Up touched": Key.up,
    "Down touched": Key.down
}

def main():
    try:
        # Open serial connection
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        time.sleep(2)  # Wait for connection to establish

        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                if line in KEY_MAPPING:
                    print(f"Detected: {line}")
                    key = KEY_MAPPING[line]
                    # Simulate key press and release
                    keyboard.press(key)
                    keyboard.release(key)
                    print(f"Simulated key: {key}")
            time.sleep(0.01)  # Small delay to prevent high CPU usage

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
