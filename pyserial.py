import serial
import sys
import time

PORT = "/dev/tty.usbmodem101"  # change to your Pico's port
BAUD = 115200


def wait_for_ready(ser, timeout=10):
    """Wait until the Pico sends READY."""
    print("Waiting for Pico to be ready...")
    start = time.time()
    while time.time() - start < timeout:
        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"  Pico: {line}")
        if line == "READY":
            return True
    print("Timed out waiting for READY.")
    return False


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT

    print(f"Connecting to {port} at {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=1) as ser:
        if not wait_for_ready(ser):
            return

        print("Sending S...")
        ser.write(b'S')

        # Read responses until DONE
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                continue
            print(f"  Pico: {line}")
            if line == "DONE":
                print("Sequence complete.")
                break


if __name__ == "__main__":
    main()