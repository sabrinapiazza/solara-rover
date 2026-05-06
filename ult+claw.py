import serial
import time

ULTRASONIC_PORT = "/dev/tty.usbmodem101"  # ultrasonic Pico
CLAW_PORT       = "/dev/tty.usbmodem102"  # claw Pico
BAUD            = 115200

TRIGGER_DISTANCE_M = 0.30  # trigger claw when something is within 30 cm


def parse_line(line: str):
    """Parse 'Sensor1: 231 cm | Sensor2: 0 cm' -> (distance1, distance2) in meters."""
    try:
        parts = line.split('|')
        d1 = float(parts[0].split(':')[1].replace('cm', '').strip()) / 100.0
        d2 = float(parts[1].split(':')[1].replace('cm', '').strip()) / 100.0
        return d1, d2
    except (ValueError, IndexError):
        return None


def send_claw(claw_ser):
    """Send S to claw Pico and wait for DONE."""
    print("  >> Triggering claw...")
    claw_ser.write(b'S')
    while True:
        line = claw_ser.readline().decode(errors="ignore").strip()
        if line:
            print(f"  Claw: {line}")
        if line == "DONE":
            print("  >> Claw sequence complete.")
            break


def main():
    print(f"Connecting to ultrasonic on {ULTRASONIC_PORT}...")
    print(f"Connecting to claw on {CLAW_PORT}...")

    with serial.Serial(ULTRASONIC_PORT, BAUD, timeout=1) as ultrasonic_ser, \
         serial.Serial(CLAW_PORT, BAUD, timeout=1) as claw_ser:

        print("Waiting for claw Pico to be ready...")
        time.sleep(2)  # give claw Pico time to boot

        print(f"Reading ultrasonic (trigger threshold: {TRIGGER_DISTANCE_M*100:.0f} cm)\n")

        claw_busy = False

        while True:
            raw = ultrasonic_ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue

            result = parse_line(raw)
            if result is None:
                print(f"[unparseable] {raw}")
                continue

            d1, d2 = result
            print(f"Sensor1: {d1*100:.1f} cm  |  Sensor2: {d2*100:.1f} cm")

            if not claw_busy and d1 < TRIGGER_DISTANCE_M:
                claw_busy = True
                send_claw(claw_ser)
                claw_busy = False


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")