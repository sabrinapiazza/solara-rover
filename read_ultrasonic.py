import serial
import sys
 
PORT = "/dev/tty.usbmodem101"  # change to your Pico's port
BAUD = 115200
 
def parse_line(line: str):
    """Parse 'Sensor1: 231 cm | Sensor2: 0 cm' -> (distance1, distance2) in meters."""
    try:
        parts = line.split('|')
        distance1_cm = float(parts[0].split(':')[1].replace('cm', '').strip())
        distance2_cm = float(parts[1].split(':')[1].replace('cm', '').strip())
        return distance1_cm / 100.0, distance2_cm / 100.0
    except (ValueError, IndexError):
        return None
 
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
 
    print(f"Connecting to {port} at {BAUD} baud...")
    with serial.Serial(port, BAUD, timeout=1) as ser:
        print("Reading (Ctrl+C to stop)\n")
        while True:
            raw = ser.readline().decode(errors="ignore").strip()
            if not raw:
                continue
 
            result = parse_line(raw)
            if result is not None:
                d1, d2 = result
                print(f"Sensor1: {d1:.3f} m ({d1*100:.1f} cm)  |  Sensor2: {d2:.3f} m ({d2*100:.1f} cm)")
            else:
                print(f"[unparseable] {raw}")
 
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")