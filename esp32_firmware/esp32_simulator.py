import json
import time
import argparse
import os
import serial


DATA_DIR = "data"
LOG_FILE = os.path.join(DATA_DIR, "esp32_log.jsonl")


def get_data(ser):
    """Reads one JSON line from the ESP32. Returns parsed dict or None."""
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if not line:
        return None

    try:
        payload = json.loads(line)
        payload["_rx_unix"] = time.time()
        return payload
    except json.JSONDecodeError:
        print("[skip] Not JSON:", line)
        return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    # Ensure data directory exists
    os.makedirs(DATA_DIR, exist_ok=True)

    ser = serial.Serial(args.port, args.baud, timeout=2.0)
    time.sleep(1.0)

    print(f"[node] Listening on {args.port}")

    while True:
        data = get_data(ser)

        if data is None:
            continue

        print("[data]", data)

        # Append to file
        with open(LOG_FILE, "a") as f:
            f.write(json.dumps(data) + "\n")


if __name__ == "__main__":
    main()