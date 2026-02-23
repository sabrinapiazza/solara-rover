import json
import time
import argparse
import requests
import serial

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--url", required=True)
    args = p.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=2.0)
    time.sleep(1.0)

    print(f"[node] {args.port} -> {args.url}")

    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            print("[serial]", line)
            continue

        payload["_rx_unix"] = time.time()

        try:
            r = requests.post(args.url, json=payload, timeout=2.0)
            print("[ok]", r.status_code, payload)
        except Exception as e:
            print("[err]", e, payload)

if __name__ == "__main__":
    main()