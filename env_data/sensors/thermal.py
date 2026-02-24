# MLX90640 thermal camera driver
# Reads temperature array (32x24 pixels)
# Returns thermal image data

import time
import board
import busio
import adafruit_mlx90640

def get_data():
    # Single read of MLX90640 frame. Returns min/max/avg stats as a dict for collector.py.
    i2c = busio.I2C(board.SCL, board.SDA, frequency=8e5)
    mlx = adafruit_mlx90640.MLX90640(i2c)
    mlx.refresh_rate = adafruit_mlx90640.RefreshRate.REFRESH_8_Hz

    frame = [0] * 768  # 32x24 = 768 pixels
    mlx.getFrame(frame)

    return {
        "min_c": round(min(frame), 2),              # coldest pixel in degrees C
        "max_c": round(max(frame), 2),              # hottest pixel in degrees C
        "avg_c": round(sum(frame) / len(frame), 2), # average across all pixels
    }

def main():
    # Standalone loop for testing thermal.py directly.
    print("Thermal camera initialized. Reading data...")
    try:
        while True:
            data = get_data()
            print(
                f"Min: {data['min_c']} °C, "
                f"Max: {data['max_c']} °C, "
                f"Avg: {data['avg_c']} °C"
            )
            time.sleep(1)
    except KeyboardInterrupt:
        print("Thermal camera stopped.")

if __name__ == "__main__":
    main()