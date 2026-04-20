import serial

ser = serial.Serial('/dev/cu.usbmodem101', 115200, timeout=15.0)

print("Waiting for Pico...")
while True:
    line = ser.readline().decode().strip()
    if line == "READY":
        break

print("Pico ready, sending S...")
ser.write(b'S')
ser.flush()

# wait for DONE, ignore any extra READY lines
while True:
    response = ser.readline().decode().strip()
    if response == "DONE":
        print("Pico responded: DONE")
        break
    print(f"ignoring: {response}")

ser.close()