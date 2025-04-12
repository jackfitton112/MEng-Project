import serial
import time

PORT = '/dev/ttyACM0'
BAUD = 115200
TIMEOUT = 1  # seconds

commands = []
with open('commands.csv', 'r') as f:
    for line in f:
        x, y, z = map(int, line.strip().split(','))
        commands.append((x, y, z))

def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(2)  # Wait for microcontroller reset

    for x, y, z in commands:
        cmd = f"{x},{y},{z}\n"
        print(f"Sending: {cmd.strip()}")
        ser.write(cmd.encode())

        while True:
            response = ser.readline().decode().strip()
            print(f"Response: {response}")
            if response == "OK" or response == "ERROR":
                break

        

        if response != "OK":
            print("ERROR received, stopping.")
            break

        time.sleep(0.05)  # Optional: let MCU process next command

    ser.close()
    print("Finished sending commands.")

if __name__ == '__main__':
    starttime = time.time()
    main()
    print("Elapsed time:", time.time() - starttime)
