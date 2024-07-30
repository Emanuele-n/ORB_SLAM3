import serial
import time

# Setup serial connection (adjust the port name to match your system)
ser = serial.Serial("/dev/ttyACM2", 9600)  # Example for Linux
# ser = serial.Serial("COM3", 9600)  # Example for Windows


def send_command(command):
    """Send a single character command to the Arduino."""
    ser.write(command.encode())  # Encode the string to bytes
    print(f"Sent '{command}' to Arduino")
    time.sleep(0.1)  # Give some time for Arduino to process the command


def main():
    try:
        while True:
            # User inputs the command
            command = input(
                "Enter command ('f' for forward, 'b' for backward, 's' for stop): "
            )
            if command in ["f", "b", "s"]:
                send_command(command)
            else:
                print("Invalid command. Please enter only 'f', 'b', or 's'.")
    except KeyboardInterrupt:
        print("Program exited by user.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
