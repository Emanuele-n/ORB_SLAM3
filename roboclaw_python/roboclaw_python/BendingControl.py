import json
import socket
import threading
import time
from roboclaw_3 import Roboclaw

Couple12 = Roboclaw("/dev/ttyACM0", 115200)
Couple34 = Roboclaw("/dev/ttyACM1", 115200)

"""
motor_1 = Couple12.M1
motor_2 = Couple12.M2
motor_3 = Couple34.M1
motor_4 = Couple34.M2

up = motor_1
down = motor_3
right = motor_2
left = motor_4

down-up range
motor_3      -   motor_1
[-3000000,0) U [0,3000000]

left-right range
motor_4      -   motor_2
[-4000000,0) U [0,4000000]
4000000      - 0 - 4000000
"""


import tty
import sys
import termios
import time
from roboclaw_3 import Roboclaw
import threading


class BendingControl:
    def __init__(self, couple12_port, couple34_port, baud_rate=115200):
        self.couple12 = Roboclaw(couple12_port, baud_rate)
        self.couple34 = Roboclaw(couple34_port, baud_rate)
        self.couple12.Open()
        self.couple34.Open()
        self.address = 0x80
        self.increment = 1000000  # Increment for each key press
        self.orig_settings = termios.tcgetattr(sys.stdin)
        self.horizontal_state = 0
        self.vertical_state = 0
        self.horizontal_limit = [-4000000, 4000000]
        self.vertical_limit = [-3000000, 3000000]

        # Set intial position to zero
        self.go_to_zero()

        # Update states thread
        update_thread = threading.Thread(target=self.listen_to_encoders, daemon=True)
        update_thread.start()

        # Print thread
        print_thread = threading.Thread(target=self.print_current_position, daemon=True)
        print_thread.start()

        # # Send data thread
        # send_thread = threading.Thread(target=self.sendData, daemon=True)
        # send_thread.start()

        print("Bending control initialized")

    def control_position(self, direction):
        print(f"Moving {direction}")
        # TODO: revise the logic for the following cases
        if direction == "up":
            if self.vertical_state >= 0:
                new_position = self.vertical_state + self.increment
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            elif -self.increment < self.vertical_state < 0:
                # Move motor 3 to zero
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address, 320000, 500000, 320000, 0, 0
                )
                time.sleep(0.001)
                # Move motor 1 to new position difference between increment and vertical state, which is negative
                new_position = self.increment - abs(self.vertical_state)
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            else:
                new_position = abs(self.vertical_state) - self.increment
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            time.sleep(0.001)
        elif direction == "down":
            if self.vertical_state <= 0:
                new_position = abs(self.vertical_state) + self.increment
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            elif 0 < self.vertical_state < self.increment:
                # Move motor 1 to zero
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address, 320000, 500000, 320000, 0, 0
                )
                time.sleep(0.001)
                # Move motor 3 to new position difference between increment and vertical state, which is positive
                new_position = self.increment - self.vertical_state
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address, 320000, 500000, 320000, new_position, 0
                )
            else:
                new_position = self.vertical_state - self.increment
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )

            time.sleep(0.001)
        elif direction == "right":
            if self.horizontal_state >= 0:
                new_position = self.horizontal_state + self.increment
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            elif -self.increment < self.horizontal_state < 0:
                # Move motor 4 to zero
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address, 320000, 500000, 320000, 0, 0
                )
                time.sleep(0.001)
                # Move motor 2 to new position difference between increment and vertical state, which is negative
                new_position = abs(self.increment - self.horizontal_state)
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            else:
                new_position = abs(self.horizontal_state) - self.increment
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            time.sleep(0.001)
        elif direction == "left":
            if self.horizontal_state <= 0:
                new_position = abs(self.horizontal_state) + self.increment
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )
            elif 0 < self.horizontal_state < self.increment:
                # Move motor 2 to zero
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address, 320000, 500000, 320000, 0, 0
                )
                time.sleep(0.001)
                # Move motor 4 to new position difference between increment and vertical state, which is positive
                new_position = self.increment - self.horizontal_state
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address, 320000, 500000, 320000, new_position, 0
                )
            else:
                new_position = self.horizontal_state - self.increment
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    320000,
                    500000,
                    320000,
                    new_position,
                    0,
                )

            time.sleep(0.001)

        else:
            print("Invalid direction")

    def go_to_zero(self):
        print("Moving to zero position")
        # Position all motors to zero position
        self.couple12.SpeedAccelDeccelPositionM1(
            self.address, 320000, 500000, 320000, 0, 0
        )
        time.sleep(0.001)
        self.couple12.SpeedAccelDeccelPositionM2(
            self.address, 320000, 500000, 320000, 0, 0
        )
        time.sleep(0.001)
        self.couple34.SpeedAccelDeccelPositionM1(
            self.address, 320000, 500000, 320000, 0, 0
        )
        time.sleep(0.001)
        self.couple34.SpeedAccelDeccelPositionM2(
            self.address, 320000, 500000, 320000, 0, 0
        )
        time.sleep(0.001)

    def listen_to_encoders(self):
        while True:
            self.update_states()
            time.sleep(0.01)

    def on_close(self):
        print("Closing motors")
        self.go_to_zero()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_settings)
        print("Terminal settings restored.")

    def print_current_position(self):
        print(
            "Current position: LR: {}, DU: {}".format(
                self.horizontal_state, self.vertical_state
            )
        )
        time.sleep(0.1)

    def read_current_position(self):
        enc1 = self.couple12.ReadEncM1(self.address)
        enc2 = self.couple12.ReadEncM2(self.address)
        enc3 = self.couple34.ReadEncM1(self.address)
        enc4 = self.couple34.ReadEncM2(self.address)
        # print("Encoder1:"),
        # if enc1[0] == 1:
        #     print(enc1[1])
        # else:
        #     print("failed")
        # print("Encoder2:", end=" ")
        # if enc2[0] == 1:
        #     print(enc2[1])
        # else:
        #     print("failed ")
        # print("Encoder3:", end=" ")
        # if enc3[0] == 1:
        #     print(enc3[1])
        # else:
        #     print("failed ")
        # print("Encoder4:"),
        # if enc4[0] == 1:
        #     print(enc4[1])
        # else:
        #     print("failed ")
        time.sleep(0.001)

        return enc1[1], enc2[1], enc3[1], enc4[1]

    def run(self):
        tty.setcbreak(sys.stdin)
        try:
            print("Use arrow keys to bend. Press 'q' to quit.")
            while True:
                key = sys.stdin.read(1)
                if key == "\x1b":
                    next1, next2 = sys.stdin.read(2)
                    if next1 == "[":
                        if next2 == "A":  # Up arrow
                            self.control_position("up")
                        elif next2 == "B":  # Down arrow
                            self.control_position("down")
                        elif next2 == "C":  # Right arrow
                            self.control_position("right")
                        elif next2 == "D":  # Left arrow
                            self.control_position("left")
                elif key == "q":
                    print("Exiting program.")
                    # Bring back motors to zero
                    self.on_close()
                    break
                time.sleep(0.1)
        finally:
            self.on_close()

    def sendData(self):
        print("Trying to connect")
        host = "127.0.0.1"  # The server's hostname or IP address
        port = 65432  # The port used by the server

        while True:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((host, port))
                    print("Connection established")
                    while True:
                        enc1, enc2, enc3, enc4 = self.read_current_position()
                        self.horizontal_state = enc2 - enc4
                        self.vertical_state = enc1 - enc3
                        data = {"x": self.horizontal_state, "y": self.vertical_state}
                        json_data = (
                            json.dumps(data) + "\n"
                        )  # Append a newline to each JSON message
                        s.sendall(json_data.encode("utf-8"))
                        time.sleep(
                            0.1
                        )  # Throttle the data sending to prevent overwhelming the receiver
            except Exception as e:
                print(f"Connection failed: {e}")
                print("Retrying to connect...")
                time.sleep(1)

    def update_states(self):
        enc1, enc2, enc3, enc4 = self.read_current_position()
        self.horizontal_state = enc2 - enc4
        self.vertical_state = enc1 - enc3


if __name__ == "__main__":
    bender = BendingControl("/dev/ttyACM0", "/dev/ttyACM1")
    bender.run()
