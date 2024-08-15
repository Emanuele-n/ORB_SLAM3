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

import json
import socket
import threading
import time
from roboclaw_python.roboclaw_3 import Roboclaw
import serial
import re


class RobotControl:
    def __init__(
        self, couple12_port, couple34_port, arduino_port, baud_rate=115200, plot=False
    ):

        # Initialize motors
        self.couple12 = Roboclaw(couple12_port, baud_rate)
        self.couple34 = Roboclaw(couple34_port, baud_rate)
        self.couple12.Open()
        self.couple34.Open()
        self.address = 0x80

        # Set up the serial connection
        self.arduino = serial.Serial(arduino_port, 9600)

        # # Initialize state variables
        # self.horizontal_state = 0
        # self.vertical_state = 0

        # Initialize parameters
        # self.horizontal_limit = [-4000000, 4000000]
        # self.vertical_limit = [-3000000, 3000000]
        self.speed = 100
        self.running = True

        # # Encoders moving average
        # moving_avg_size = 10
        # self.moving_avg_size = moving_avg_size
        # self.enc1_buffer = [0] * moving_avg_size
        # self.enc2_buffer = [0] * moving_avg_size
        # self.enc3_buffer = [0] * moving_avg_size
        # self.enc4_buffer = [0] * moving_avg_size

        # Variable set byt the parent keyboard controller
        self.moving_up = False
        self.moving_down = False
        self.moving_right = False
        self.moving_left = False
        self.moving_in = False
        self.moving_out = False
        self.move_up_prev = False
        self.move_down_prev = False
        self.move_right_prev = False
        self.move_left_prev = False
        self.moving_in_prev = False
        self.moving_out_prev = False

        # Set intial position to zero
        self.go_to_zero()

        # # Update states thread
        # update_thread = threading.Thread(target=self.listen_to_encoders, daemon=True)
        # update_thread.start()

        # Main thread
        self.main_thread = threading.Thread(target=self.run, daemon=True)
        self.main_thread.start()

        # # Print thread
        # print_thread = threading.Thread(target=self.print_current_position, daemon=True)
        # print_thread.start()

        # if plot:
        #     # Send data thread
        #     send_thread = threading.Thread(target=self.sendData, daemon=True)
        #     send_thread.start()

        print("Robot control initialized")

    def go_to_zero(self):
        print("Moving to zero position")
        # Position all motors to zero position
        self.couple12.SpeedAccelDeccelPositionM1(
            self.address, 320000, 500000, 320000, 0, 1
        )
        self.couple12.SpeedAccelDeccelPositionM2(
            self.address, 320000, 500000, 320000, 0, 1
        )
        self.couple34.SpeedAccelDeccelPositionM1(
            self.address, 320000, 500000, 320000, 0, 1
        )
        self.couple34.SpeedAccelDeccelPositionM2(
            self.address, 320000, 500000, 320000, 0, 1
        )
        time.sleep(1)
        print("Motors at zero position")

    # Move functions TODO: adjust logic based on pressure sensors data
    def move_down(self):
        print("BC: Moving down")
        # if pressure_1 < pressure_3:
        self.couple34.ForwardM1(self.address, self.speed)
        # else:
        #     self.couple12.BackwardM1(self.address, self.speed)
        print("Speed: {}".format(self.speed))
        time.sleep(0.1)

    def move_left(self):
        print("BC: Moving left")
        # if pressure_2 < pressure_4:
        self.couple34.ForwardM2(self.address, self.speed)
        # else:
        #     self.couple12.BackwardM2(self.address, self.speed)
        print("Speed: {}".format(self.speed))
        time.sleep(0.1)

    def move_right(self):
        print("BC: Moving right")
        # if pressure_2 > pressure_4:
        self.couple12.ForwardM2(self.address, self.speed)
        # else:
        #     self.couple34.BackwardM2(self.address, self.speed)
        print("Speed: {}".format(self.speed))
        time.sleep(0.1)

    def move_up(self):
        print("BC: Moving up")
        # if pressure_1 > pressure_3:
        self.couple12.ForwardM1(self.address, self.speed)
        # else:
        #     self.couple34.BackwardM1(self.address, self.speed)
        print("Speed: {}".format(self.speed))
        time.sleep(0.1)

    # ----------------------------

    def on_close(self):
        # Stop run thread and close motors
        self.stop()
        self.main_thread.join()

        print("Closing motors")
        self.go_to_zero()

        print("Closing serial connection")
        self.send_arduino("s")
        self.arduino.close()

    def read_arduino(self):
        pass
        # try:
        #     while True:
        #         # Read a line from the serial port
        #         line = arduino.readline().decode("utf-8").strip()
        #         if self.debug:
        #             print(line)

        #         # Extract the pressure value using regex
        #         match = re.search(r"Pressure from A0: (\d+\.\d+) MPa", line)
        #         if match:
        #             pressure_value = float(match.group(1))
        #             # if self.debug : print(pressure_value)

        #             if pressure_value < 0.0:
        #                 pressure_value = 0.0

        #             if pressure_value > 2:
        #                 pressure_value = 2

        #             # Set the pressure value
        #             self.constraints[0].value = [pressure_value]
        #             # if self.debug : print("Pressure value: " + str(self.constraints[0].value.value[0]))

        # except KeyboardInterrupt:
        #     # Close the serial connection when you terminate the script
        #     arduino.close()
        #     if self.debug:
        #         print("Serial connection closed.")

    # main loop
    def run(self):
        while self.running:
            # print("run")
            # print(self.moving_up, self.moving_down, self.moving_right, self.moving_left)
            if self.moving_up and not self.move_up_prev:
                self.move_up()
                self.move_up_prev = True
            if not self.moving_up and self.move_up_prev:
                self.stop_motors()
                self.move_up_prev = False
            if self.moving_down and not self.move_down_prev:
                self.move_down()
                self.move_down_prev = True
            if not self.moving_down and self.move_down_prev:
                self.stop_motors()
                self.move_down_prev = False
            if self.moving_right and not self.move_right_prev:
                self.move_right()
                self.move_right_prev = True
            if not self.moving_right and self.move_right_prev:
                self.stop_motors()
                self.move_right_prev = False
            if self.moving_left and not self.move_left_prev:
                self.move_left()
                self.move_left_prev = True
            if not self.moving_left and self.move_left_prev:
                self.stop_motors()
                self.move_left_prev = False
            if self.moving_in and not self.moving_in_prev:
                if self.moving_out:
                    self.moving_out = False
                    continue
                self.send_arduino("f")
                self.moving_in_prev = True
            if not self.moving_in and self.moving_in_prev:
                self.send_arduino("s")
                self.moving_in_prev = False
            if self.moving_out and not self.moving_out_prev:
                if self.moving_in:
                    self.moving_in = False
                    continue
                self.send_arduino("b")
                self.moving_out_prev = True
            if not self.moving_out and self.moving_out_prev:
                self.send_arduino("s")
                self.moving_out_prev = False
            # time.sleep(0.01)

    def send_arduino(self, command):
        self.arduino.write(command.encode())
        print(f"Sent '{command}' to Arduino")
        time.sleep(0.1)

    # Set functions
    def set_moving_up(self, value):
        self.moving_up = value

    def set_moving_down(self, value):
        self.moving_down = value

    def set_moving_right(self, value):
        self.moving_right = value

    def set_moving_left(self, value):
        self.moving_left = value

    def set_moving_in(self, value):
        self.moving_in = value

    def set_moving_out(self, value):
        self.moving_out = value

    def stop(self):
        self.running = False

    # ----------------------------

    def stop_motors(self):
        print("BC: Stopping motors")
        self.couple12.ForwardM1(self.address, 0)
        self.couple12.ForwardM2(self.address, 0)
        self.couple34.ForwardM1(self.address, 0)
        self.couple34.ForwardM2(self.address, 0)
        time.sleep(0.1)


"""
OLD FUNCTIONS:


    def check_limits(self):
        if self.horizontal_state < self.horizontal_limit[0]:
            print("Horizontal left limit reached")
            return True
        if self.horizontal_state > self.horizontal_limit[1]:
            print("Horizontal right limit reached")
            return True

        if self.vertical_state < self.vertical_limit[0]:
            print("Vertical down limit reached")
            return True
        if self.vertical_state > self.vertical_limit[1]:
            print("Vertical up limit reached")
            return True
        return False














    def listen_to_encoders(self):
        while True:
            self.update_states()
            time.sleep(0.01)










    def print_current_position(self):
        print(
            "Current position: x: {}, y: {}".format(
                self.horizontal_state, self.vertical_state
            )
        )
        time.sleep(0.1)












    def read_current_position(self):
        enc1 = self.couple12.ReadEncM1(self.address)[1]
        enc2 = self.couple12.ReadEncM2(self.address)[1]
        enc3 = self.couple34.ReadEncM1(self.address)[1]
        enc4 = self.couple34.ReadEncM2(self.address)[1]
        return enc1, enc2, enc3, enc4









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
        # Read current position from encoders
        start_time = time.time()
        raw_enc1, raw_enc2, raw_enc3, raw_enc4 = self.read_current_position()
        end_time = time.time()

        elapsed_time = end_time - start_time
        print("Time to read encoders: {} seconds".format(elapsed_time))

        # Update buffers
        self.enc1_buffer.append(raw_enc1)
        self.enc2_buffer.append(raw_enc2)
        self.enc3_buffer.append(raw_enc3)
        self.enc4_buffer.append(raw_enc4)

        # Maintain buffer size
        self.enc1_buffer.pop(0)
        self.enc2_buffer.pop(0)
        self.enc3_buffer.pop(0)
        self.enc4_buffer.pop(0)

        # Calculate moving averages
        avg_enc1 = sum(self.enc1_buffer) / len(self.enc1_buffer)
        avg_enc2 = sum(self.enc2_buffer) / len(self.enc2_buffer)
        avg_enc3 = sum(self.enc3_buffer) / len(self.enc3_buffer)
        avg_enc4 = sum(self.enc4_buffer) / len(self.enc4_buffer)

        # Update state variables with averaged values
        self.horizontal_state = int(avg_enc2 - avg_enc4)
        self.vertical_state = int(avg_enc1 - avg_enc3)
        print("x: {}, y: {}".format(self.horizontal_state, self.vertical_state))
        time.sleep(0.01)

        # Check limits
        if self.check_limits():
            self.stop_motors()
















    def control_position(self, direction):
        # Check direction is valid
        if direction not in ["up", "down", "left", "right"]:
            print("Invalid direction")
            return
        print(f"Moving {direction}")

        # Get current position
        current_horizontal = self.horizontal_state
        current_vertical = self.vertical_state

        # Check if the new position is within the limits
        if (
            direction == "up"
            and current_vertical + self.increment > self.vertical_limit[1]
        ):
            print("Vertical limit reached")
            return
        if (
            direction == "down"
            and current_vertical - self.increment < self.vertical_limit[0]
        ):
            print("Vertical limit reached")
            return
        if (
            direction == "right"
            and current_horizontal + self.increment > self.horizontal_limit[1]
        ):
            print("Horizontal limit reached")
            return
        if (
            direction == "left"
            and current_horizontal - self.increment < self.horizontal_limit[0]
        ):
            print("Horizontal limit reached")
            return

        if current_vertical >= self.increment:
            # 1
            if direction == "up":
                print("Case 1")
                new_position = current_vertical + self.increment
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
            # 2
            if direction == "down":
                print("Case 2")
                new_position = current_vertical - self.increment
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
        elif 0 <= current_vertical < self.increment:
            # 3
            if direction == "up":
                print("Case 3")
                new_position = current_vertical + self.increment
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
            # 4
            if direction == "down":
                print("Case 4")
                # Bring motor 1 to zero
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address, self.acceleration, self.speed, self.deceleration, 0, 1
                )
                # time.sleep(0.01)
                new_position = abs(current_vertical - self.increment)
                # Bring motor 3 to new position
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
        if current_vertical <= -self.increment:
            # 5
            if direction == "down":
                print("Case 5")
                new_position = abs(current_vertical - self.increment)
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
            # 6
            if direction == "up":
                print("Case 6")
                new_position = abs(current_vertical + self.increment)
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
        elif -self.increment < current_vertical < 0:
            # 7
            if direction == "down":
                print("Case 7")
                new_position = abs(current_vertical - self.increment)
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
            # 8
            if direction == "up":
                print("Case 8")
                # Bring motor 3 to zero
                self.couple34.SpeedAccelDeccelPositionM1(
                    self.address, self.acceleration, self.speed, self.deceleration, 0, 1
                )
                # time.sleep(0.01)
                new_position = abs(current_vertical + self.increment)
                # Bring motor 1 to new position
                self.couple12.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.acceleration,
                    self.speed,
                    self.deceleration,
                    new_position,
                    1,
                )
        if current_horizontal >= self.increment:
            # 9
            if direction == "right":
                print("Case 9")
                new_position = current_horizontal + self.increment
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
            # 10
            if direction == "left":
                print("Case 10")
                new_position = current_horizontal - self.increment
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
        elif 0 <= current_horizontal < self.increment:
            # 11
            if direction == "right":
                print("Case 11")
                new_position = current_horizontal + self.increment
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
            # 12
            if direction == "left":
                print("Case 12")
                # Bring motor 2 to zero
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address, self.acceleration, self.speed, self.deceleration, 0, 1
                )
                # time.sleep(0.01)
                new_position = abs(current_horizontal - self.increment)
                # Bring motor 4 to new position
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
        if current_horizontal <= -self.increment:
            # 13
            if direction == "left":
                print("Case 13")
                new_position = abs(current_horizontal - self.increment)
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
            # 14
            if direction == "right":
                print("Case 14")
                new_position = abs(current_horizontal + self.increment)
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
        elif -self.increment < current_horizontal < 0:
            # 15
            if direction == "left":
                print("Case 15")
                new_position = abs(current_horizontal - self.increment)
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
            # 16
            if direction == "right":
                print("Case 16")
                # Bring motor 4 to zero
                self.couple34.SpeedAccelDeccelPositionM2(
                    self.address, self.acceleration, self.speed, self.deceleration, 0, 1
                )
                # time.sleep(0.01)
                new_position = abs(current_horizontal + self.increment)
                # Bring motor 2 to new position
                self.couple12.SpeedAccelDeccelPositionM2(
                    self.address,
                    round(self.acceleration * 4 / 3),
                    round(self.speed * 4 / 3),
                    round(self.deceleration * 4 / 3),
                    new_position,
                    1,
                )
        # time.sleep(0.001)

"""
