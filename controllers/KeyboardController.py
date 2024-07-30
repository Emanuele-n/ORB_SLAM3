from pynput import keyboard
from RobotControl import RobotControl


class KeyboardController:
    def __init__(self, couple12_port, couple34_port, arduino_port):
        self.robot_control = RobotControl(
            couple12_port=couple12_port,
            couple34_port=couple34_port,
            arduino_port=arduino_port,
            plot=False,
        )
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == "q":
                print("Exiting program.")
                self.robot_control.on_close()
                return False  # Stop listener
            elif key.char == "w":
                self.robot_control.set_moving_up(True)
                # print("KC: Moving up set to ", self.robot_control.moving_up)
            elif key.char == "s":
                self.robot_control.set_moving_down(True)
                # print("KC: Moving down set to ", self.robot_control.moving_down)
            elif key.char == "d":
                self.robot_control.set_moving_right(True)
                # print("KC: Moving right set to ", self.robot_control.moving_right)
            elif key.char == "a":
                self.robot_control.set_moving_left(True)
                # print("KC: Moving left set to ", self.robot_control.moving_left)
            elif key.char == "i":
                self.robot_control.set_moving_in(True)
                # print("KC: Moving in set to ", self.robot_control.moving_in)
            elif key.char == "o":
                self.robot_control.set_moving_out(True)
                # print("KC: Moving out set to ", self.robot_control.moving_out)
            elif key.char == "p":
                # self.robot_control.print_current_positions()
                print("Deprecated")
        except AttributeError:
            pass  # Special keys (like arrow keys) don't have a char attribute

    def on_release(self, key):
        try:
            if key.char == "w":
                self.robot_control.set_moving_up(False)
                # print("KC: Moving up set to ", self.robot_control.moving_up)
            elif key.char == "s":
                self.robot_control.set_moving_down(False)
                # print("KC: Moving down set to ", self.robot_control.moving_down)
            elif key.char == "d":
                self.robot_control.set_moving_right(False)
                # print("KC: Moving right set to ", self.robot_control.moving_right)
            elif key.char == "a":
                self.robot_control.set_moving_left(False)
                # print("KC: Moving left set to ", self.robot_control.moving
            elif key.char == "i":
                self.robot_control.set_moving_in(False)
                # print("KC: Moving in set to ", self.robot_control.moving_in)
            elif key.char == "o":
                self.robot_control.set_moving_out(False)
                # print("KC: Moving out set to ", self.robot_control.moving_out)
        except AttributeError:
            pass
