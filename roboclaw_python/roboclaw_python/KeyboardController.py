from pynput import keyboard
from BendingControl import BendingControl


class KeyboardController:
    def __init__(self):
        self.bending_control = BendingControl(
            "/dev/ttyACM0", "/dev/ttyACM1", plot=False
        )
        self.listener = keyboard.Listener(
            on_press=self.on_press, on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        try:
            if key.char == "q":
                print("Exiting program.")
                self.bending_control.on_close()
                return False  # Stop listener
            elif key.char == "w":
                self.bending_control.set_moving_up(True)
                # print("KC: Moving up set to ", self.bending_control.moving_up)
            elif key.char == "s":
                self.bending_control.set_moving_down(True)
                # print("KC: Moving down set to ", self.bending_control.moving_down)
            elif key.char == "d":
                self.bending_control.set_moving_right(True)
                # print("KC: Moving right set to ", self.bending_control.moving_right)
            elif key.char == "a":
                self.bending_control.set_moving_left(True)
                # print("KC: Moving left set to ", self.bending_control.moving_left)
            elif key.char == "p":
                self.bending_control.print_current_positions()
        except AttributeError:
            pass  # Special keys (like arrow keys) don't have a char attribute

    def on_release(self, key):
        try:
            if key.char == "w":
                self.bending_control.set_moving_up(False)
                # print("KC: Moving up set to ", self.bending_control.moving_up)
            elif key.char == "s":
                self.bending_control.set_moving_down(False)
                # print("KC: Moving down set to ", self.bending_control.moving_down)
            elif key.char == "d":
                self.bending_control.set_moving_right(False)
                # print("KC: Moving right set to ", self.bending_control.moving_right)
            elif key.char == "a":
                self.bending_control.set_moving_left(False)
                # print("KC: Moving left set to ", self.bending_control.moving

        except AttributeError:
            pass
