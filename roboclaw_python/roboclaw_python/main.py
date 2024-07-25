from KeyboardController import KeyboardController

if __name__ == "__main__":
    keyboard_controller = KeyboardController()
    keyboard_controller.listener.join()
