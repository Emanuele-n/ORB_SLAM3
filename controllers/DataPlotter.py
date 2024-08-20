from collections import deque
import socket
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading


class DataPlotter:
    def __init__(self):
        # Initialize plotting variables
        self.fig, self.ax = plt.subplots()
        # Initialize bar plot with 4 bars
        self.bars = self.ax.bar(
            range(4), [0, 0, 0, 0], color=["blue", "green", "red", "purple"]
        )
        self.ax.set_xticks(range(4))
        self.ax.set_xticklabels(["p1", "p2", "p3", "p4"])
        self.ax.set_ylim(0, 10)  # Set an example limit for bar height, adjust as needed

    def update(self, heights):
        """Update the heights of the bars."""
        for bar, height in zip(self.bars, heights):
            bar.set_height(height)
        return self.bars

    def listen_for_data(self):
        """Listen for incoming data, apply filtering if necessary, and yield it for the plot."""
        host, port = "127.0.0.1", 65432

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((host, port))
            s.listen()
            print("Listening for connection...")
            conn, addr = s.accept()
            print(f"Connected by {addr}")
            buffer = ""
            while True:
                data = conn.recv(1024)
                if not data:
                    print("No data received, breaking connection loop.")
                    break
                buffer += data.decode("utf-8")
                while "\n" in buffer:
                    message, buffer = buffer.split("\n", 1)
                    try:
                        received_data = json.loads(message)
                        if all(
                            key in received_data for key in ["p1", "p2", "p3", "p4"]
                        ):
                            heights = [
                                received_data["p1"],
                                received_data["p2"],
                                received_data["p3"],
                                received_data["p4"],
                            ]
                            print(heights)
                            yield heights
                        else:
                            print("Invalid data format.")
                    except json.JSONDecodeError as e:
                        print(f"Error decoding JSON: {e}")

    def animate(self):
        """Create and run the animation using data fetched from the generator."""
        generator = self.listen_for_data()
        ani = FuncAnimation(
            self.fig, self.update, frames=generator, blit=True, repeat=False
        )
        plt.show()


if __name__ == "__main__":
    data_plotter = DataPlotter()
    threading.Thread(target=data_plotter.listen_for_data, daemon=True).start()
    data_plotter.animate()
