from collections import deque
import socket
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

# Initialize plotting variables
fig, ax = plt.subplots()
# Make the scatter point larger and more visible
(point,) = ax.plot(
    [], [], "o", color="blue", markersize=10
)  # Changed to a plot point for better visibility

# Set plot limits and decorations
ax.set_xlim(-4000000, 4000000)
ax.set_ylim(-3000000, 3000000)
# Visual aids
ax.add_patch(
    plt.Rectangle(
        (-4000000, -3000000), 8000000, 6000000, fill=None, edgecolor="gray", linewidth=2
    )
)
ax.axhline(0, color="red", linestyle="--")
ax.axvline(0, color="green", linestyle="--")
ax.set_xlabel("LR (Left-Right Position)")
ax.set_ylabel("DU (Down-Up Position)")


def update(frame):
    """Update the plot point position with new coordinates."""
    point.set_data([frame[0]], [frame[1]])
    return (point,)


def listenForData():
    """Listen for incoming data, apply sophisticated filtering, and yield it for the plot."""
    host, port = "127.0.0.1", 65432
    window_size = 10  # Size of the moving average window
    data_queue_x = deque(maxlen=window_size)
    data_queue_y = deque(maxlen=window_size)

    def moving_average(values):
        return sum(values) / len(values)

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
                    if "x" in received_data and "y" in received_data:
                        x, y = received_data["x"], received_data["y"]
                        # Apply filtering for bounds
                        if -4000000 <= x <= 4000000 and -3000000 <= y <= 3000000:
                            # Filter data
                            data_queue_x.append(x)
                            data_queue_y.append(y)
                            smooth_x = moving_average(data_queue_x)
                            smooth_y = moving_average(data_queue_y)
                            position = (smooth_x, smooth_y)
                            # print(f"Filtered data: {position}")
                            yield position
                        else:
                            print(f"Invalid data received: x={x}, y={y}")
                except json.JSONDecodeError as e:
                    print(f"Error decoding JSON: {e}")


def animate():
    """Create and run the animation using data fetched from the generator."""
    generator = listenForData()
    ani = FuncAnimation(fig, update, frames=generator, blit=True, repeat=False)
    plt.show()


if __name__ == "__main__":
    threading.Thread(target=listenForData, daemon=True).start()
    animate()
