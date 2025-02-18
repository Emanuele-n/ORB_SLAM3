import os
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# Function to calculate the average duration from a file
def calculate_average_duration(file_path):
    durations = []

    with open(file_path, "r") as file:
        for line in file:
            # Extract the number part of each line
            parts = line.strip().split(": ")
            if len(parts) == 2:
                try:
                    duration = float(parts[1].replace("milliseconds", "").strip())
                    durations.append(duration)
                except ValueError:
                    # Skip any line where the conversion fails
                    pass

    # Calculate the average
    if durations:
        average_duration = sum(durations) / len(durations)
        print(f"Average duration: {average_duration:.2f} milliseconds")
    else:
        print("No valid durations found in the file.")


def plot_trajectory_from_logs(input_prefix):

    # Get all files in logs directory that start with input_prefix
    log_files = [
        f
        for f in os.listdir("logs/")
        if f.startswith(input_prefix) and f.endswith(".txt")
    ]
    print(log_files)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    for log_file in log_files:
        print(f"Plotting {log_file}")
        positions = []
        with open(os.path.join("logs", log_file), "r") as f:
            for line in f:
                # Split line and convert first three values to float
                try:
                    values = line.strip().split(",")
                    pos = [float(values[0]), float(values[1]), float(values[2])]
                    positions.append(pos)
                except (ValueError, IndexError):
                    continue

        if positions:
            positions = np.array(positions)
            ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label=log_file)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    # TODO: use this https://github.com/MichaelGrupp/evo
    plot_trajectory_from_logs("trajectory_record_b1_1738283378.8285117")
    # calculate_average_duration("logs/log.txt")
