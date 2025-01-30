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


# Call the function with the path to the log file
calculate_average_duration("logs/log.txt")
