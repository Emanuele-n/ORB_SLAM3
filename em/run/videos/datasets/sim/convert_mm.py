import csv

input_path = "/home/emanuele/Desktop/github/ORB_SLAM3/em/run/videos/datasets/sim/ca_data_b1_1738215363.9776962.csv"
output_path = "/home/emanuele/Desktop/github/ORB_SLAM3/em/run/videos/datasets/sim/ca_data_b1_1738215363.9776962_mm.csv"

try:
    with open(input_path, "r") as fin, open(output_path, "w", newline="") as fout:
        reader = csv.reader(fin)
        writer = csv.writer(fout)
        next(reader)  # Skip the header row
        for row in reader:
            writer.writerow([float(row[0]), float(row[1]), float(row[2]) * 1000])
except FileNotFoundError:
    print("File not found")
except Exception as e:
    print("An error occurred: ", e)
else:
    print("Conversion completed successfully")
