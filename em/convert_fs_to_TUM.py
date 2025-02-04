#!/usr/bin/env python3

import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
import os


def convert_fs_to_tum(input_file, output_file):
    """
    Convert Frenet-Serret frames (px, py, pz, Tx, Ty, Tz, Nx, Ny, Nz, Bx, By, Bz)
    into TUM format:

        timestamp tx ty tz qx qy qz qw

    The rotation columns (Tx,Nx,Bx; Ty,Ny,By; Tz,Nz,Bz) are interpreted as a
    right-handed rotation matrix from local -> world.
    """

    with open(input_file, "r") as fin, open(output_file, "w") as fout:
        lines = fin.readlines()

        for i, line in enumerate(lines):
            # Each line has 12 floats: px, py, pz, Tx, Ty, Tz, Nx, Ny, Nz, Bx, By, Bz
            vals = line.strip().split(",")
            if len(vals) != 12:
                # You might have lines with trailing spaces, or empty lines. Handle them gracefully.
                continue

            # Parse floats
            px, py, pz = map(float, vals[0:3])
            px = px / 1000.0
            py = py / 1000.0
            pz = pz / 1000.0
            Tx, Ty, Tz = map(float, vals[3:6])
            Nx, Ny, Nz = map(float, vals[6:9])
            Bx, By, Bz = map(float, vals[9:12])

            # Build the rotation matrix (columns are T, N, B)
            # R = [ [Tx Nx Bx],
            #       [Ty Ny By],
            #       [Tz Nz Bz] ]
            rot_mat = np.array([[Tx, Nx, Bx], [Ty, Ny, By], [Tz, Nz, Bz]])

            # Convert rotation matrix to quaternion (qx, qy, qz, qw)
            # Note: scipy's as_quat() returns [qx, qy, qz, qw].
            rot = R.from_matrix(rot_mat)
            qx, qy, qz, qw = rot.as_quat()

            # In TUM format:  time tx ty tz qx qy qz qw
            # We can use the line index as the "timestamp" or any other scheme you prefer.
            timestamp = float(i)  # or e.g. i * 0.01, etc.

            fout.write(
                f"{timestamp:.6f} {px:.6f} {py:.6f} {pz:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
            )


def main():
    parser = argparse.ArgumentParser(
        description="Convert Frenet-Serret frames to TUM trajectory format."
    )
    parser.add_argument(
        "input_path",
        type=str,
        help="Path to the input .txt file or folder containing .txt files.",
    )
    parser.add_argument(
        "output_folder",
        type=str,
        help="Path to the output folder for TUM-format trajectories.",
    )
    args = parser.parse_args()

    # Create output folder if it doesn't exist
    os.makedirs(args.output_folder, exist_ok=True)

    if os.path.isfile(args.input_path) and args.input_path.endswith(".txt"):
        # Single file processing
        basename = os.path.splitext(os.path.basename(args.input_path))[0]
        output_file = os.path.join(args.output_folder, f"{basename}_gt.txt")
        convert_fs_to_tum(args.input_path, output_file)
        print(f"Converted {args.input_path} -> {output_file}")

    elif os.path.isdir(args.input_path):
        # Process all .txt files in the folder
        for filename in os.listdir(args.input_path):
            if filename.endswith(".txt"):
                input_file = os.path.join(args.input_path, filename)
                basename = os.path.splitext(filename)[0]
                output_file = os.path.join(args.output_folder, f"{basename}_gt.txt")
                convert_fs_to_tum(input_file, output_file)
                print(f"Converted {input_file} -> {output_file}")

    else:
        print("Error: Input path must be a .txt file or a folder containing .txt files")


if __name__ == "__main__":
    main()


# /home/emanuele/Desktop/github/ORB_SLAM3/em/run/centerline_frames/sim/b21/b21.txt
# /home/emanuele/Desktop/github/ORB_SLAM3/em/logs/b21_gt.txt
