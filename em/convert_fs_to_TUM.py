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
        first = True
        o_T_w = None

        for i, line in enumerate(lines):
            # Each line has 12 floats: px, py, pz, Tx, Ty, Tz, Nx, Ny, Nz, Bx, By, Bz
            vals = line.strip().split(",")
            if len(vals) != 12:
                continue

            # Parse floats
            x, y, z = map(float, vals[0:3])
            x = x / 1000.0
            y = y / 1000.0
            z = z / 1000.0
            tx, ty, tz = map(float, vals[3:6])
            nx, ny, nz = map(float, vals[6:9])
            bx, by, bz = map(float, vals[9:12])

            # Build rotation matrix as in file: t forward, n down, b left
            rot_matrix = np.array([[tx, nx, bx], [ty, ny, by], [tz, nz, bz]])

            # Check determinant and adjust if necessary
            if np.linalg.det(rot_matrix) < 0:
                print("Determinant is negative. Adjusting rotation matrix.")
                rot_matrix[:, 2] *= -1.0

            # Verify orthogonality
            RtR = rot_matrix.T @ rot_matrix
            I = np.eye(3)
            error = RtR - I
            max_error = np.abs(error).max()
            if max_error > 1e-6:
                print(f"Rotation matrix not orthogonal enough. Max error: {max_error}")
                continue

            # Handle first frame
            if first:
                o_T_w = np.eye(4)
                o_T_w[:3, :3] = rot_matrix
                o_T_w[:3, 3] = [x, y, z]
                first = False

            # Build o_T_ci
            o_T_ci = np.eye(4)
            o_T_ci[:3, :3] = rot_matrix
            o_T_ci[:3, 3] = [x, y, z]

            # Compute w_T_ci = (o_T_w)^-1 * o_T_ci
            o_T_w_inv = np.linalg.inv(o_T_w)
            w_T_ci = o_T_w_inv @ o_T_ci

            # Rotate 90 degrees around n axis
            Ry = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
            Ry_inv = Ry.T

            # Apply rotation correction
            R_new = Ry_inv @ w_T_ci[:3, :3] @ Ry
            t_new = w_T_ci[:3, 3]

            # Convert to quaternion
            rot = R.from_matrix(R_new.astype(np.float64))
            qx, qy, qz, qw = rot.as_quat()

            # Write TUM format: timestamp tx ty tz qx qy qz qw
            timestamp = float(i)
            fout.write(
                f"{timestamp:.6f} {t_new[0]:.6f} {t_new[1]:.6f} {t_new[2]:.6f} {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n"
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
