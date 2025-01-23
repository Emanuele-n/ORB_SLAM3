import numpy as np
from scipy.spatial.transform import Rotation as R


def convert_dataset_format(
    input_txt,
    output_txt,
    # Decide which camera axis is "forward" → tangent
    # Typically, if camera looks along +Z in its own frame, we can pick z-axis
    # as tangent, y-axis as normal, x-axis as binormal. Adjust as needed.
    tangent_axis,  # "x", "y", or "z"
    normal_axis,  # "x", "y", or "z" different from tangent
    binormal_axis,  # whichever axis is left
):
    """
    Convert lines of the form:
        frame_index Tx Ty Tz Qx Qy Qz Qw
    into lines of the form:
        Tx, Ty, Tz, T_x, T_y, T_z, N_x, N_y, N_z, B_x, B_y, B_z
    and save to output_txt.
    """

    axis_map = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }

    # Safety check
    if len({tangent_axis, normal_axis, binormal_axis}) < 3:
        raise ValueError(
            "tangent_axis, normal_axis, and binormal_axis must all be different!"
        )

    with open(input_txt, "r") as fin, open(output_txt, "w") as fout:
        for line in fin:
            if not line.strip():
                continue  # skip empty lines

            # Parse each line:
            # frame_index Tx Ty Tz Qx Qy Qz Qw
            parts = line.strip().split()
            if len(parts) < 8:
                continue  # skip malformed lines

            frame_idx = float(parts[0])  # Not used in output, but you have it
            Tx = float(parts[1]) / 1000.0  # Convert to meters
            Ty = float(parts[2]) / 1000.0  # Convert to meters
            Tz = float(parts[3]) / 1000.0  # Convert to meters
            Qx = float(parts[4])
            Qy = float(parts[5])
            Qz = float(parts[6])
            Qw = float(parts[7])

            # Convert quaternion → rotation matrix
            # Rotation.from_quat expects [x, y, z, w] order
            rot = R.from_quat([Qx, Qy, Qz, Qw])
            R_mat = rot.as_matrix()  # 3x3

            # Depending on your chosen camera-forward axis,
            # extract the tangent (T), normal (N), and binormal (B).
            # If the camera's +Z is "forward," then the third column of R_mat
            # is that forward direction in world space, etc.

            # Helper to pick which column of R_mat is which axis:
            def extract_axis_from_matrix(matrix, which):
                """
                which: 'x', 'y', or 'z'.
                'x' → column 0
                'y' → column 1
                'z' → column 2
                """
                idx = {"x": 0, "y": 1, "z": 2}[which]
                return matrix[:, idx]

            # For example:
            T_vec = extract_axis_from_matrix(R_mat, tangent_axis)
            N_vec = extract_axis_from_matrix(R_mat, normal_axis)
            B_vec = extract_axis_from_matrix(R_mat, binormal_axis)

            # Now write out in the desired format:
            # Tx, Ty, Tz, T_x, T_y, T_z, N_x, N_y, N_z, B_x, B_y, B_z
            fout.write(
                f"{Tx}, {Ty}, {Tz}, "
                f"{T_vec[0]}, {T_vec[1]}, {T_vec[2]}, "
                f"{N_vec[0]}, {N_vec[1]}, {N_vec[2]}, "
                f"{B_vec[0]}, {B_vec[1]}, {B_vec[2]}\n"
            )

    print(f"Conversion complete. Output saved to: {output_txt}")


if __name__ == "__main__":
    # Example usage
    input_file = "dataset/stable_seq_000_part_0_dif_0.txt"
    output_file = "b1.txt"

    # Adjust these axes if your camera forward axis is different:
    # E.g., if the camera forward is the -Z axis, or if you want T = +X, etc.
    convert_dataset_format(
        input_txt=input_file,
        output_txt=output_file,
        tangent_axis="x",  # camera forward
        normal_axis="y",  # camera up
        binormal_axis="z",  # camera right
    )
