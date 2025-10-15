import os
import glob
import argparse
import open3d as o3d
import numpy as np
import laspy


def get_file_extension(input_file):
    """
    Returns the file extension for the given file path in lowercase.
    If the file has no extension, returns an empty string.

    Args
        input_file:

    Returns

    """
    _, extension = os.path.splitext(input_file)
    if len(extension) == 0:
        extension = os.listdir(input_file)[0][-4:]
    return extension.lower()


def get_laz_backend():
    """
    """
    # Try using the Lazrs backend first
    backend = laspy.LazBackend.Lazrs
    if backend.is_available():
        print("Using lazrs backend for LAZ files.")
        return backend
    # Fallback: try the Laszip backend
    backend = laspy.LazBackend.Laszip
    if backend.is_available():
        print("Using laszip backend for LAZ files.")
        return backend
    raise Exception("No LAZ backend available. Please install lazrs or laszip.")



def las_to_ply(input_file, output_file):
    point_cloud = laspy.read(input_file)
    points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(output_file, point_cloud)


def laz_to_ply(input_file, output_file):
    laz_backend = get_laz_backend()
    point_cloud = laspy.read(input_file, laz_backend=laz_backend)
    points = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).T
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud(output_file, point_cloud)


def txt_to_ply(input_file, output_file):
    # Read txt file
    points = []
    with open(input_file, 'r') as f:
        for line in f:
            data = line.strip().split()
            if len(data) >= 3:
                points.append([float(data[0]), float(data[1]), float(data[2])])

    # Create PointCloud object
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points)

    # Write to ply file
    o3d.io.write_point_cloud(output_file, point_cloud)
    print(f"Converted {input_file} to {output_file}")



def read_pc_np(input_file, output_file):
    """
    Function that reads in files based on extension
    Inefficient as this selection is also made within files_to_ply function*
    """
    ext = get_file_extension(input_file)
    if ext == ".las":
        las_to_ply(input_file, output_file)
    elif ext == ".laz":
        laz_to_ply(input_file, output_file)
    elif ext == ".txt":
        txt_to_ply(input_file, output_file)
    else:
        raise ValueError(f"Unsupported file extension: {ext} for file {input_file}")


def files_to_ply(input_dir, output_dir):
    # Create output directory if not exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Iterate through txt files in input directory
    for filename in os.listdir(input_dir):
        if filename.endswith('.las'):
            input_file = os.path.join(input_dir, filename)
            output_file = os.path.join(output_dir, filename.replace('.las', '.ply'))
            read_pc_np(input_file, output_file)
        elif filename.endswith('.txt'):
            input_file = os.path.join(input_dir, filename)
            output_file = os.path.join(output_dir, filename.replace('.laz', '.ply'))
            read_pc_np(input_file, output_file)
        elif filename.endswith('.txt'):
            input_file = os.path.join(input_dir, filename)
            output_file = os.path.join(output_dir, filename.replace('.txt', '.ply'))
            read_pc_np(input_file, output_file)

