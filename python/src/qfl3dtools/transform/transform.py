import numpy as np
import open3d as o3d
import os


def get_affine_matrix(xy, direction='anticlockwise'):
    """
    Get affine transformation matrix for rotating a rectangular plot pointcloud
    to be axis aligned

    Args
        xy (numpy.array): points of dimension (N x 2)
        direction (str): rotation direction, either 'anticlockwise' or clockwise (default: 'anticlockwise')

    Returns
        (numpy.array): affine transformation matrix (4 x 4) 
    """
    # Make sure to only get x and y column in case 3d points are given
    xy = xy[:, :2]
    
    p_left = xy[np.argmin(xy[:, 0])]
    p_bottom = xy[np.argmin(xy[:, 1])]

    if direction == 'clockwise':
        p0 = p_bottom.copy()
        x, y = p_left - p0
        theta = - np.arctan(-x / y)
    elif direction == 'anticlockwise':
        p0 = p_left.copy()
        x, y = p_bottom - p0
        theta = np.arctan(-y / x)  

    # Rotation matrix for the computed angle
    cos_a = np.cos(theta)
    sin_a = np.sin(theta)

    # Translation
    R = np.array([
        [cos_a, -sin_a],
        [sin_a, cos_a],
    ])
    T = - R @ p0.reshape(2, 1)

    # Affine transformation matrix
    M = np.array([
        [cos_a, -sin_a, 0, T[0, 0]],
        [sin_a, cos_a, 0, T[1, 0]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    return M


def transform_points_affine(xyz, M):
    xyz_1 = np.hstack((xyz, np.ones((xyz.shape[0], 1))))
    xyz_1_transf = M @ xyz_1.T
    return xyz_1_transf[:3, :].T


def get_inverse_affine_matrix(M):
    R = M[:2, :2]
    T = M[:2, -1]

    T_inv = - R.T @ T

    M_inv = np.array([
        [R.T[0, 0], R.T[0, 1], 0, T_inv[0]],
        [R.T[1, 0], R.T[1, 1], 0, T_inv[1]],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    return M_inv


def write_affine_matrix(matrix, outpath):
    """
    Save 4x4 affine matrix to csv file
    """
    np.savetxt(outpath, matrix, delimiter=",")


def read_affine_matrix(path):
    """
    Read 4x4 affine matrix from csv file
    """
    return np.loadtxt(path, delimiter=",")


def affine_transform_mesh(inpath, outpath, matrix):
    """
    Args
        inpath (str): path to mesh file or directory with mesh files (ply or obj)
        outpath (str): path or directory to write output mesh file(s) (ply or obj)
        matrix (numpy.array): 4x4 affine transformation matrix
    """
    # Get all input and output filepaths to meshes 
    if os.path.isdir(inpath):
        os.makedirs(outpath, exist_ok=True)
        inpaths = [os.path.join(inpath, f) for f in os.listdir(inpath) if f[-3:] in ['ply', 'obj']]
        outpaths = [os.path.join(outpath, f[:-3] + 'ply') for f in os.listdir(inpath) if f[-3:] in ['ply', 'obj']]
    elif os.path.isfile(inpath):
        inpaths = [inpath]
        outpaths = [outpath]
    else:
        raise ValueError('inpath or outpath is not a directory or file')

    # Loop over meshes
    for inpath, outpath in zip(inpaths, outpaths):

        # Read mesh
        print(f'Reading {inpath}')
        mesh = o3d.io.read_triangle_mesh(inpath)

        # Transform points with affine transformation matrix
        points = np.array(mesh.vertices)
        points_transf = transform_points_affine(points, matrix)
        mesh_transf = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(points_transf), mesh.triangles)

        # Write transformed mesh
        print(f'Writing projected mesh to {outpath}')
        o3d.io.write_triangle_mesh(outpath, mesh_transf)


def crop_and_downsample(path_in, path_out, min_x=None, max_x=None, min_y=None, max_y=None, voxel_size=None):
    """
    Crop a point cloud axis aligned given minimum and maximum x and y bounds, and/or downsample. 

    Args
        path_in (str): path to point cloud (ply or obj)
        path_out (str): path to cropped/downsampled point cloud (ply or obj)
        min_x (float): minimum x value for cropping
        max_x (float): maximum x value for cropping
        min_y (float): minimum y value for cropping
        max_y (float): maximum y value for cropping
        voxel_size (float): voxel size to downsample to

    """
    print('reading point cloud')
    pcd = o3d.t.io.read_point_cloud(path_in)

    # Clip if bounds provided
    if None not in (min_x, max_x, min_y, max_y):
        aabb = o3d.t.geometry.AxisAlignedBoundingBox(
            min_bound=[min_x, min_y, -float('inf')],
            max_bound=[max_x, max_y, float('inf')]
        )
        print('cropping point cloud')
        pcd = pcd.crop(aabb)

    # Downsample if voxel size is provided
    if voxel_size:
        print('downsampling point cloud')
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    print('saving point cloud')
    o3d.t.io.write_point_cloud(path_out, pcd)