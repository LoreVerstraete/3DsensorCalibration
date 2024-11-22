import open3d as o3d
import numpy as np
import yaml
from datetime import datetime

def ImportTarget(yamlfile, visualise = False, save=False):
    # Read yaml file
    with open(yamlfile, 'r') as file:
        target = yaml.safe_load(file)

    # Extract the mesh
    mesh = o3d.io.read_triangle_mesh(target['filepath'])
    mesh.scale(target['scaling_factor'], center=mesh.get_center()) 
    
    # Align origin with minima
    vertices = np.asarray(mesh.vertices)
    min_x = np.min(vertices[:, 0])  # Minimum x-coordinate
    min_y = np.min(vertices[:, 1])  # Minimum y-coordinate
    min_z = np.min(vertices[:, 2])  # Minimum z-coordinate
    mesh.translate([-min_x, -min_y, -min_z])

    # Crop mesh if cropped values are given
    try:
        min_bound = np.array([target['crop']['min']['x'], target['crop']['min']['y'], target['crop']['min']['z']])
        max_bound = np.array([target['crop']['max']['x'], target['crop']['max']['y'], target['crop']['max']['z']])
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
        mesh = mesh.crop(aabb)
    except:
        pass

    # Convert mesh to pointcloud
    pointcloud = mesh.sample_points_uniformly(number_of_points=10000)
    pointcloud.paint_uniform_color([0, 1, 0])
    pointcloud.translate(-(np.min(np.asarray(pointcloud.points),axis=0)))

    # Transform the already cropped pointcloud from origin at minima
    transform = np.eye(4)
    transform[:3,:3] = MakeRotationMatrix(target['rotate_degrees'])
    transform[0,3] = target['translate']['x']
    transform[1,3] = target['translate']['y']
    transform[2,3] = target['translate']['z']
    pointcloud.transform(transform)
    
    if visualise:
        o3d.visualization.draw_geometries([pointcloud])

    if save:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M")
        name = f'./pointclouds/pc_target_{timestamp}.pcd'
        o3d.io.write_point_cloud(name, pointcloud)

    print('Target received')
    
    return pointcloud

def MakeRotationMatrix(rotate_degrees):
    roll = np.deg2rad(rotate_degrees['roll']) 
    pitch = np.deg2rad(rotate_degrees['pitch'])
    yaw = np.deg2rad(rotate_degrees['yaw'])

    # Compute individual rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    R = np.round(np.dot(R_z, np.dot(R_y, R_x)),10)
    
    return R

if __name__ == '__main__':
    ImportTarget('./target.yaml', visualise=True, save=False)