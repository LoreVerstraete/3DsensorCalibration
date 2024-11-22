from icp_pointclouds import ApplyICP
from target import ImportTarget
from extract_data import Extract
from adding_noise import AddNoise

import open3d as o3d
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

def perform_icp(pointcloud, target, guess):
    # Apply the ICP method to transform the pc_sensors_full towards each other
    firstguess = True
    rmse = 100
    rmse_final = 0
    treshold = [0.05, 0.01, 0.005, 0.001, 0.0005, 0.0001, 0.00005, 0.00001]
    i = 0
    while rmse > 0.0 and i < len(treshold):
        if firstguess:
            Data = ApplyICP(pointcloud, target, treshold[i], guess)
            correction = Data.transform_icp
            firstguess = False
        else:
            Data = ApplyICP(Data.source_tr, target, treshold[i])
            if Data.rmse != 0.0:
                correction = np.dot(Data.transform_icp, correction)

        rmse = Data.rmse
        i = i + 1
    # o3d.visualization.draw_geometries([Data.source_tr + target])
    return correction

def transform_to_input(transform):
    rotation = R.from_matrix(transform[:3,:3])
    rotate = rotation.as_euler('xyz', degrees=False)
    translate = np.array([transform[0,3], transform[1,3], transform[2,3]])
    return  translate, rotate
   
def copy_pointcloud(pc):
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(np.asarray(pc.points))
    if pc.has_colors():
        new_pc.colors = o3d.utility.Vector3dVector(np.asarray(pc.colors))
    if pc.has_normals():
        new_pc.normals = o3d.utility.Vector3dVector(np.asarray(pc.normals))
    return new_pc


if __name__ == '__main__':
    # Import the target from the configurations of the yaml file
    target = ImportTarget('./target.yaml')

    # Extract the sensor data
    data = Extract('./sensors.yaml')

    # Visualise the initial pointclouds and the target
    o3d.visualization.draw_geometries([
        data.sensor[sensor]['pc_base'] for sensor in data.sensor.keys()]
         + [target])

    # Loop through all sensors                               
    icp_info = {}
    for sensor in data.sensor.keys():
        print(f"\n\033[95m--- {sensor} ---\033[0m")
        icp_info[sensor] = {}

        # Transform target to parent
        icp_info[sensor]['pc_target_parent'] = copy_pointcloud(target)
        icp_info[sensor]['pc_target_parent'].transform(data.sensor[sensor]['transformbaseparent'])

        # Apply ICP preferably with cropped sensor data
        try:
            o3d.visualization.draw_geometries([data.sensor[sensor]['pc_parent_crop'] + icp_info[sensor]['pc_target_parent']])
            icp_info[sensor]['correction'] = perform_icp(data.sensor[sensor]['pc_parent_crop'], icp_info[sensor]['pc_target_parent'], data.sensor[sensor]['init_guess'])
        except:
            o3d.visualization.draw_geometries([data.sensor[sensor]['pc_parent'] + icp_info[sensor]['pc_target_parent']])
            icp_info[sensor]['correction'] = perform_icp(data.sensor[sensor]['pc_parent'], icp_info[sensor]['pc_target_parent'],  data.sensor[sensor]['init_guess'])
        
        # Reconstruct full pointcloud with the correction of ICP
        icp_info[sensor]['transform_corrected'] = np.dot(icp_info[sensor]['correction'], data.sensor[sensor]['transformchildparent'])
        icp_info[sensor]['pc_parent_corrected'] = copy_pointcloud(data.sensor[sensor]['pc_child'])
        icp_info[sensor]['pc_parent_corrected'].transform(icp_info[sensor]['transform_corrected'])

        # Give the calibration output
        icp_info[sensor]['translation'], icp_info[sensor]['rotation'] = transform_to_input(icp_info[sensor]['transform_corrected'])
        # print(f' Transformation:')
        # print(icp_info[sensor]['transform_corrected'])
        print(f'Calibration input for the {sensor} which is the transform from {data.sensor[sensor]["parent"]} to {data.sensor[sensor]["child"]}:')
        print(f'- translation [x y z]:       {icp_info[sensor]["translation"].tolist()}')
        print(f'- rotation [roll pitch yaw]: {icp_info[sensor]["rotation"].tolist()}')

        # Visualise the corrected pointcloud and the target
        o3d.visualization.draw_geometries([icp_info[sensor]['pc_target_parent']+icp_info[sensor]['pc_parent_corrected']])

    # Visualise all corrected pointclouds and the target
    o3d.visualization.draw_geometries([
        icp_info[sensor]['pc_parent_corrected'].transform(data.sensor[sensor]['transformparentbase']) for sensor in data.sensor.keys()]
         + [target])
