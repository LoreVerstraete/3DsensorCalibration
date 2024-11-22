from icp_pointclouds import ApplyICP
from extract_data import Extract
from adding_noise import AddNoise
from ransac_pointclouds import perform_RANSAC

import open3d as o3d
import numpy as np
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
            firstguess = False
        else:
            Data = ApplyICP(Data.source_tr, target, treshold[i])

        rmse = Data.rmse
        if rmse != 0.0:
            rmse_final = rmse

        i = i + 1
    print(rmse_final)
    return Data.source_tr, rmse_final

def crop_pointcloud(pointcloud, minbound, maxbound):
    # crop the point clouds
    min_bound = np.array(minbound)
    max_bound = np.array(maxbound)
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    pointcloud_cropped = pointcloud.crop(aabb)
    # o3d.visualization.draw_geometries([pointcloud_cropped])
    return pointcloud_cropped
   
def copy_pointcloud(pc):
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(np.asarray(pc.points))
    if pc.has_colors():
        new_pc.colors = o3d.utility.Vector3dVector(np.asarray(pc.colors))
    if pc.has_normals():
        new_pc.normals = o3d.utility.Vector3dVector(np.asarray(pc.normals))
    return new_pc


if __name__ == '__main__':
    # Extract the sensor data
    data = Extract('./sensors_simulation.yaml')

    # Extract target out of head camera point cloud
    target = crop_pointcloud(data.sensor['xtion']['pc_base'], [-2, -2, 0.1], [2, 2, 2])
    target.paint_uniform_color([0, 1, 0])

    # Visualise the initial pointclouds and the target
    o3d.visualization.draw_geometries([
        data.sensor[sensor]['pc_base'] for sensor in data.sensor.keys()]
         + [target])
    

    # Loop through all sensors                               
    icp_info = {}
    rmse = {}
    target_parent = {}    
    for sensor in data.sensor.keys():
        # Transform target to parent
        target_parent[sensor] = copy_pointcloud(target)
        target_parent[sensor].transform(data.sensor[sensor]['transformbaseparent'])
        
    guess = np.eye(4)

    for sensor in data.sensor.keys():
        icp_info[sensor] = {}
        rmse[sensor] = {}
        for rpy in [[0,0,0],[3,3,3],[6,6,6],[9,9,9]]:
            icp_info[sensor][str(rpy[0])] = {}
            rmse[sensor][str(rpy[0])] = {}
            for xyz in [[0,0,0], [0.03,0.03,0.03], [0.06,0.06,0.06], [0.09,0.09,0.09]]:
                icp_info[sensor][str(rpy[0])][str(xyz[0])] = {}
                rmse[sensor][str(rpy[0])][str(xyz[0])] = {}
                for i in range(1):
                    rmse[sensor][str(rpy[0])][str(xyz[0])][str(i)] = {}
                    print(f"\n\033[95m--- {sensor} with rpy is {rpy} and xyz is {xyz} iteration {i} ---\033[0m")


                    # Apply ICP preferably with cropped sensor data
                    try:
                        pc_noise = AddNoise(data.sensor[sensor]['pc_parent_crop'],rpy,xyz)
                    except:
                        pc_noise = AddNoise(data.sensor[sensor]['pc_parent'],rpy,xyz)
                        
                    # o3d.visualization.draw_geometries([pc_noise + target_parent[sensor]])
                    guess = perform_RANSAC(pc_noise, target_parent[sensor], visualize=True)
                    icp_info[sensor]['pc_transformed'], rmse[sensor][str(rpy[0])][str(xyz[0])][str(i)] = perform_icp(pc_noise, target_parent[sensor], guess)
                    
                    # Visualise the corrected pointcloud and the target
                    o3d.visualization.draw_geometries([target_parent[sensor]+icp_info[sensor]['pc_transformed']])

