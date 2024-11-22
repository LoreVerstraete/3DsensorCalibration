import numpy as np
import random 
import open3d as o3d
from scipy.spatial.transform import Rotation as R

def AddNoise(pointcloud, rpy=[], translation=[], visualise=False, print=False):
    if rpy == []:
        rpy = [random.uniform(-5,5),
                random.uniform(-5,5),
                random.uniform(-5,5)]
    r = R.from_euler('xyz', rpy, degrees=True)
    rotation = r.as_matrix()

    if translation == []:
        translation = [random.uniform(-0.1,0.1),
                        random.uniform(-0.1,0.1),
                        random.uniform(-0.1,0.1)]

    transform = np.identity(4)
    transform[0:3,0:3] = rotation
    transform[0:3,3] = np.transpose(translation)

    if print:
        print(f'The noise rotations in degrees are (in roll/pitch/yaw): {rpy}')
        print(f'The noise translation is {translation}')
        print(f'The noise transform is \n{transform}') 

    pcd = pointcloud.transform(transform)

    if visualise:
        o3d.visualization.draw_geometries([pcd])

    return pcd

if __name__ == '__main__':
    AddNoise(o3d.io.read_point_cloud('./pointclouds/pc_target_20241031_1416.pcd'),[90,0,0],[0,0,0])