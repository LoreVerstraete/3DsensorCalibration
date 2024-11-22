import open3d as o3d
import numpy as np
import copy

def perform_RANSAC(source, target, voxelsize = 0.001, visualize = False, printing=False):
    source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxelsize, source, target)

    result_ransac = execute_global_registration(source_down, target_down,
                                                source_fpfh, target_fpfh,
                                                voxelsize)
    if printing:
        print(result_ransac)
    if visualize:
        draw_registration_result(source_down, target_down, result_ransac.transformation)
    return result_ransac.transformation
    
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

def preprocess_point_cloud(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 8
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(voxel_size, source, target):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def execute_fast_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))
    return result

def crop_pointclouds(pc, min, max):
    min_bound = np.array(min)  # minimum x, y, z values for cropping
    max_bound = np.array(max)  # maximum x, y, z values for cropping
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    pc_cropped = copy.deepcopy(pc)
    pc_cropped = pc_cropped.crop(aabb)
    return pc_cropped

if __name__ == '__main__':
    perform_RANSAC(o3d.io.read_point_cloud('./pointclouds/pc_orbecc_20241031_1408.pcd'), 
           o3d.io.read_point_cloud('./pointclouds/pc_target_20241031_1416.pcd'),
           0.01, [-0.5, -0.2, -0.2], [0.5, 4, 1])
