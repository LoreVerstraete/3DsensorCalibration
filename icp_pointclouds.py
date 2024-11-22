import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from colorama import Fore, Style
from scipy.spatial.transform import Rotation 
from datetime import datetime

class ApplyICP:
    '''Apply the ICP method to transform the pointclouds towards each other'''
    def __init__(self, source, target, threshold=None, guess = np.eye(4), method = o3d.pipelines.registration.TransformationEstimationPointToPlane(), save=False):
        self.source = source
        self.target = target
        self.guess = guess

        # methods = [
        # o3d.pipelines.registration.TransformationEstimationPointToPoint(),    
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        # o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        
        # o3d.pipelines.registration.TransformationEstimationPointToPlane(o3d.pipelines.registration.TukeyLoss(k=0.1)),
        # ]
        self.method = method

        # Calculate icp treshold
        if threshold == None:
            avg_distance_target = self.compute_average_distance(self.target)
            avg_distance_source = self.compute_average_distance(self.source)
            self.threshold = 300*min(avg_distance_target, avg_distance_source)
        else:
            self.threshold = threshold

        # self.draw_pc(self.source + self.target)

        self.perform_icp()
        if save:
            self.save_pc()

    def draw_pc(self, pointcloud):
        """Visualize the registration result."""
        o3d.visualization.draw_geometries([pointcloud])
        
    def perform_icp(self):
        """Perform ICP registration."""
        self.target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # used for PointToPlane
        self.target.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # used for Generalized ICP
        self.source.estimate_covariances(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)) # used for Generalized ICP

        reg_icp = o3d.pipelines.registration.registration_icp(
            self.source, self.target, self.threshold,
            self.guess,
            self.method,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration = 100)
        )
        self.rmse = reg_icp.inlier_rmse
        if reg_icp.inlier_rmse == 0.0:
            self.transform_icp = np.eye(4)
            self.source_tr = self.source
            # print("DID NOT CONVERGE")
        else:
            # print("Inlier RMSE:", reg_icp.inlier_rmse)
            # print("Fitness:", reg_icp.fitness)
            # print("Transformation matrix:\n", reg_icp.transformation)
            self.transform_icp = reg_icp.transformation
            self.source_tr = self.source.transform(self.transform_icp)
            # self.draw_pc(self.source_tr + self.target)

    def compute_average_distance(self, pcd):
        """Compute the average distance between neighboring points."""
        distances = pcd.compute_nearest_neighbor_distance()
        avg_distance = np.mean(distances)
        return avg_distance

    def save_pc(self):
        # Save final point cloud
        if str(self.method) == str(o3d.pipelines.registration.TransformationEstimationPointToPoint()):
            short_method = 'ptpt'
        elif str(self.method) == str(o3d.pipelines.registration.TransformationEstimationPointToPlane()):
            short_method = 'ptpl'
        elif str(self.method) == str(o3d.pipelines.registration.TransformationEstimationForGeneralizedICP()):
            short_method = 'gen'
        else:
            short_method = 'other'
        now = datetime.now()
        o3d.io.write_point_cloud('./PointClouds/Result_'+short_method+'_'+str(now.strftime("%Y-%m-%d-%H-%M"))+'.ply',self.target + self.source_tr)