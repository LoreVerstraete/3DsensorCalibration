import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import tf2_ros
import numpy as np
import os
import copy
import tf
import tf2_py as tf2
import yaml
from tf.transformations import quaternion_matrix
from scipy.spatial.transform import Rotation as R
from datetime import datetime

class Extract:
    def __init__(self, yamlfile, simulation=False, save=False):
        self.simulation = simulation
        self.save = save

        with open(yamlfile, 'r') as file:
            self.sensor = yaml.safe_load(file)

        for sensor in self.sensor.keys():
            self.sensor[sensor]['transform?'] = False
            self.sensor[sensor]['child_found'] = False
            if self.sensor[sensor]['color'] == 'blue':
                self.sensor[sensor]['colorcode'] = [0, 0, 1]
            elif self.sensor[sensor]['color'] == 'red':
                self.sensor[sensor]['colorcode'] = [1, 0, 0]
            elif self.sensor[sensor]['color'] == 'green':
                self.sensor[sensor]['colorcode'] = [0, 1, 0]
            elif self.sensor[sensor]['color'] == 'purple':
                self.sensor[sensor]['colorcode'] = [1, 0, 1]
            else:
                print('pointcloud not colored, use blue/red/green/purple or extend code')
            
        self.pointcloud_init = {}
        for sensor in self.sensor.keys():
            self.pointcloud_init[sensor] = False

        # Initialize ROS node
        rospy.init_node('pointcloud_extractor', anonymous=True)

        # Initialize tf2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Subscribe to the point cloud topics
        for sensor in self.sensor.keys():
            rospy.Subscriber(self.sensor[sensor]['topic'], PointCloud2, lambda msg, sensor=sensor: self.callback(msg, sensor))

        print("Subscribed to topics")

        rospy.spin()

    def callback(self, msg, sensor):
        """Callback for the topic."""
        if self.pointcloud_init[sensor] == False:
            self.sensor[sensor]['frame'] = msg.header.frame_id
            if self.sensor[sensor]['child_found'] == False:
                self.sensor[sensor]['child'], self.sensor[sensor]['child_found']= self.find_child_frame(self.sensor[sensor]['frame'], self.sensor[sensor]['parent'])
                # print(self.sensor[sensor]['child'])

            # print(self.sensor[sensor]['frame'])
            self.sensor[sensor]['pc_init'] = self.pointcloud2_to_pcd(msg)

            try:
                self.sensor[sensor]['init_guess'] = transform_matrix_from_rpy(self.sensor[sensor]['guess'])
            except Exception as e:
                self.sensor[sensor]['init_guess'] = np.eye(4)

            self.sensor[sensor]['pc_init'].paint_uniform_color(self.sensor[sensor]['colorcode'])
            self.pointcloud_init[sensor] = True

        if self.sensor[sensor]['transform?'] == False:
            self.sensor[sensor]['transform?'] = self.check_for_transform(sensor)
            if self.sensor[sensor]['transform?']:
                self.sensor[sensor]['pc_parent'] = copy_pointcloud(self.sensor[sensor]['pc_init'])
                self.sensor[sensor]['pc_child'] = copy_pointcloud(self.sensor[sensor]['pc_init'])
                self.sensor[sensor]['pc_base'] = copy_pointcloud(self.sensor[sensor]['pc_init'])
                self.sensor[sensor]['pc_parent'].transform(self.sensor[sensor]['transforminitparent'])
                self.sensor[sensor]['pc_child'].transform(self.sensor[sensor]['transforminitchild'])
                self.sensor[sensor]['pc_base'].transform(self.sensor[sensor]['transforminitbase'])
                try:
                    if self.sensor[sensor]['crop'] is not None:
                        self.sensor[sensor]['pc_base_crop'] = copy_pointcloud(self.sensor[sensor]['pc_base'])
                        self.sensor[sensor]['pc_base_crop']= crop_pointcloud(self.sensor[sensor]['pc_base_crop'], self.sensor[sensor]['crop'])
                        self.sensor[sensor]['pc_parent_crop']= copy_pointcloud(self.sensor[sensor]['pc_base_crop'])
                        self.sensor[sensor]['pc_parent_crop'].transform(self.sensor[sensor]['transformbaseparent'])
                except:
                    pass

            if self.save:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M")
                name = f'./pointclouds/pc_{sensor}_init_{timestamp}.pcd'
                o3d.io.write_point_cloud(name, self.sensor[sensor]['pc_init'])

        self.check_all_transformed()

    

            

    def pointcloud2_to_pcd(self, pointcloud_msg):
        """Convert sensor_msgs/PointCloud2 to XYZ point cloud (numpy array)."""
        points_list = []
        for point in pc2.read_points(pointcloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(points_list))
        return pcd  

    def check_for_transform(self, sensor):
        # Only proceed if frames are available
        try:
            # Transforms to visualise different settings
            # - frame towards parentframe
            # - frame towards childrame
            # - frame towards baseframe
            # - baseframe towards parentframe
            # - parentframe towards baseframe
            # - childframe towards parentframe

            if self.simulation:
                tansform_initparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'] , self.sensor[sensor]['frame'], rospy.Time.now(), rospy.Duration(1.0))
                tansform_initchild = self.tf_buffer.lookup_transform(self.sensor[sensor]['child'] , self.sensor[sensor]['frame'] ,rospy.Time.now(), rospy.Duration(1.0))
                tansform_initbase = self.tf_buffer.lookup_transform('base_footprint' , self.sensor[sensor]['frame'], rospy.Time.now(), rospy.Duration(1.0))
                tansform_baseparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'] ,'base_footprint', rospy.Time.now(), rospy.Duration(1.0))
                tansform_parentbase = self.tf_buffer.lookup_transform('base_footprint', self.sensor[sensor]['parent'] ,rospy.Time.now(), rospy.Duration(1.0))
                tansform_childparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'], self.sensor[sensor]['child'] ,rospy.Time.now(), rospy.Duration(1.0))
                tansform_parentchild = self.tf_buffer.lookup_transform(self.sensor[sensor]['child'], self.sensor[sensor]['parent'] ,rospy.Time.now(), rospy.Duration(1.0))

            else:
                tansform_initparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'] , self.sensor[sensor]['frame'], rospy.Time(0), rospy.Duration(1.0))
                tansform_initchild = self.tf_buffer.lookup_transform(self.sensor[sensor]['child'] , self.sensor[sensor]['frame'] ,rospy.Time(0), rospy.Duration(1.0))
                tansform_initbase = self.tf_buffer.lookup_transform('base_footprint' , self.sensor[sensor]['frame'], rospy.Time(0), rospy.Duration(1.0))
                tansform_baseparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'] , 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
                tansform_parentbase = self.tf_buffer.lookup_transform('base_footprint', self.sensor[sensor]['parent'] ,rospy.Time(0), rospy.Duration(1.0))
                tansform_childparent = self.tf_buffer.lookup_transform(self.sensor[sensor]['parent'], self.sensor[sensor]['child'] ,rospy.Time(0), rospy.Duration(1.0))
                tansform_parentchild = self.tf_buffer.lookup_transform(self.sensor[sensor]['child'], self.sensor[sensor]['parent'] ,rospy.Time(0), rospy.Duration(1.0))


            # Convert the transform into a 4x4 matrix
            self.sensor[sensor]['transforminitparent'] = transform_matrix_from_quaternion(tansform_initparent.transform.translation, tansform_initparent.transform.rotation)
            self.sensor[sensor]['transforminitchild'] = transform_matrix_from_quaternion(tansform_initchild.transform.translation, tansform_initchild.transform.rotation)
            self.sensor[sensor]['transforminitbase'] = transform_matrix_from_quaternion(tansform_initbase.transform.translation, tansform_initbase.transform.rotation)
            self.sensor[sensor]['transformbaseparent'] = transform_matrix_from_quaternion(tansform_baseparent.transform.translation, tansform_baseparent.transform.rotation)
            self.sensor[sensor]['transformparentbase'] = transform_matrix_from_quaternion(tansform_parentbase.transform.translation, tansform_parentbase.transform.rotation)
            self.sensor[sensor]['transformchildparent'] = transform_matrix_from_quaternion(tansform_childparent.transform.translation, tansform_childparent.transform.rotation)
            self.sensor[sensor]['transformparentchild'] = transform_matrix_from_quaternion(tansform_parentchild.transform.translation, tansform_parentchild.transform.rotation)

            print(f'Transformations of {sensor} extracted')
            return True # Indicate that a transform was found
        except (tf2.LookupException, tf2.ExtrapolationException) as e:
            # If needed print the exceptions (mostly due to a incomplete tf frame or not having synchronized clocks)
            # rospy.logwarn(f"Transform not found: {e}")
            return False  # No transform found
    
    def check_all_transformed(self):
        # If all transformations are extracted, close the subscriber
        for sensor in self.sensor.keys():
            if self.sensor[sensor]['transform?'] == False:
                return
        rospy.signal_shutdown('')

    def find_child_frame(self, initial_frame, parent_frame):
        reached_frame = initial_frame
        child_found = False

        # Check each frame in the buffer
        while child_found == False:
            for frame in self.tf_buffer.all_frames_as_string().splitlines():
                search_frame = frame.split(" ")[1]  # Extract the frame name -child-
                if search_frame == reached_frame: 
                    reached_frame = frame.split(" ")[-1].rstrip(".")  # Extract the frame name -parent-
                    if reached_frame == parent_frame:
                        child_found = True
                        child_frame = search_frame
                        break
    
        return child_frame, child_found

def transform_matrix_from_quaternion(translation, rotation):
    # Convert quaternion to rotation matrix (3x3 part of the 4x4 matrix)
    quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
    transformation_matrix = quaternion_matrix(quaternion)

    # Set the translation (the last column of the 4x4 matrix)
    transformation_matrix[0][3] = translation.x
    transformation_matrix[1][3] = translation.y
    transformation_matrix[2][3] = translation.z

    return transformation_matrix 
    

def crop_pointcloud(pointcloud, crop):
    # crop the point clouds
    min_bound = np.array([crop['min']['x'], crop['min']['y'], crop['min']['z']])
    max_bound = np.array([crop['max']['x'], crop['max']['y'], crop['max']['z']])
    aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    pointcloud_cropped = pointcloud.crop(aabb)
    return pointcloud_cropped

def copy_pointcloud(pc):
    # copy point cloud to let the other point cloud untouched
    new_pc = o3d.geometry.PointCloud()
    new_pc.points = o3d.utility.Vector3dVector(np.asarray(pc.points))
    if pc.has_colors():
        new_pc.colors = o3d.utility.Vector3dVector(np.asarray(pc.colors))
    if pc.has_normals():
        new_pc.normals = o3d.utility.Vector3dVector(np.asarray(pc.normals))
    return new_pc

def transform_matrix_from_rpy(transform):
    transformation_matrix = np.eye(4)

    # Extract the rotation matrix
    r = R.from_euler('xyz', [transform['roll'], transform['pitch'], transform['yaw']], degrees=False)
    transformation_matrix[:3,:3] = r.as_matrix()

    # Set the translation (the last column of the 4x4 matrix)
    transformation_matrix[0][3] = transform['x']
    transformation_matrix[1][3] = transform['y']
    transformation_matrix[2][3] = transform['z']

    return transformation_matrix

if __name__ == '__main__':
    data = Extract('./sensors.yaml', simulation=True, save=False)
    
    for sensor in data.sensor.keys():
        o3d.visualization.draw_geometries([data.sensor[sensor]['pc_parent']])