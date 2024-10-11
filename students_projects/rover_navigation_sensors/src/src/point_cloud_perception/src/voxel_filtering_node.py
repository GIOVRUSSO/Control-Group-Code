#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from utils import convert_pointcloud2_to_numpy, convert_numpy_to_pointcloud2

"""
VoxelFilteringNode: This class consist of a ROS node which is responsible for voxel filtering the point cloud data.
"""
class VoxelFilteringNode():
    """
    This method initializes the VoxelFilteringNode.
    """
    def __init__(self):
        rospy.init_node('voxel_filtering_node')
        
        rospy.set_param('input_topic', "/husky2/points")
        rospy.set_param('output_topic', "/voxelized_points")
        
        self._input_topic = rospy.get_param('input_topic')
        self._output_topic = rospy.get_param('output_topic')
        
        
        self.voxelized_pub = rospy.Publisher(self._output_topic, PointCloud2, queue_size = 10)
        self.input_sub = rospy.Subscriber(self._input_topic, PointCloud2, self.voxel_filtering_callback, queue_size = 10)

    """
    This method applies the VoxelGrid filter to the point cloud data.
    
    :param points: The point cloud data as a numpy array.
    :param leaf_size: The leaf size for the VoxelGrid filter.
    
    :return: The filtered points as a numpy array.
    """
    def voxel_grid_filter(self, points, leaf_size=0.05):
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        voxel_size = leaf_size

        # Compute the voxel grid dimensions
        voxel_grid_dims = np.ceil((max_bound - min_bound) / voxel_size).astype(int)

        # Quantize the points to voxel grid
        voxel_indices = np.floor((points - min_bound) / voxel_size).astype(int)
        unique_voxel_indices, inverse_indices = np.unique(voxel_indices, axis=0, return_inverse=True)

        # Compute the voxel centroids
        voxel_centroids = np.zeros((unique_voxel_indices.shape[0], 3))
        for i in range(unique_voxel_indices.shape[0]):
            voxel_centroids[i] = np.mean(points[inverse_indices == i], axis=0)
        
        return voxel_centroids

    
    """
    This method is a callback function that is called when a new message is received on the /husky2/points topic.
    It applies the VoxelGrid filter to the point cloud data and publishes the filtered point cloud.
    
    :param point_cloud_msg: The PointCloud2 message received on the /husky2/points topic.
    """
    def voxel_filtering_callback(self, point_cloud_msg):
        #rospy.loginfo("Received PointCloud2 message for voxel filtering")

        # Convert PointCloud2 message to numpy array
        points = convert_pointcloud2_to_numpy(point_cloud_msg)

        # Apply VoxelGrid filter
        filtered_points = self.voxel_grid_filter(points)

        #rospy.loginfo("Voxel Grid Filtered PointCloud2 message")

        # Create a PointCloud2 message for filtered points
        header = point_cloud_msg.header
        voxelized_cloud_msg = convert_numpy_to_pointcloud2(filtered_points, header)

        # Publish the voxelized point cloud
        self.voxelized_pub.publish(voxelized_cloud_msg)

def main(args=None):
    node = VoxelFilteringNode()
    rospy.spin()

if __name__ == '__main__':
    main()
