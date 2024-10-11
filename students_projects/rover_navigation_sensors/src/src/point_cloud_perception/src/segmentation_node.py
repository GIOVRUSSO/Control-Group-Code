#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import numpy as np
from sklearn.linear_model import RANSACRegressor

from utils import convert_pointcloud2_to_numpy, convert_numpy_to_pointcloud2

"""
SegmentationNode: This class consist of a ROS node which is responsible for segmenting the plane points in the point cloud data.
"""
class SegmentationNode():
    """
    This method initializes the SegmentationNode.
    """
    def __init__(self):
        rospy.init_node('segmentation_node')
        
        rospy.set_param('input_topic', "/voxelized_points")
        rospy.set_param('output_plane_topic', "/plane_points")
        rospy.set_param('output_non_plane_topic', "/non_plane_points")
        
        
        self._output_plane_topic = rospy.get_param('output_plane_topic')
        self._output_non_plane_topic = rospy.get_param('output_non_plane_topic')
        
        self._input_topic = rospy.get_param('input_topic')
        
        self.non_plane_pub = rospy.Publisher(self._output_non_plane_topic, PointCloud2, queue_size = 10)
        self.plane_pub = rospy.Publisher(self._output_plane_topic, PointCloud2, queue_size = 10)
        
        self.input_sub = rospy.Subscriber(self._input_topic, PointCloud2, self.segmentation_callback, queue_size = 10)

    """
    This method segments the plane points in the point cloud data using RANSAC.
    
    :param points: The point cloud data as a numpy array.
    :param distance_threshold: The distance threshold for RANSAC.
    
    :return: The non-plane points and the plane points as numpy arrays.
    """
    def segment_plane(self, points, distance_threshold=0.18):
        # Fit the plane using RANSAC
        ransac = RANSACRegressor(residual_threshold=distance_threshold)
        ransac.fit(points[:, :2], points[:, 2])

        # Get the inliers and outliers
        inliers = ransac.inlier_mask_
        outliers = np.logical_not(inliers)

        return points[outliers], points[inliers]

    
    """
    This method is a callback function that is called when a new message is received on the /voxelized_points topic.
    It segments the plane points in the point cloud data and publishes the segmented plane and non-plane points.
    
    :param point_cloud_msg: The PointCloud2 message received on the /voxelized_points topic.
    """
    def segmentation_callback(self, point_cloud_msg):
        #rospy.loginfo("Received PointCloud2 message for segmentation")

        # Convert PointCloud2 message to numpy array
        points = convert_pointcloud2_to_numpy(point_cloud_msg)

        # Segment the plane
        non_plane_points, plane_points = self.segment_plane(points)

        #rospy.loginfo("Plane Segmented PointCloud2 message")

        # Create a PointCloud2 message for segmented plane points
        header = point_cloud_msg.header
        non_plane_cloud_msg = convert_numpy_to_pointcloud2(non_plane_points, header)
        plane_cloud_msg = convert_numpy_to_pointcloud2(plane_points, header)

        # Publish the segmented plane point cloud
        self.plane_pub.publish(plane_cloud_msg)
        self.non_plane_pub.publish(non_plane_cloud_msg)

def main():
    segmentation_node = SegmentationNode()
    rospy.spin()

if __name__ == '__main__':
    main()