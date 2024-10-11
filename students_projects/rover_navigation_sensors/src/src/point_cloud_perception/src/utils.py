import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros

"""
This module contains utility functions for working with point cloud data.
"""

"""
This function converts a PointCloud2 message to a numpy array.

:param point_cloud_msg: The PointCloud2 message to be converted.

:return points_array: The numpy array containing the point cloud data.
"""
def convert_pointcloud2_to_numpy(point_cloud_msg):
    if not isinstance(point_cloud_msg, PointCloud2):
        rospy.logerr("Input message is not of type PointCloud2. Aborting...")
        return None
    
    points = []
    for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        if point[2] < 0.2:
            points.append([point[0], point[1], point[2]])
    
    points_array = np.array(points)
    
    
    if points_array.size == 0:
        #rospy.logerr("No points in the point cloud. Aborting...")
        return None
    
    if points_array.shape[1] != 3:
        #rospy.logerr("Converted points do not have 3 columns. Aborting...")
        return None

    return points_array

"""
This function converts a numpy array to a PointCloud2 message.

:param points: The numpy array containing the point cloud data.
:param header: The header of the PointCloud2 message.

:return point_cloud_msg: The PointCloud2 message.
"""
def convert_numpy_to_pointcloud2(points, header):
    if not isinstance(points, np.ndarray):
        rospy.logerr("Input points are not a numpy array. Aborting...")
        return None

    if points.shape[1] != 3:
        rospy.logerr("Input points do not have 3 columns. Aborting...")
        return None

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]

    return pc2.create_cloud(header, fields, points)

"""
This function converts a numpy array to a PointCloud2 message with RGB values.

:param points: The numpy array containing the point cloud data.
:param header: The header of the PointCloud2 message.

:return point_cloud_msg: The PointCloud2 message.
"""
def convert_numpy_to_rgb_pointcloud2(points, header):
    if not isinstance(points, np.ndarray):
        rospy.logerr("Input points are not a numpy array. Aborting...")
        return None

    required_fields = {'x', 'y', 'z', 'r', 'g', 'b'}
    if points.dtype.fields is None or not required_fields.issubset(points.dtype.fields):
        rospy.logerr("Input points do not have correct dtype. Aborting...")
        return None

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('r', 12, PointField.UINT8, 1),
        PointField('g', 13, PointField.UINT8, 1),
        PointField('b', 14, PointField.UINT8, 1),
    ]

    # Convert structured array to list of tuples
    points_list = [tuple(p) for p in points]

    return pc2.create_cloud(header, fields, points_list)


"""
This function gets the transformation between two frames.

:param source_frame: The source frame.
:param target_frame: The target frame.
:param tf_cache_duration: The duration for which the transformation is cached.

:return transformation: The transformation from source_frame to target_frame.
"""
def get_transform(source_frame, target_frame, tf_cache_duration=0.2):
    
    tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
    tf2_ros.TransformListener(tf_buffer)

    # get the transformation from source_frame to target_frame.
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(0.3))
        return transformation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                    % source_frame, target_frame)

    

        

    