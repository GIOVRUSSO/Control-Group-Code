#!/usr/bin/env python3

import rospy
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2
import numpy as np
import matplotlib.cm as cm
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

from utils import convert_pointcloud2_to_numpy, convert_numpy_to_rgb_pointcloud2, get_transform


"""
PointClusterNode: This class consist of a ROS node which is responsible for clustering the non-plane points in the point cloud data.
"""
class PointsClusterNode():
    
    """
    This method initializes the PointsClusterNode.
    It sets the input_topic parameter to '/non_plane_points', the output_topic parameter to '/clustered_points', the obstacle_centroids_topic parameter to '/obstacle_cluster_centroids', and the robot_namespace parameter to 'husky2'.
    It initializes the _num_clusters and _points_per_cluster variables to empty dictionaries.
    It initializes the tf_prefix variable 'husky2_tf'
    """
    def __init__(self):
        rospy.init_node('points_cluster_node')
        rospy.set_param('input_topic', "/non_plane_points")
        rospy.set_param('output_topic', "/clustered_points")
        rospy.set_param('obstacle_centroids_topic', "/obstacle_cluster_centroids")
        rospy.set_param('robot_namespace', "husky2")
        
        self._num_clusters = {}
        self._points_per_cluster = 0
        
        self._namespace = rospy.get_param('robot_namespace')
        self._tf_prefix = self._namespace + "_tf"
        
        self._input_topic = rospy.get_param('input_topic')
        self._output_topic = rospy.get_param('output_topic')
        self.obstacle_centroids_topic = rospy.get_param('obstacle_centroids_topic')
        
        self.clustered_pub = rospy.Publisher(self._output_topic, PointCloud2, queue_size=1)
        self.obstacle_centroids_pub = rospy.Publisher(self.obstacle_centroids_topic, MarkerArray, queue_size=1)
        self.input_sub = rospy.Subscriber(self._input_topic, PointCloud2, self._cluster_points_callback, queue_size=1)

        self._message_count = 0
        
        self._sensor_frame = self._tf_prefix + "/velodyne"
        self._reference_frame = self._tf_prefix + "/map"

    """
    This method publishes the centroids of the obstacles as MarkerArray.
    
    :param centroids: A dictionary containing the cluster_id as the key and the centroid of the cluster as the value.
    :param header: The header of the PointCloud2 message.
    """
    def publish_centroids_as_markers(self, centroids, header):
        
        marker_array = MarkerArray()
        marker = Marker()
        #marker.action = Marker.DELETEALL
    
        velodyne_to_map_transform = get_transform(self._sensor_frame, self._reference_frame)
        if velodyne_to_map_transform is None:
            rospy.logerr("Failed to get transform from velodyne to", self._reference_frame)
            return

        for cluster_id, centroid in centroids.items():
            obs_pose = PoseStamped()
            
            obs_pose.pose.position.x = centroid[0]
            obs_pose.pose.position.y = centroid[1]
            obs_pose.pose.position.z = centroid[2]
            obs_pose.pose.orientation.x = 0.0
            obs_pose.pose.orientation.y = 0.0
            obs_pose.pose.orientation.z = 0.0
            obs_pose.pose.orientation.w = 1.0

            obs_pose.header = header
            obs_pose.header.frame_id = self._sensor_frame        
            transformed_pose = obs_pose
                
            marker = Marker()
            marker.header = header
            marker.ns = "centroids"
            marker.id = cluster_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = transformed_pose.pose
            marker.scale.x = 0.1  # Diameter of the sphere
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Don't forget to set the alpha!
            marker.color.r = 1.0  # Red color
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker_array.markers.append(marker)

        self.obstacle_centroids_pub.publish(marker_array)

    """
    This method is a callback function that is called when a new message is received on the /non_plane_points topic.
    It clusters the points in the point cloud data using DBSCAN clustering algorithm.
    It publishes the clustered point cloud data on the /clustered_points topic.
    
    :param point_cloud_msg: The PointCloud2 message received on the /non_plane_points topic.
    """
    def _cluster_points_callback(self, point_cloud_msg):
        self._message_count += 1
        if self._message_count % 3 != 0 and not self._message_count == 1:
            return
        self._message_count = 0
        #rospy.loginfo("Received PointCloud2 message for clustering")

        # Convert PointCloud2 message to numpy array
        points = convert_pointcloud2_to_numpy(point_cloud_msg)
        
        if points is None:
            return

        # Perform DBSCAN clustering
        clustered_points, num_clusters, points_per_cluster, centroids = self.cluster_points(points)
        self._num_clusters = num_clusters
        self._points_per_cluster = points_per_cluster

        # rospy.loginfo("Clustered PointCloud2 message")
        # rospy.loginfo(f"Number of clusters: {self._num_clusters}")
        # rospy.loginfo(f"Points per cluster: {self._points_per_cluster}")

        # Create a PointCloud2 message for clustered points
        header = point_cloud_msg.header
        header.frame_id = self._sensor_frame
        clustered_cloud_msg = convert_numpy_to_rgb_pointcloud2(clustered_points, header)

        # Publish the clustered point cloud
        self.clustered_pub.publish(clustered_cloud_msg)
        
        # Publish centroids as MarkerArray
        self.publish_centroids_as_markers(centroids, header)

    
    """
    This method clusters the points in the point cloud data using DBSCAN clustering algorithm.
    
    :param points: The numpy array containing the point cloud data.
    :param eps: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
    :param min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
    
    :return clustered_points: The numpy array containing the clustered point cloud data.
    """
    def cluster_points(self, points, eps=0.45, min_samples=20):
        
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        clusters = dbscan.fit_predict(points)
        unique_clusters = np.unique(clusters)
        
        num_clusters = len(unique_clusters)
        points_per_cluster = {}
        centroids = {}
        
        # Define a structured dtype
        dtype = np.dtype([
            ('x', np.float32), 
            ('y', np.float32), 
            ('z', np.float32), 
            ('r', np.uint8), 
            ('g', np.uint8), 
            ('b', np.uint8)
        ])
        
        clustered_points = np.empty(len(points), dtype=dtype)
        colors = cm.get_cmap('hsv', num_clusters)  # Generate distinct colors for each cluster
        
        idx = 0
        for cluster_id in unique_clusters:
            cluster_points = points[clusters == cluster_id]
            num_points = len(cluster_points)
            points_per_cluster[cluster_id] = num_points
            
            # Calculate centroid
            centroid = np.mean(cluster_points[:, :3], axis=0)
            centroids[cluster_id] = centroid
            
            
            # Assign colors
            color = (np.array(colors(cluster_id % num_clusters)[:3]) * 255).astype(np.uint8)  # Get RGB values as uint8

            for point in cluster_points:
                clustered_points[idx] = (*point, color[0], color[1], color[2])  # Create tuple (x, y, z, r, g, b)
                idx += 1
                
        return clustered_points, num_clusters, points_per_cluster, centroids

def main():
    
    points_cluster_node = PointsClusterNode()
    rospy.spin()

if __name__ == '__main__':
    main()
    
    

