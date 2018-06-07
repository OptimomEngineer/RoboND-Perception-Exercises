#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Voxel Grid Downsampling
    #first create a gridfilter obect
    vox = cloud.make_voxel_grid_filter()
    #choose the leaf size and set the leaf size
    LEAF_SIZE = .01 
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    #now use the filter function to create a filtered version of the cloud that has been downsampled
    cloud_filtered = vox.filter()
    filename = 'voxel_downsampled.pcd'
    pcl.save(cloud_filtered, filename)
    # TODO: PassThrough Filter
    #first make the filter object
    passthrough = cloud_filtered.make_passthrough_filter()
    
    #set the axis and the range for the passthrough filter object
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = .6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    
    #now appliy the filter and save the point cloud
    cloud_filtered = passthrough.filter()
    filename = 'pass_through_filtered.pcd'
    pcl.save(cloud_filtered, filename)

    # TODO: RANSAC Plane Segmentation
    # Create the segmentation object
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    
    # Max distance for a point to be considered fitting the model
    # Experiment with different values for max_distance 
    # for segmenting the table
    max_distance = .01
    seg.set_distance_threshold(max_distance)
    
    # Call the segment function to obtain set of inlier indices and model     coefficients
    inliers, coefficients = seg.segment()
    
    # TODO: Extract inliers and outliers
    # Extract inliers

    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    filename = 'cloud_objects.pcd'
    pcl.save(cloud_objects, filename)


    # Save pcd for tabletop objects
    # Extract outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    filename = 'cloud_table.pcd'
    pcl.save(cloud_table, filename)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    #perform cluster extraction
    ec = white_cloud.make_EuclideanClusterExtraction()
    #set tolerance and min and max parameters
    ec.set_ClusterTolerance(.00001)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(50)
    ec.set_SearchMethod(tree)
    #extract the indices
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #assign the colors to each segmented object
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
	for i, indice in enumerate(indices):
		color_cluster_point_list.appren([white_cloud[indice][0], 
						white_cloud[indice][1],
						white_cloud[indice][2],
						rgb_to_float(cluster_color[j])])
    #create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size =1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size =1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size =1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size =1)
    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
	rospy.spin()