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
 
   # TODO: Convert ROS msg to PCL data, do this by using the pcl_helper functions:
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering (http://pointclouds.org/documentation/tutorials/statistical_outlier.php)
    #Create a filter object
    outlier_filter = cloud.make_statistical_outlier_filter()
    #set neighboring point to analyze, we will use 10 and then change if needed
    outlier_filter.set_mean_k(5)

    #set your multiplier: here if there is a mean distance larger than mean distance + x + std_dev we make it an outlier
    outlier_filter.set_std_dev_mul_thresh(.01)
    #now call the filter to your object
    cloud = outlier_filter.filter()
    # TODO: Voxel Grid Downsampling (http://pointclouds.org/documentation/tutorials/voxel_grid.php)
    #create a voxel grid filter object
    vox = cloud.make_voxel_grid_filter()
    #define leaf size and set the leave/voxel size
    LEAF_SIZE = .005
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE, LEAF_SIZE) #3Dimensional
    #call on filter and save pcl file.
    cloud_filtered = vox.filter()

 # TODO: PassThrough Filter
    # Create a PassThrough filter object for z.
    # Assign axis and range to the passthrough filter object for z
    passthrough_z = cloud_filtered.make_passthrough_filter()
    passthrough_z.set_filter_field_name('z')
    passthrough_z.set_filter_limits(0.6, 1.1)
    
    #now appliy the filter and save the point cloud
    cloud_filtered = passthrough_z.filter()
    


 # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    # Set the model you wish to fit 
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
        
    # Max distance .01 for a point to be considered fitting the model for plane (table segementation)
    seg.set_distance_threshold(.04)


    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    
    # Extract outliers 
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Extract inliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)


# TODO: Euclidean Clustering
    #take cloud objects and change to xyz using the helper code
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    #make a kd tree
    tree = white_cloud.make_kdtree()
    #create an extraction cluster object
    ec = white_cloud.make_EuclideanClusterExtraction()
    #set tolerance and min and max parameters
    ec.set_ClusterTolerance(.02)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(5000)
    #search tree for clusters
    ec.set_SearchMethod(tree)
    #extract the indices for each cluster
    cluster_indices = ec.Extract()
    
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
	for i, indice in enumerate(indices):
		color_cluster_point_list.append([white_cloud[indice][0], 
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
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices): 
        # Grab the points for the cluster	
	pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)
        # Compute the associated feature vector
        
	chists = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label) 

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

       	# Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)
    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)



if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous = True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size =1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size =1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size =1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size =1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size =1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size =1)

    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']
    

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
	rospy.spin()