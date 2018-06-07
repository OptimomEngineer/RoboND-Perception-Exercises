 # Voxel Grid filter
#create a voxel grid filter object
vox = cloud.make_voxel_grid_filter()
#define leaf size and set the leave/voxel size
LEAF_SIZE = .005
vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE, LEAF_SIZE) #3Dimensional
#call on filter and save pcl file.
cloud_filtered = vox.filter()
     
 
 
 # PassThrough filter

# Create a PassThrough filter object for z.
# Assign axis and range to the passthrough filter object for z
passthrough = cloud_filtered.make_passthrough_filter()
passthrough.set_filter_field_name('z')
passthrough.set_filter_limits(.6, 1.1)
#now appliy the filter and save the point cloud
cloud_filtered = passthrough.filter()
 
 # RANSAC plane segmentation
# TODO: RANSAC Plane Segmentation to remove the table
seg = cloud_filtered.make_segmenter()
# Set the model you wish to fit which is a plane mode in our case
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
# Max distance .01 for a point to be considered fitting the model for plane (table segementation)
# i tried different ones but ended up using .01
seg.set_distance_threshold(.01)
 
 
 # Extract inliers

 # TODO: Extract inliers and outliers
inliers, coefficients = seg.segment()
# Extract outliers 
cloud_objects = cloud_filtered.extract(inliers, negative=True)
# Extract inliers
cloud_table = cloud_filtered.extract(inliers, negative=False)
# Save pcd for table
# pcl.save(cloud, filename)
filename = 'cloud_objects.pcd'
pcl.save(cloud_objects, filename)
# Save pcd for tabletop objects
filename = 'cloud_table.pcd'
pcl.save(cloud_table, filename)
 
 
# Extract outliers
 
 
# Save pcd for tabletop objects
 

