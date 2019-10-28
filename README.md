# vision-guidede-robot-pick-and-place

# target 
with the helps of abundant information of RGB images and 3D information from depth image / point cloud, 
the pre-defined model can be regconize by its edge features, and whose 3D point cloud can be segmented. After 
point cloud matching/registering with initial pose feed, the 3d pose related to pre-defined model is estimated.
Then, transfer the pose to robot coordinate to guide the robot.

code type: c++

code structure: please see "structure.PNG"

# procedures

* image pre-process procedures
1. capture both mono/rgb & depth/range images
2. register mono image to depth image ,or verse, to make them matched pixel-by-pixel
3. project depth map to point cloud with camera matrix if this is required

* calibration procedures
bind camera coordinate to robot coordinate

* model definition
1. select model region in rgb image, which is feed to define the 2d features
2. with the edits of ground height, define the 3d features without noise depth affect
3. check the error of pose estimation (if error is too high, redo 1 to 3) 

* scene localization
1. find 2d features
2. segment object with pre-defined mask
3. performance point cloud alignment
4. guide robot with 3d pose
