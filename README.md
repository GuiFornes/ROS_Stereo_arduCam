# ROS_Stereo_arduCam

Ros package to process stereo-vision and transmit point cloud.  
It makes part of a project for CoRo lab at ETS Montreal.

# Dependencies
  * ROS
  * OpenCV
  * cv_bridge
  * sensor_msgs
  * std_msgs
  * ArducamSDK

# Usage
_Currently there are still problems with the conversion of a colored o3d point cloud to a ROS point cloud._
  * Clone the repository in your catkin workspace
  * Build the package
  * Run the `arduCam_publisher.py` node to launch the camera
  * You can run `image_subscriber_node.py` to visualize source images from stereo cam
  * Then you can run `img_2_cloud_publisher` to process the images and publish the point cloud
  * At least run `depth_publisher_node.py` to publish the coordinates of the closest point of the cloud

# Authors
  * Guillaume FORNES