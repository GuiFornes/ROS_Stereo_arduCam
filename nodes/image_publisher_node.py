#!/usr/bin/env python3

import numpy
import cv2
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Initialize the node with rospy
rospy.init_node('image_publisher_node')
# Create publisher
pub = rospy.Publisher('cam_stream', Image, queue_size=1)
# image type converter
bridge = CvBridge()
# Initialize the camera
cap = cv2.VideoCapture(0)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # Read the image
    ret, frame = cap.read()
    if ret:
        # Convert to ROS image message
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        # Publish the image
        pub.publish(msg)
        # Sleep for 1 second
    rate.sleep()

# spin to keep the script for exiting
rospy.spin()
