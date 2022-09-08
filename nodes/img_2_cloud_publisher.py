#!/usr/bin/env python3
import open3d as o3d
import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError

import stereo_vision.pcd_converter as pcd_converter
from stereo_vision.utils_stereovision import point_cloud_from_stereo


def callback(data):
    try:
        """
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        o3d_pcd = point_cloud_from_stereo(frame)
        ros_pcd = pcd_converter.convert_open3d_to_ros(o3d_pcd)
        pub.publish(ros_pcd)
        print("[INFO] Point cloud published")
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.random.rand(100, 3))
        pcd.paint_uniform_color([1, 0.706, 0])
        ros_pcd = pcd_converter.convert_open3d_to_ros(pcd)
        pub.publish(ros_pcd)
        print("[INFO] Point cloud published")
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    # Create publisher and Initialize the node with rospy
    rospy.init_node('cloud_publisher_node')
    bridge = CvBridge()
    pub = rospy.Publisher('cloud_stream', numpy_msg(PointCloud2), queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('arducam_stream', Image, callback)
        rate.sleep()

# rospy.spin() is needed ?
