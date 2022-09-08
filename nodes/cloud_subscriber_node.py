#!/usr/bin/env python3
import open3d as o3d

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError

import stereo_vision.pcd_converter as pcd_converter

bridge = CvBridge()


def callback(data):
    try:
        o3d_pcd = pcd_converter.convert_ros_to_open3d(data)
        o3d.visualization.draw_geometries([o3d_pcd])
    except CvBridgeError as e:

        print(e)


if __name__ == '__main__':
    rospy.init_node('cloud_subscriber_node')
    rospy.Subscriber('cloud_stream', numpy_msg(PointCloud2), callback)
    rospy.spin()
