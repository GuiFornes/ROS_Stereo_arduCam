#!/usr/bin/env python3
import numpy as np
import open3d as o3d
import rospy
import stereo_vision.pcd_converter as pcd_converter
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, Image


def callback(data):
    try:
        o3d_pcd = pcd_converter.convert_ros_to_open3d(data)

        # TODO: Compute nearest depth
        # Finding nearest red object for example ?
        prox = o3d_pcd.get_min_bound()
        print(prox)
        o3d_pcd.paint_uniform_color([0.7, 0.7, 0.7])
        pts = np.asarray(o3d_pcd.points)
        clr = np.asarray(o3d_pcd.colors)
        np.append(pts, prox)
        np.append(clr, [1, 0, 0])
        o3d_pcd.points = o3d.utility.Vector3dVector(pts)
        o3d_pcd.colors = o3d.utility.Vector3dVector(clr)
        o3d.visualization.draw_geometries([o3d_pcd])

        # o3d_pcd.compute_point_cloud_distance()

        print("[INFO] prox depth published")
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    # Create publisher and Initialize the node with rospy
    rospy.init_node('depth_publisher_node')
    bridge = CvBridge()
    pub = rospy.Publisher('depth_stream', numpy_msg(PointCloud2), queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('cloud_stream', Image, callback)
        rate.sleep()

# rospy.spin() is needed ?
