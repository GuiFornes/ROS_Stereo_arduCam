#!/usr/bin/env python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2


def callback(data):
    try:
        cv2.imshow("Image window", bridge.imgmsg_to_cv2(data, "bgr8"))
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            exit(0)  # Do not work because of the rospy.spin() loop
    except CvBridgeError as e:
        print(e)


if __name__ == '__main__':
    rospy.init_node('image_subscriber_node')
    bridge = CvBridge()
    rospy.Subscriber('arducam_stream', Image, callback)  # or arducam_stream
    rospy.spin()
