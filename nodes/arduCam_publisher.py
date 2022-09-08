#!/usr/bin/env python3

import os
import time
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import stereo_vision.utils_arducam as arducam


def main():
    # Initialisation rospy node, publisher and converter
    rospy.init_node('arduCam_publisher')
    pub = rospy.Publisher('arducam_stream', Image, queue_size=1)
    bridge = CvBridge()
    rate = rospy.Rate(5)

    # Initialisation of the camera
    ret_val = arducam.init()
    if ret_val:
        print("[ERROR] Failed to initialize the camera")
        exit()

    # Variables declaration
    time0 = time.time()
    count = 0

    # Capturing loop
    while not rospy.is_shutdown():

        # Capture frame
        frame = arducam.getFrame()

        # Compute fps
        time1 = time.time()
        if time1 - time0 >= 1:
            print("[INFO] %s %d %s\n" % ("fps:", count, "/s"))
            count = 0
            time0 = time1
        count += 1

        # Save frame
        if arducam.save_flag:
            if not os.path.exists("../images"):
                os.makedirs("../images")
            cv2.imwrite("../images/frame.png", frame)
            arducam.save_flag = False
            print("[INFO] Image saved to images/frame.png")

        # Publish image
        msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        rate.sleep()

    # Close the camera and release the resources
    arducam.close()


if __name__ == "__main__":
    main()
    rospy.spin()
