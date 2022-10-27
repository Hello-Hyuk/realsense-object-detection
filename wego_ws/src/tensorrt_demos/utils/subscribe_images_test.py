#!/usr/bin/env python
#-*- coding: utf-8 -*-

# Python 2/3 compatibility
from __future__ import print_function

# External modules
import cv2

# ROS module
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

CV_BRIDGE = CvBridge()

# all topics are processed in this callback function
def callback(image):
    rospy.loginfo('image subscribe')

    # image callback function
    try:
        img = CV_BRIDGE.imgmsg_to_cv2(image, 'bgr8')
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {}".format(e))
    cv2.imshow("img",img)
    print(img.shape)

    height = img.shape[0]
    width = img.shape[1]
    cv2.waitKey()
    cv2.destroyAllWindows()

# practical main function
def listener(realsense_img):
    # Start node
    rospy.init_node('realsense_raw', anonymous=True)
    rospy.loginfo('Image topic: {}'.format(realsense_img))

    # Subscribe to topics
    rospy.Subscriber(realsense_img,Image,callback)
    # Keep python from exiting until this node is stopped
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('Shutting down')

if __name__ == '__main__':
    # YOLO
    realsense_img = '/camera/color/image_raw'

    # Start subscriber
    listener(realsense_img)