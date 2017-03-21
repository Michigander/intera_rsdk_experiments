#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Image

import cv_bridge

class SimpleHeadDisplay(object):

    def __init__(self):
        """
        Constructor
        """

        self._image_pub = rospy.Publisher('/robot/head_display', Image, latch=True, queue_size=10)


    def display_image(self, img):
        """
        Publish a single image as image message
        """

        if not image_msg:
            rospy.logerr("Image message has no data!")

        else:
            cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
            self._image_pub.publish(img_msg)
            rospy.sleep()
