#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2

def bbox_callback(bbox_data):
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print("aaaaaaaa")


def main():
    rospy.init_node('bbox_sub', anonymous=True)
    while True:
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBox, bbox_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
