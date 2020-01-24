#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from crane_x7_examples.srv import bbox_pos, bbox_posResponse

def DarknetBboxCallback(darknet_bboxs):
    bbox = darknet_bboxs.bounding_boxes[0]
    x = bbox.xmax - bbox.xmin
    y = bbox.ymax - bbox.xmin

def Callback_srv(data):
    resp = bbox_posResponse()
    if data.data == True:
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, DarknetBboxCallback)
        resp.x_center = x
        resp.y_center = y
        return resp

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            rospy.init_node('bbox_seaver', anonymous=True)
            rospy.Service('bbox_service', bbox_pos, Callback_srv)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
