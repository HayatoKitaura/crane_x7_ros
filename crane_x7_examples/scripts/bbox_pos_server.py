#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from crane_x7_examples.srv import bbox_pos, bbox_posResponse

class BBoxCenterServer():
    def __init__(self):
        rospy.init_node('bbox_server', anonymous=True)
        rospy.Service('bbox_service', bbox_pos, self.Callback_srv)
        self.x = 0
        self.y = 0

    def DarknetBboxCallback(self, darknet_bboxs):
        self.bbox = darknet_bboxs.bounding_boxes[0]
        bbox = self.bbox
        self.x = (bbox.xmax + bbox.xmin)/2
        self.y = (bbox.ymax + bbox.ymin)/2

    def Callback_srv(self, data):
        resp = bbox_posResponse()
        if data.data == True:
            rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBboxCallback)
            resp.x_center = self.x
            resp.y_center = self.y
            return resp

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            BBoxCenterServer()
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
