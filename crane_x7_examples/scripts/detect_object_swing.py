#! /usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import math
import time
import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBoxes
from crane_x7_examples.srv import call_dso, call_dsoResponse
from crane_x7_examples.srv import bbox_pos, bbox_posResponse

def detect_swing_object():
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    # while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
    #     rospy.sleep(1.0)
    # rospy.sleep(1.0)

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

#探索
    arm.set_named_target("search")
    arm.go()

    #ハンドのデカルト座標
    t_x = 0.25
    t_y = 0

    #画像座標系の物体の中心座標
    pos_x = 0
    pos_y = 0

    #画像座標上での中心範囲
    # range_x_min = 300
    # range_x_max = 340
    # range_y_min = 220
    # range_y_max = 260
    range_x_min = 380
    range_x_max = 400
    range_y_min = 280
    range_y_max = 300


    while(pos_x < range_x_min or range_x_max < pos_x or pos_y < range_y_min or range_y_max < pos_y):

        # if(range_x_min < pos_x and pos_x < range_x_max and range_y_min < pos_y and pos_y < range_y_max): break

        rospy.wait_for_service('bbox_service')
        try:
            b_s = rospy.ServiceProxy('bbox_service', bbox_pos)
            resp = b_s(True)
            print(resp)
            print("------------")
            print("x:", t_x)
            print("y:", t_y)
            print("------------")
            pos_x = resp.x_center
            pos_y = resp.y_center
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        if(pos_x < range_x_min): t_y -= 0.01
        if(pos_x > range_x_max): t_y += 0.01
        if(pos_y < range_y_min): t_x -= 0.01
        if(pos_y > range_y_max): t_x += 0.01

        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = t_x
        target_pose.position.y = t_y
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行
        time.sleep(2.0)

    # 掴みに行く
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = t_x
    target_pose.position.y = t_y
    target_pose.position.z = 0.1
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを閉じる
    gripper.set_joint_value_target([0.01, 0.01]) #掴むobjectによって変更する
    gripper.go()

    # 持ち上げる
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.3
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()							# 実行

    arm.set_named_target("vertical")
    arm.go()

    data = [ [[1,20]] , [[1,10],[3,-60],[5,-30]], [[1,30],[3,1],[5,30]], [[1,10],[3,-60],[5,-30]], [[1,30],[3,1],[5,30]], [[1,10],[3,-60],[5,-30]], [[1,30],[3,1],[5,30]] ]

    num = 0
    arm_joint_values = arm.get_current_joint_values()
    for flame in range(len(data)):
        for joint_data in range(len(data[flame])):
            joint=int(data[flame][joint_data][0])
            angle = float(data[flame][joint_data][1])/180.0*math.pi
            print(joint, angle)
            arm_joint_values[joint] = angle

        num += 1
        print(num)

        if num %2==0: speed=0.3
        else: speed=0.9

        print("flame")
        print(speed)
        print(arm_joint_values)
        arm.set_max_velocity_scaling_factor(speed)
        arm.set_joint_value_target(arm_joint_values)
        arm.go()

    print("done")
    arm.set_named_target("home")
    arm.go()

def main(String):
    if String.data == True:
        detect_swing_object()
        resp = call_dsoResponse()
        resp.back = True
        # time.sleep(100)
        return resp

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            rospy.init_node("google_assistant_robot", anonymous=True)
            print('waiting')
            rospy.Service('detect_swing_object', call_dso, main)
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
