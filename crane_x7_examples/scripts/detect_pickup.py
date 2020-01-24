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
from crane_x7_examples.srv import bbox_pos, bbox_posResponse

def main():
    rospy.init_node("google_assistant_robot")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.3)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

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
    range_x_min = 290
    range_x_max = 350
    range_y_min = 210
    range_y_max = 270


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
        # time.sleep(3)

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
    gripper.set_joint_value_target([0.1, 0.1]) #掴むobjectによって変更する
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

    # with open('swing_object.csv') as f:   #csvファイルを読み込む
    #     reader = csv.reader(f)#, quoting=csv.QUOTE_NONNUMERIC)
    #     data = [row for row in reader]

    data = [[["a",1,20]],[["a",3,-60],["a",5,-30]],[["a",3,1],["a",5,10]],[["a",3,-60],["a",5,-30]],[["a",3,1],["a",5,10]]]#縦フリ

    arm_joint_values = arm.get_current_joint_values()
    for flame in range(len(data)):
        for joint_data in range(len(data[flame])):
            part=data[flame][joint_data][0]
            joint=int(data[flame][joint_data][1])
            angle = float(data[flame][joint_data][2])/180.0*math.pi

            print(part, joint, angle)
            if part == "a":
                # arm_joint_values = arm.get_current_joint_values()
                arm_joint_values[joint] = angle
            # elif part == "g":
            #     gripper_joint_values = gripper.get_current_joint_values()
            #     gripper_joint_values[joint] = angle
            #     gripper.set_joint_value_target(gripper_joint_values)
        print("flame")
        print(arm_joint_values)
        arm.set_joint_value_target(arm_joint_values)
        arm.go()
        # gripper.go()
    print("done")

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
