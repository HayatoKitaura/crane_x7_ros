#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import time
import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler
from darknet_ros_msgs.msg import BoundingBox

# detect_x = 0
# detect_y = 0

# def get_BBox(darknet_bbox):
#     local_x = darknet_bbox.xmax - darknet_bbox.xmin
#     local_y = darknet_bbox.ymax - darknet_bbox.ymin
#     print("x:",local_x)
#     print("y:",local_y)


def main():
    rospy.init_node("crane_x7_detect_and_swing_penlight")
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

    # アーム初期ポーズを表示
    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()


    # 掴む準備をする
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
    arm.go()  # 実行

    # 探索
    print("search")
    arm.set_named_target("search")
    arm.go()
    time.sleep(3.0)
    # print("sub")
    # rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBox, get_BBox)

    # ハンドを開く
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    # 掴みに行く
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.2
    target_pose.position.y = 0.0
    target_pose.position.z = 0.08
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    # ハンドを閉じる
    gripper.set_joint_value_target([0.01, 0.01])
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

    data = [["a",0,1],["a",1,40],["a",5,30],["a",1,-40],["a",5,1],["a",1,40],["a",5,30],["a",1,-40],["a",5,1],["a",1,1],["a",0,90],["a",1,20],["a",3,-60],["a",5,-30],["a",3,1],["a",5,1],["a",3,-60],["a",5,-30],["a",3,1],["a",5,1]]
    for i in range(len(data)):
        data[i][0],data[i][1],data[i][2]
        part=data[i][0]
        joint=int(data[i][1])
        angle = float(data[i][2])/180.0*math.pi

        print(part, joint, angle)
        if part == "a":
            arm_joint_values = arm.get_current_joint_values()
            arm_joint_values[joint] = angle
            arm.set_joint_value_target(arm_joint_values)
            arm.go()
        elif part == "g":
            gripper_joint_values = gripper.get_current_joint_values()
            gripper_joint_values[joint] = angle
            gripper.set_joint_value_target(gripper_joint_values)
            gripper.go()
    print("done")


if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
