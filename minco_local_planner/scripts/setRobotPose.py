#! /usr/bin/python3
# -*- coding: utf-8 -*-
# @Author: Yunkai Xia
# @Date:   2022-08-26 08:45:13
# @Last Modified by:   Yunkai Xia
# @Last Modified time: 2023-03-09 11:05:26
#! /usr/bin/python
#! -*- coding: utf-8 -*-

'''

监听initial_pose并依此更改gazebo中机器人的位置
author: flztiii
2021/6/9

'''

import rospy
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

# 设置机器人初始位置
class SetRobotState:
    # 构造函数
    def __init__(self):
        # 消息订阅
        self.init_pose_sub_ = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.initialPoseCallback, queue_size=1)
        # 服务订阅
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state_service_ = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    # 位置信息获取
    def initialPoseCallback(self, pose_msg):
        model_state_msg = ModelState()
        model_state_msg.model_name = rospy.get_param("~simulation_model_name", "turtlebot3_waffle")
        model_state_msg.pose = pose_msg.pose.pose
        # model_state_msg.pose.position.x = -4.035038547533068
        # model_state_msg.pose.position.y = 5.179230222003518
        model_state_msg.twist.linear.x = 0
        model_state_msg.twist.linear.y = 0
        model_state_msg.twist.linear.z = 0
        model_state_msg.twist.angular.x = 0
        model_state_msg.twist.angular.y = 0
        model_state_msg.twist.angular.z = 0
        try:
            response = self.set_model_state_service_(model_state_msg)
        except rospy.ServiceException:
            print("Service call failed")

# 主函数
def main():
    # 初始化ros
    rospy.init_node("set_robot_pose_node")
    # 程序启动
    set_robot_state = SetRobotState()
    # 程序阻塞
    rospy.spin()

if __name__ == "__main__":
    main()
