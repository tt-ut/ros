#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import math

class Publishers():
    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)

        self.message = JointState()

        self.rate = rospy.Rate(20)

    def make_msg(self):

        self.message.header = Header()
        self.message.header.stamp = rospy.Time.now()
        self.message.position = []
        self.message.velocity = []
        self.message.effort = []

        tmp = [0.0 for i in range(20)]
        tmp[10] = 20.0       
        self.message.position = tmp

    def send_msg(self):
        self.make_msg()
        rospy.loginfo(self.message)
        self.publisher.publish(self.message)
        self.rate.sleep()

class Subscribe_publishers():
    def __init__(self, pub):
        # Subscriberを作成
        self.subscriber = rospy.Subscriber('puppet_joint_states', tf, self.callback)
        # messageの型を作成
        self.message = Pose2D()
        # publsishu

    def callback(self, message):
        # callback時の処理
        self.pub.make_msg(message)
        # publish
        self.pub.send_msg()



def main():
    # nodeの立ち上げ
    rospy.init_node('skeleton2av', anonymous=True)

    # クラスの作成
    pub = Publishers()
    #sub = Subscribe_publishers(pub)
    while not rospy.is_shutdown:
        try:
            pub.send_msg()
        except rospy.ROSInterruptException:
            pass
        rospy.sleep(0.5)


if __name__ == '__main__':
   main()