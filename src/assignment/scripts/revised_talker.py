#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import numpy as np

class Skeleton2av():

    robot_joint_dict = {"HEAD-NECK-Y":0, "HEAD-NECK-P":1, 
                        "LARM-SHOULDER-R":2, "LARM-SHOULDER-P":3, "LARM-ELBOW-P":4, "LARM-GRIPPER-R":5,
                        "RARM-SHOULDER-P":6, "RARM-SHOULDER-R":7, "RARM-ELBOW-P":8, "RARM-GRIPPER-R":9,
                        "LLEG-CROTCH-R":10, "LLEG-CROTCH-P":11, "LLEG-KNEE-P":12, "LLEG-ANKLE-P":13, "LLEG-ANKLE-R":14,
                        "RLEG-CROTCH-R":15, "RLEG-CROTCH-P":16, "RLEG-KNEE-P":17, "RLEG-ANKLE-P":18, "RLEG-ANKLE-R":19}
 
    human_joint_list = ["camera_depth_frame", "head", "neck", "torso", "left_shoulder",
                        "left_elbow", "left_hand", "left_hip", "left_knee", "left_foot",
                        "right_shoulder", "right_elbow", "right_hand", "right_hip",
                        "right_knee", "right_foot"]

    def __init__(self):
        # Publisherを作成
        self.publisher = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)
        self.listener = tf.TransformListener()

        self.msg = JointState()
        self.msg.header = Header()
        self.msg.name = self.robot_joint_dict.keys()
        self.msg.position = []
        self.msg.velocity = []
        self.msg.effort = []

        self.worldcoords = self.human_joint_list[0]
        self.rate = rospy.Rate(10)
        self.av = [0.0 for i in range(20)]

    def make_msg(self):
        self.msg.header.stamp = rospy.Time.now()
        self.av[2] = 90
        self.msg.position = self.av

    def send_msg(self):
        self.make_msg()
        rospy.loginfo(self.msg)
        self.publisher.publish(self.msg)
        self.rate.sleep()

    def get_relative_vector(self, from_index, to_index):
        if type(from_index) is str:
            from_name = from_index
        elif type(from_index) is int:
            from_name = self.human_joint_list[from_index]
        else:
            print("invalid: get_relative_vector")
        if type(to_index) is str:
            to_name = to_index
        elif type(to_index) is int:
            to_name = self.human_joint_list[to_index]
        else:
            print("invalid: get_relative_vector")

        trans_from = []
        rot_from = []
        trans_to = []
        rot_to = []

        rate = self.rate

        while not trans_from:
            try: # listener is instance of tf.Transformlistner()
                (trans_from, rot_from) = self.listener.lookupTransform(self.worldcoords, from_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

        while not trans_to:
            try: # listener is instance of tf.Transformlistner()
                (trans_to, rot_to) = self.listener.lookupTransform(self.worldcoords, to_name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()

        rv = np.array(trans_to) - np.array(trans_from)

        return rv #shape=(3,)

    def receive_msg(self):
        self.get_relative_vector(0, 3)


def main():
    # set node
    rospy.init_node('skeleton2av', anonymous=True)

    #create instance

    while not rospy.is_shutdown:
        try:
            skeleton2av = Skeleton2av()
            skeleton2av.send_msg()
            skeleton2av.receive_msg()
        except rospy.ROSInterruptException:
            pass
        rospy.sleep(0.1)

if __name__ == '__main__':
   main()

            








    

