#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import math
import numpy as np

'''
camera_depth_frame
head
neck
torso
left_shoulder
left_elbow
left_hand
left_hip
left_knee
left_foot
right_shoulder
right_elbow
right_hand
right_hip
right_knee
right_foot
'''

angle_vector = [0.0 for i in range(20)]




def listener(from_='camera_depth_optical_frame'):
    listener = tf.TransformListener()
    #from_name = from_ + '_1'
    
#    while not rospy.is_shutdown():
#        try:
#            (trans_from, rot_from) = listener.lookupTransform('/camera_depth_frame', from_name, rospy.Time(0))
#            (trans_to, rot_to) = listener.lookupTransform('/camera_depth_frame', to_name, rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            continue
    #if(tf.LookupException or tf.ConnectivityException or tf.ExtrapolationException):
    if(tf.LookupException):
        print("exception")
    else:
        print("aaa")
        (trans, rot) = listener.lookupTransform('camera_depth_frame', from_, rospy.Time(0))
        print(np.array(trans))




def talker():
    pub = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)
    rospy.init_node('skeleton2av', anonymous=True)
    rate = rospy.Rate(10)
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    tmp = [0.0 for i in range(20)]
    tmp[10] = 20.0
    js.position = angle_vector
    js.velocity = []
    js.effort = []
    #rospy.loginfo(js)
    print(angle_vector[3])
    pub.publish(js)
    rate.sleep()

def talker_loop(duration=0.5):
    while not rospy.is_shutdown():
        try:
            talker()
            listener()
        except rospy.ROSInterruptException:
            pass
        rospy.sleep(duration=duration)

if __name__ =='__main__':
    talker_loop()
