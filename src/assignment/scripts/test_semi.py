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



def listener(from_='/right_hand', to_='/right_shoulder'):
    listener = tf.TransformListener()
    from_name = from_ + '_1'
    to_name = to_ + '_1'
    
#    while not rospy.is_shutdown():
#        try:
#            (trans_from, rot_from) = listener.lookupTransform('/camera_depth_frame', from_name, rospy.Time(0))
#            (trans_to, rot_to) = listener.lookupTransform('/camera_depth_frame', to_name, rospy.Time(0))
#        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#            continue
    if(tf.LookupException or tf.ConnectivityException or tf.ExtrapolationException):
        pass
    else:
        (trans_from, rot_from) = listener.lookupTransform('/camera_depth_frame', from_name, rospy.Time(0))
        (trans_to, rot_to) = listener.lookupTransform('/camera_depth_frame', to_name, rospy.Time(0))
        print(from_name)
        relative_coords = np.array(trans_from) - np.array(trans_to)
        print(relative_coords)
        if relative_coords[2] < 0:
            angle_vector[3] = 120
        else:
            angle_vector[3] = 0



def talker(count):
    pub = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)
    rospy.init_node('skeleton2av', anonymous=True)
    rate = rospy.Rate(10)
    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()
    js.name = ["HEAD-NECK-Y", "HEAD-NECK-P", "LARM-SHOULDER-R", "LARM-SHOULDER-P", "LARM-ELBOW-P", "LARM-GRIPPER-R",
  "RARM-SHOULDER-P", "RARM-SHOULDER-R", "RARM-ELBOW-P", "RARM-GRIPPER-R", "LLEG-CROTCH-R", "LLEG-CROTCH-P",
  "LLEG-KNEE-P", "LLEG-ANKLE-P", "LLEG-ANKLE-R", "RLEG-CROTCH-R", "RLEG-CROTCH-P", "RLEG-KNEE-P",
  "RLEG-ANKLE-P", "RLEG-ANKLE-R"]
  
    tmp = [0.0 for i in range(20)]
    tmp[3] = -45
    tmp[2] = 45
    tmp[4] = -90
    tmp[15] = 2
    tmp[16] = -2
    js.position = tmp
    js.velocity = []
    js.effort = []
    #rospy.loginfo(js)
    #print(angle_vector[3])
    pub.publish(js)
    rate.sleep()

def talker_loop(duration=1):
    count = 0
    while not rospy.is_shutdown():
        try:
            talker(count)
            count+=3
#            listener()
        except rospy.ROSInterruptException:
            pass
        rospy.sleep(duration=duration)

if __name__ =='__main__':
    talker_loop()
