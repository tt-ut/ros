#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np

def listen():
    trans1 = []
    rot1 = []
    trans2 = []
    rot2 = []

    rate = rospy.Rate(10.0)
    while not trans1:
        try:
            (trans1, rot1) = listener.lookupTransform('camera_depth_frame', 'left_hand_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    while not trans2:
        try:
            (trans2, rot2) = listener.lookupTransform('camera_depth_frame', 'left_shoulder_1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    
    trans1 = np.array(trans1)
    trans2 = np.array(trans2)
    tmp = list(trans2 - trans1)

    #print 'Translation1: ' , trans1
    #print("Translation2: {}".format(trans2))
    print('relative vector {}'.format(tmp))



if __name__ == '__main__':
    rospy.init_node('look_up_pepper_tf')
    listener = tf.TransformListener()
    
    while True:
        listen()
        #print("a")
        rospy.sleep(1)
    
