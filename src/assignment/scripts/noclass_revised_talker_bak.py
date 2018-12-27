#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import numpy as np


robot_joint_dict = {"HEAD-NECK-Y":0, "HEAD-NECK-P":1, 
                    "LARM-SHOULDER-R":2, "LARM-SHOULDER-P":3, "LARM-ELBOW-P":4, "LARM-GRIPPER-R":5,
                    "RARM-SHOULDER-P":6, "RARM-SHOULDER-R":7, "RARM-ELBOW-P":8, "RARM-GRIPPER-R":9,
                    "LLEG-CROTCH-R":10, "LLEG-CROTCH-P":11, "LLEG-KNEE-P":12, "LLEG-ANKLE-P":13, "LLEG-ANKLE-R":14,
                    "RLEG-CROTCH-R":15, "RLEG-CROTCH-P":16, "RLEG-KNEE-P":17, "RLEG-ANKLE-P":18, "RLEG-ANKLE-R":19}

human_joint_dict = {"head":0, "neck":1, "torso":2, "left_shoulder":3,
                    "left_elbow":4, "left_hand":5, "left_hip":6, "left_knee":7, "left_foot":8,
                    "right_shoulder":9, "right_elbow":10, "right_hand":11, "right_hip":12,
                    "right_knee":13, "right_foot":14}

worldcoords = "camera_depth_frame" 

init_av = [0.0 for i in range(len(robot_joint_dict))]
init_human_joint_coords = [0.0 for i in range(len(human_joint_dict))]

def value2key(dict_name, value): #key(joint_name)->key, value(joint_index)->key
    if type(value) is str:
        return value + "_1"
    elif type(value) is int:
        return dict_name.keys()[value] + "_1"
    else:
        print("invalid: value2key")
        return -1

def get_worldpos(joint_name): # get coordinates in the world coordinate system
    trans = []
    rot = []
    joint_name = value2key(human_joint_dict, joint_name)
    rate = rospy.Rate(50)
    while not trans:
        try: # sub: instance of tf.Transformlistner()
            (trans, rot) = sub.lookupTransform(worldcoords, joint_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    rate.sleep()
    return trans # tupple (len=3)

def set_worldpos(joint_name): 
    human_joint_coords[human_joint_dict[joint_name]] = get_worldpos(joint_name)

def talker():
    rate = rospy.Rate(10)

    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()    
    js.name = robot_joint_dict.keys()

    js.position = av #global variable
    js.velocity = []
    js.effort = []  
    pub.publish(js) # pub: rospy.Publisher('puppet_joint_states', JointState, queue_size=10)
    rate.sleep()

def get_relative_vector(from_, to_):
    from_name = value2key(human_joint_dict, from_)
    to_name   = value2key(human_joint_dict, to_)

    trans_from, trans_to, rot_from, rot_to = [], [], [], []
    rate = rospy.Rate(50)

    while not trans_from:
        try:
            (trans_from, rot_from) = sub.lookupTransform(worldcoords, from_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    while not trans_to:
        try:
            (trans_to, rot_to) = sub.lookupTransform(worldcoords, to_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    rv = np.array(trans_to) - np.array(trans_from)
    return rv #shape=(3,)

def listener():
    rate = rospy.Rate(50)
    check_listen()
    #rv = get_relative_vector("right_shoulder", "right_hand")
    #if rv.shape == (3,):
    #    av[3] = 90.0
    rate.sleep()

def check_listen():
    temp_list = []
    #print("check")
    for joint, i in human_joint_dict.items():
       # print(joint)
        #tmp = get_worldpos(joint)
        #av[i] = tmp[0]
        set_worldpos(joint)

def main():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            talker()
            listener()
            print(human_joint_coords)
        except rospy.ROSInterruptException:
            pass
        rate.sleep()

if __name__ == '__main__':
    #initialize valiable
    av = init_av
    human_joint_coords = init_human_joint_coords

    #initialize node
    rospy.init_node('skeleton2av')
    sub = tf.TransformListener()
    pub = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)

    #main loop
    main()


            








    

