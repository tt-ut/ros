#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import numpy as np


robot_joint_dict = {"HEAD-NECK-Y":0, "HEAD-NECK-P":1, 
                    "LARM-SHOULDER-P":2, "LARM-SHOULDER-R":3, "LARM-ELBOW-P":4, "LARM-GRIPPER-R":5,
                    "RARM-SHOULDER-P":6, "RARM-SHOULDER-R":7, "RARM-ELBOW-P":8, "RARM-GRIPPER-R":9,
                    "LLEG-CROTCH-R":10, "LLEG-CROTCH-P":11, "LLEG-KNEE-P":12, "LLEG-ANKLE-P":13, "LLEG-ANKLE-R":14,
                    "RLEG-CROTCH-R":15, "RLEG-CROTCH-P":16, "RLEG-KNEE-P":17, "RLEG-ANKLE-P":18, "RLEG-ANKLE-R":19}


human_joint_dict = {"head":0, "neck":1, "torso":2, "left_shoulder":3,
                    "left_elbow":4, "left_hand":5, "left_hip":6, "left_knee":7, "left_foot":8,
                    "right_shoulder":9, "right_elbow":10, "right_hand":11, "right_hip":12,
                    "right_knee":13, "right_foot":14}

# for easy coding
C = {"head":0, "neck":1, "torso":2}
L = {"sh":3, "el":4,  "hand":5,  "hip":6,  "knee":7,  "ft":8}
R = {"sh":9, "el":10, "hand":11, "hip":12, "knee":13, "ft":14}

worldcoords = "camera_depth_frame" 

init_av = np.zeros(len(robot_joint_dict))
init_human_joint_coords = np.zeros((len(human_joint_dict), 3))

#TODO 

def angle(a_, b_, projection=None): # return angle between a to b
    a = a_.copy()
    b = b_.copy()
    if projection == "x":
        a[0] = 0
        b[0] = 0
    elif projection == "y":
        a[1] = 0
        b[1] = 0
    i = np.inner(a, b)
    n = np.linalg.norm(a) * np.linalg.norm(b)
    cos = i/n
    angle = np.rad2deg(np.arccos(np.clip(cos, -1.0, 1.0)))
    return angle

def make_simple_motion():
    left_arm_dir = get_relative_vector(L["sh"], L["hand"])
    right_arm_dir = get_relative_vector(R["sh"], R["hand"])
    left_leg_dir = get_relative_vector(L["hip"], L["ft"])
    right_leg_dir = get_relative_vector(R["hip"], R["ft"])
    l_sh_el = get_relative_vector(L["sh"], L["el"])
    l_el_hand = get_relative_vector(L["el"], L["hand"])
    r_sh_el = get_relative_vector(R["sh"], R["el"])
    r_el_hand = get_relative_vector(R["el"], R["hand"])
    c_head_torso = get_relative_vector(C["head"], C["torso"])
    #print("l_sh_el {}, el_hn{}".format(l_sh_el / np.linalg.norm(l_sh_el), l_el_hand / np.linalg.norm(l_el_hand)))
    

    av[robot_joint_dict["LARM-SHOULDER-R"]] = angle(c_head_torso, l_sh_el, projection="x")
    av[robot_joint_dict["RARM-SHOULDER-R"]] =  -1 * angle(c_head_torso, r_sh_el, projection="x")

    av[robot_joint_dict["LARM-SHOULDER-P"]] = angle(c_head_torso, l_sh_el, projection="y")
    av[robot_joint_dict["RARM-SHOULDER-P"]] = angle(c_head_torso, r_sh_el, projection="y")

    av[robot_joint_dict["LARM-ELBOW-P"]] = -180 + angle(l_el_hand, l_sh_el)
    av[robot_joint_dict["RARM-ELBOW-P"]] = -180 + angle(r_el_hand, r_sh_el)
    
#    av[robot_joint_dict["RARM-SHOULDER-R"]] = -90 - np.rad2deg(np.arctan2(right_arm_dir[2], right_arm_dir[1]))
#    av[robot_joint_dict["RARM-SHOULDER-P"]] = 90 + np.rad2deg(np.arctan2(right_arm_dir[2], right_arm_dir[0]))

#    
    
    #av[2] = 90 + np.rad2deg(np.arctan2(right_arm_dir[2], right_arm_dir[0]))
    #av[3] = 90 + np.rad2deg(np.arctan2(right_arm_dir[2], right_arm_dir[1]))


def make_av_manually():

    #joint = "LARM-SHOULDER-P"
    #angle = 90

    #av[robot_joint_dict[joint]] = angle
    av[10] = 0

    
def value2key(dict_name, value): #key(joint_name)->key, value(joint_index)->key
    if type(value) is str:
        return value
    elif type(value) is int:
        return dict_name.keys()[value]
    else:
        print("invalid: value2key")
        return -1

def get_worldpos(joint_name, coords="body"): # get coordinates in the world coordinate system
    trans = []
    rot = []
    joint_name = value2key(human_joint_dict, joint_name) + "_1"
    rate = rospy.Rate(1000)
    while not trans:
        try: # sub: instance of tf.Transformlistner()
            (trans, rot) = sub.lookupTransform(joint_name, worldcoords, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
    trans = np.array(trans)
    if coords=="body":
        #trans[0] = -1 * trans[0]
        trans[1] = -1 * trans[1]
    return trans # shape=(3,)

def set_worldpos(joint_name): 
    human_joint_coords[human_joint_dict[joint_name]] = get_worldpos(joint_name)

def set_all_worldpos():
    for joint in human_joint_dict.keys():
        set_worldpos(joint)

def get_relative_vector(from_, to_):
    from_name = value2key(human_joint_dict, from_)
    to_name = value2key(human_joint_dict, to_)
    return human_joint_coords[human_joint_dict[to_name]] - human_joint_coords[human_joint_dict[from_name]]

def talker():
    rate = rospy.Rate(10)

    js = JointState()
    js.header = Header()
    js.header.stamp = rospy.Time.now()    
    js.name = robot_joint_dict.keys()

    js.position = list(av) #global variable
    js.velocity = []
    js.effort = []  
    pub.publish(js) # pub: rospy.Publisher('puppet_joint_states', JointState, queue_size=10)
    rate.sleep()

def listener():
    rate = rospy.Rate(20)
    set_all_worldpos()
    if config["motion_type"] == "imitate":
        make_simple_motion()
    elif config["motion_type"] == "manual":
        make_av_manually()
    rate.sleep()

def check_listen():
    #temp_list = []
    #print("check")
    for joint, i in human_joint_dict.items():
       # print(joint)
        #tmp = get_worldpos(joint)
        #av[i] = tmp[0]
        #print(joint)
        set_worldpos(joint)



def print_human_joint_coords():
    print("\n")
    print("  :{:>15} :  {:^7}  {:^7}  {:^7}".format("human_joint", "x[m]", "y[m]", "z[m]"))
    print ("-"*48)
    for joint, i in sorted(human_joint_dict.items(), key=lambda x: x[1]):
        x, y, z = human_joint_coords[i]
        print("{:>2}:{:>15} : {: .4f}  {: .4f}  {: .4f}".format(i, joint, x, y, z))

def print_robot_angle_vector():
    print("\n")
    print("  : {:>15} : {:^7}".format("robot_joint", "[rad]"))
    print ("-"*30)
    for joint, i in sorted(robot_joint_dict.items(), key=lambda x: x[1]):
        print("{:>2}: {:>15} : {: .4f}".format(i, joint, av[i]))


def main():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        try:
            talker()
            listener()
            #print(human_joint_coords)
            if config["visualize"]:
                print_human_joint_coords()
                print_robot_angle_vector()

        except rospy.ROSInterruptException:
            pass
        rate.sleep()

if __name__ == '__main__':
    #config
    config = {"visualize"  :True,
         "motion_type":"imitate"}

    #initialize valiable
    av = init_av
    human_joint_coords = init_human_joint_coords

    #initialize node
    rospy.init_node('skeleton2av')
    sub = tf.TransformListener()
    pub = rospy.Publisher('puppet_joint_states', JointState, queue_size=10)

    #main loop
    main()


            








    

