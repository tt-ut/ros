#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from numpy.random import randn
from std_msgs.msg import Float64

rospy.init_node('publisher_python')

pub = rospy.Publisher('rand', Float64, queue_size=10) # トピック名: rand, メッセージの型: float64

rate = rospy.Rate(20) # 1秒あたり20回送信

while not rospy.is_shutdown(): 
    r = randn() # 平均0 分散1 の正規分布に従う乱数を生成
    pub.publish(r)
    rate.sleep()

    
    