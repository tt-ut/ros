#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
import time
import actionlib
from assignment.msg import TimerAction, TimerGoal, TimerResult

rospy.init_node("timer_action_client")
client = actionlib.SimpleActionClient("timer", TimerAction)
client.wait_for_server()
goal = TimerGoal()
goal.time_to_wait__goal = rospy.Duration.from_sec(2.0)
client.send_goal(goal)
client.wait_for_result()
print("Time elapsed: %f" %(client.get_result().time_elapsed__result.to_sec()))