#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy
from assignment.srv import *

def capitalize(request):
    return CapitalizeResponse(request.words.upper())

rospy.init_node("service_server")

service = rospy.Service("capitalize", Capitalize, capitalize)

rospy.spin()