#!/usr/bin/env python3

import rospy
import numpy as np
from week2.srv import trajectory,trajectoryResponse

dt = 0.05

def calc(request):
    xf = request.x + (request.v)*dt*(np.cos(request.theta))
    yf = request.y + (request.v)*dt*(np.sin(request.theta))
    tf = request.theta + (request.w)*dt
    return trajectoryResponse(xf,yf,tf)

rospy.init_node('service_server',anonymous=True)
service = rospy.Service('calc',trajectory,calc)
rospy.spin()
