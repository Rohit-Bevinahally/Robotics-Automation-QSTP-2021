#!/usr/bin/env python3

import rospy
from week2.srv import compute_ang_vel,compute_ang_velResponse

v = 0.1

def calc(request):
    w = v/(request.r)
    return compute_ang_velResponse(w)

if __name__ == '__main__':
    rospy.init_node('ang_vel_server',anonymous=True)
    service = rospy.Service('compute_ang_vel',compute_ang_vel,calc)
    rospy.spin()
