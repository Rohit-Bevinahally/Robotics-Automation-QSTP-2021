#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

r = 1

def talker():
    rospy.init_node('radius_talk',anonymous=True)
    pub = rospy.Publisher('radius',Float32,queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(r)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

