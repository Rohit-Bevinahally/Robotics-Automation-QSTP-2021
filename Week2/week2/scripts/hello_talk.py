#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('hello', String, queue_size=10)
    rospy.init_node('hello_talk', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        op = "Hello, "
        rospy.loginfo(op)
        pub.publish(op)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
