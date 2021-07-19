#!/usr/bin/env python3 
import rospy
from std_msgs.msg import String

def talker():
    publisher  = rospy.Publisher('world',String,queue_size=10)
    rospy.init_node('world_talk',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        op = 'World!'
        rospy.loginfo(op)
        publisher.publish(op)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
