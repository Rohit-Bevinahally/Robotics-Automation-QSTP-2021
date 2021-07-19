#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class helloworld:
    def __init__(self):
        rospy.init_node('helloworld_listen',anonymous=True)
        self.pub = rospy.Publisher('helloworld',String,queue_size=10)
        rospy.Subscriber('hello',String,self.cb_hello)
        rospy.Subscriber('world',String,self.cb_world)
        self.hello = ''
        self.world = ''

    def cb_hello(self,msg):
        self.hello = msg.data

    def cb_world(self,msg):
        self.world = msg.data

    def talker(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            op = self.hello + self.world
            rospy.loginfo(op)
            self.pub.publish(op)
            rate.sleep()


if __name__ == '__main__':
    hw = helloworld()
    try:
        hw.talker()
    except rospy.ROSInterruptException:
        pass
