#!/usr/bin/env python3

import rospy
import math
from week2.srv import compute_ang_vel
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Client:
    def __init__(self):
        rospy.init_node('ang_vel_client',anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.wait_for_service('compute_ang_vel')
        self.calc = rospy.ServiceProxy('compute_ang_vel',compute_ang_vel)
        rospy.Subscriber('radius',Float32,self.callback)
        self.odm = rospy.Subscriber('odom',Odometry,self.cb)
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.rot = Twist()
        self.turn = 1
        self.px = 0
        self.py = 0
        self.tot_dist = 0

    def callback(self,msg):
        r = msg.data
        ang_vel = self.calc(r)
        self.rot.angular.z = ang_vel.w*self.turn
        self.rot.linear.x = 0.1
        self.pub.publish(self.rot)

    def cb(self,msg):
        cx = msg.pose.pose.position.x
        cy = msg.pose.pose.position.y
        dx = cx - self.px
        dy = cy - self.py
        dd = (dx**2 + dy**2)
        self.tot_dist += math.sqrt(dd)
        self.px = cx
        self.py = cy
        if(self.tot_dist > 2*math.pi):
            self.tot_dist = 0
            self.turn = -1*self.turn

    def shutdown(self):
        self.rot.linear.x = 0.0
        self.rot.angular.z = 0.0
        self.pub.publish(self.rot)
        rospy.sleep(1)


if __name__ == '__main__':
    new_client = Client()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
