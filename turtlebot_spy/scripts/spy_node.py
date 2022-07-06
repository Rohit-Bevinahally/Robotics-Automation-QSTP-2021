#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Path,Odometry
import sys
import tf
import math

class PID:
    def __init__(self):
        rospy.init_node('spy_node',anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('thief_pose',PoseStamped,self.cb)
        rospy.Subscriber('odom',Odometry,self.cb_odm)
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.mv = Twist()
        self.kp = 0.05    # Kp
        self.kd = 0.01    # Kd/dt , dt = 0.1
        self.ki = 0.0001  # Ki*dt
        self.cx = 0       # cur,prev (x,y) coordinates
        self.cy = 0
        self.px = 0
        self.py = 0
        self.co = 0       # Yaw
        self.cor = 0      # Angle to goal
        self.sum = 0      # Sum of errors
        self.standby = False
        self.warn_thief = False

    def cb(self,pose):
        # Goal (x,y):
        gx = pose.pose.position.x
        gy = pose.pose.position.y
        # Error :
        ex = (gx - self.cx)
        ey = (gy - self.cy)
        de = math.sqrt(ex**2 + ey**2)
        # Differential error :
        dex = (self.cx - self.px)
        dey = (self.cy - self.py)
        dde = math.sqrt(dex**2 + dey**2)
        # Integral error :
        self.sum += de
        # Linear velocity
        vel = self.kp*de + self.kd*dde + self.ki*self.sum
        self.mv.linear.x = 0 if self.standby else vel
        # Orientation correction :
        self.cor = math.atan2(ey,ex)
        orch = (self.cor - self.co)
        if orch > math.pi:
            orch -= 2*math.pi
        if orch < -math.pi:
            orch += 2*math.pi
        self.mv.angular.z = 0 if self.standby else 0.5*orch

        # Enter standby phase if goal is reached :
        if (abs(de) <= 1):
            self.standby = True
            if self.warn_thief == False :
                rospy.loginfo("Found You!!")
                self.warn_thief = True
            self.sum = 0
        else:
            self.standby = False
            self.warn_thief = False

        # Publish to cmd_vel :
        self.pub.publish(self.mv)


    def cb_odm(self,msg):
        self.px = self.cx
        self.py = self.cy
        self.cx = msg.pose.pose.position.x
        self.cy = msg.pose.pose.position.y
        o = msg.pose.pose.orientation
        angs = tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])
        self.co = angs[2]


    def shutdown(self):
        stop = Twist()
        self.pub.publish(stop)
        rospy.sleep(1)


if __name__ == '__main__':
    PID_Controller = PID()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

