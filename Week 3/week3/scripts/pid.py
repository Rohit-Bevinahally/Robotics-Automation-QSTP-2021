#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path,Odometry
import sys
import tf
import math

class PID:
    def __init__(self,path_name):
        rospy.init_node('pid',anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber(path_name,Path,self.cb)
        rospy.Subscriber('odom',Odometry,self.cb_odm)
        self.pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
        self.mv = Twist()
        self.kp = 0.05    # Kp
        self.kd = 0.01    # Kd/dt , dt = 0.1
        self.ki = 0.0001  # Ki*dt
        self.cg = 0       # Goal index
        self.cx = 0       # cur,prev (x,y) coordinates
        self.cy = 0
        self.px = 0
        self.py = 0
        self.co = 0       # Yaw
        self.cor = 0      # Angle to goal
        self.sum = 0      # Sum of errors
        self.adjust = False

    def cb(self,path):
        # Goal (x,y):
        gx = (path.poses[self.cg].pose.position.x)
        gy = (path.poses[self.cg].pose.position.y)
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
        self.mv.linear.x = 0 if self.adjust else vel
        # Orientation correction :
        self.cor = math.atan2(ey,ex)
        orch = (self.cor - self.co)
        if orch > math.pi:
            orch -= 2*math.pi
        if orch < -math.pi:
            orch += 2*math.pi
        self.mv.angular.z = 0.5*orch
        # Orient towards next goal :
        if self.adjust and abs(orch) <= 0.05:
            self.adjust = False
        # Reset if goal is reached :
        if (abs(ex) <= 0.05) and (abs(ey) <= 0.05):
            self.cg = (self.cg + 1) % (len(path.poses))
            self.sum = 0
            self.adjust = True
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
    path_name = sys.argv[1]
    PID_Controller = PID(path_name)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
