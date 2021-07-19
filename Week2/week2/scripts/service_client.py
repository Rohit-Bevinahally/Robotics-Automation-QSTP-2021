#!/usr/bin/env python3

import rospy
from week2.srv import trajectory
import matplotlib.pyplot as plt
n = 50
v = 5
w = 4
x_vals = [0]
y_vals = [0]
rospy.init_node('service_client',anonymous=True)
rospy.wait_for_service('calc')
calculate = rospy.ServiceProxy('calc',trajectory)
x = 0
y = 0
theta = 0.77
for i in range(n):
    new_pos = calculate(x,y,theta,v,w)
    x = new_pos.cx
    y = new_pos.cy
    theta = new_pos.ctheta
    x_vals.append(x)
    y_vals.append(y)

plt.title(f"Unicycle Model: {v}, {w}")
plt.xlabel("X-Coordinates")
plt.ylabel("Y-Coordinates")
plt.plot(x_vals, y_vals, color="red", alpha=0.75)
plt.grid()
plt.show()
