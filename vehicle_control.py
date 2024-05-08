#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist

class VehicleControler:
    """
    Manages the steering of the vehicle.
    """    
    def __init__(self):
        self.ctrl_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.ctrl_msg = Twist()
        
    def control(self, speed, steering):
        self.ctrl_msg.linear.x = speed
        self.ctrl_msg.angular.z = steering
        self.ctrl_pub.publish(self.ctrl_msg)
    