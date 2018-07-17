#!/usr/bin/python
# -*- coding: utf-8 -*-

import os
import time
import roslib
import rospy
import tf
import rosbag
import subprocess
import time
from std_msgs.msg import UInt8
from sensor_msgs.msg import LaserScan


class LaserListener():
    def __init__(self):
        rospy.init_node('LaserListener')
        rospy.Subscriber("/r2000_node/scan", LaserScan, self.callback)
        self.LaserPub = rospy.Publisher('/scan', LaserScan, queue_size=5)

    def callback(self,laser_data):
        print("LaserScan Call Back!!!")
        print(laser_data.angle_min)


if __name__ == '__main__':
    try:
        LaserListener()
        rospy.spin()
    except:
        rospy.loginfo("LaserListener terminated.")
