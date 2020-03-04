#!/usr/bin/env python
import rospy 
'''
Python implementation of ekfslam for mapping of racecourse and for localizing a racecar. 

Author: Jack McRobbie
'''

import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class slamTopics:
    def __init__(self):
        self.lidarCloudTopic = "/lidar/cones"
        self.cameraCloudTopic = "/camera/cones" 