#!/usr/bin/env python

'''
Python implementation of ekfslam for mapping of racecourse and for localizing a racecar. 

Author: Jack McRobbie
'''

import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from ekfslam import EkfSlam
from raceCarSensors import racecarSensors
from slamTopics import slamTopics


def main(): 
    topics = slamTopics() 

    sense = racecarSensors(topics)

    rospy.init_node('EKFslam', anonymous=False)

    '''
    Call subscribers to various important nodes
    '''
    rospy.Subscriber("/lidar/cones", PointCloud2, sense.lidarCallback)
    rospy.Subscriber("/camera/cones", PointCloud2, sense.cameraCallback)
    '''
    Slam begin!
    '''    

    '''
    Rospy spin the node 
    '''
    rospy.spin()
    
if __name__ == "__main__":
    main()
