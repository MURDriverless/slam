'''
RaceCar sensor class 

Jack McRobbie

'''

import numpy as np 

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2


class racecarSensors:
    '''
    A class that manages callbacks and sensors states for the racecar. 
    '''
    def __init__(self,topics):
        self.lidarTopic = topics.lidarCloudTopic
        self.cameraTopic = topics.cameraCloudTopic
        print type(self.lidarTopic)
        print self.cameraTopic
        self.lidarCloud = np.array([]) 
        self.cameraCloud = np.array([])
    
    def lidarCallback(self, data):    
        dat = pc2.read_points(data,
                                field_names=("x", "y", "z"),
                                skip_nans=True)
        arr = list(dat)
        self.lidarCloud = np.array(arr)

    def cameraCallback(self,data):
        dat = pc2.read_points(data,
                        field_names=("x", "y", "z"),
                        skip_nans=True)
        arr = list(dat)
        self.cameraCloud = np.array(arr)
        print(np.size(self.lidarCloud))