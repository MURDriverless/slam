 #!/usr/bin/env python
 
import rospy 
'''
EKF slam implementation for MUR driverless

Author: Jack McRobbie 
'''
import numpy as np
import rospy 

class EkfSlam:
    
    def __init__(self,sensors):
        self.data = sensors 
    
    
    def doSlam(self):
        while(True):
            mean = np.mean(self.data.cameraCloud)
            rospy.loginfo(mean)