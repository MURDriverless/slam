#!/usr/bin/env python
'''
Python script that creates a ros node that does a few cool things. 
- Creates a world of landmarks 
- drives a robot through that world utilising control inputs
- publishes sensor measurements 

Used for basic performance validation of the slam algorithm.
'''
import rospy
from mur_common.msg import cone_msg
from geometry_msgs.msg import Pose2D
import math
import random as rn


class Simulation:
    def __init__(self, odomTopic, conesTopic, rate):
        self.tr = map(10, 10, 5)
        self.car = robot()
        self.rateHz = rate

    def run(self):
        dt = 1.0 / self.rateHz
        ros_rate = rospy.Rate(self.rateHz)
        while(not rospy.is_shutdown()):
            '''
            Updating of simulation and plotting
            '''
            self.car.forwardK(dt, 0.5, 0.01)  # Hardcoded inputs for now
            detected = self.getSensorReadings()
            self.doPublishing(detected)
            '''
            Plotting
            '''

            '''
            Ros housekeeping
            '''
            rospy.spin()
            ros_rate.sleep()
        return

    def getSensorReadings(self):
        x_s = self.car.x
        y_s = self.car.y
        theta_s = self.car.theta
        detected = []
        for lm in self.tr.track:
            y = lm[1]
            x = lm[0]
            r = math.sqrt((x_s - x)**2 + (y_s - y)**2)
            theta = math.atan2(y - y_s, x - x_s)

            if (r < 5 and abs(theta-theta_s) < 1):
                # Assume cone can be detected
                detected.append((x, y))
        return detected

    def doPublishing(self, detected):
        pass


class map:
    def __init__(self, x_size, y_size, lm_count, raceTrack=False):
        self.x_range = (-x_size/2, x_size/2)
        self.y_range = (-y_size/2, y_size/2)
        pass

    def generateRandomTrack(self, num_cones):
        self. track = []
        for i in range(num_cones):
            x = rn.uniform(self.x_range[0], self.x_range[1])
            y = rn.uniform(self.y_range[0], self.y_range[1])
            self.track.append((x, y))
        return


class robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def forwardK(self, dt, v, omega):
        '''
        Perform forward kinematics state update
        '''
        self.x = self.x + dt * v * math.cos(self.theta)
        self.y = self.y + dt * v * math.sin(self.theta)
        self.theta = self.theta + dt * omega
        return


if __name__ == "__main__":
    rospy.init_node('slamVal')
    rateHz = 30
    targetLMTopic = "/cones/lidar"
    targetPoseTopic = "/Pose/2D"

    sim = Simulation(targetPoseTopic, targetPoseTopic, rateHz)
    sim.run()
