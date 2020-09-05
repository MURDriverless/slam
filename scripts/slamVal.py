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
import matplotlib.pyplot as plt
from mur_common.msg import mur_drive_cmd


def pi2pi(val):
    if val > math.pi:
        return -(math.pi - val % math.pi)
    elif (val < - math.pi):
        return (math.pi - val % math.pi)
    else:
        return val


class Simulation:
    def __init__(self, odomTopic, conesTopic, rate):
        self.tr = map(10, 10, 15)
        self.car = robot()
        self.map = []
        self.poseEst = robot()
        self.rateHz = rate
        self.conePub = rospy.Publisher(
            "/cone_messages", cone_msg, queue_size=1)
        # self.odomPub = rospy.Publisher("/Odom", Pose2D, queue_size=1)
        self.cmdPub = rospy.Publisher(
            "/cmd_vel", mur_drive_cmd, queue_size=1)
        self.mapSub = rospy.Subscriber(
            "/slam/map", cone_msg, self.mapClb, queue_size=1)
        self.poseSub = rospy.Subscriber(
            "/slam/odom", Pose2D, self.poseclb, queue_size=1)

    def poseclb(self, pose):
        self.poseEst.x = pose.x
        self.poseEst.y = pose.y
        self.poseEst.theta = pose.theta

    def mapClb(self, map):
        self.map = list(zip(map.x, map.y))
        return

    def run(self):
        dt = 1.0 / self.rateHz
        ros_rate = rospy.Rate(self.rateHz)
        self.fig = plt.figure()
        plt.xlabel('x (m)')
        plt.ylabel('y (m)')
        plt.ion()
        self.v = 0.2
        self.omegaVel = 0.17
        while(not rospy.is_shutdown()):
            '''
            Updating of simulation and plotting
            '''
            self.car.forwardK(
                dt, self.v, self.omegaVel)  # Hardcoded inputs for now
            detected = self.getSensorReadings()
            self.doPublishing(detected)
            # rospy.loginfo("x: %.2f ||  y :%.2f", self.car.x, self.car.y)
            '''
            Plotting
            '''
            self.plotting(detected)
            '''
            Ros housekeeping
            '''
            ros_rate.sleep()
        return

    def plotting(self, det):
        plt.cla()
        axes = plt.gca()
        axes.set_xlim([self.tr.x_range[0], self.tr.x_range[1]])
        axes.set_ylim([self.tr.y_range[0], self.tr.y_range[1]])
        plt.scatter(self.car.x, self.car.y)

        x = self.poseEst.x
        y = self.poseEst.y
        theta = self.poseEst.theta
        plt.arrow(x, y, self.v * math.cos(theta),
                  self.v * math.sin(theta), head_width=0.05, head_length=0.1, fc='b', ec='b')

        x = self.car.x
        y = self.car.y
        theta = self.car.theta
        plt.arrow(x, y, self.v * math.cos(theta), self.v * math.sin(theta),
                  head_width=0.05, head_length=0.1, fc='g', ec='g')

        for lm in self.tr.track:
            plt.scatter(lm[0], lm[1], marker="o", c="r", )

        for lm in det:
            plt.scatter(lm[0]*math.cos(theta) + lm[1] * math.sin(theta) +
                        x, -1 * lm[0]*math.sin(theta) + lm[1] * math.cos(theta)+y, marker="x", c="g")
        for lm in self.map:
            plt.scatter(lm[0], lm[1], marker="x", c="b")

        plt.grid(True)
        plt.pause(0.0001)

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
                xr = (x-x_s)*math.cos(theta_s) - (y-y_s)*math.sin(theta_s)
                yr = +(x-x_s)*math.sin(theta_s) + (y-y_s)*math.cos(theta_s)
                detected.append((xr + rn.gauss(0, 0.01),
                                 yr + rn.gauss(0, 0.01)))
        return detected

    def doPublishing(self, detected):
        detected_msg = cone_msg()
        x_list = []
        y_list = []
        for item in detected:
            x_list.append(item[0])
            y_list.append(item[1])
        detected_msg.x = x_list
        detected_msg.y = y_list
        detected_msg.header.frame_id = "base_link"
        self.conePub.publish(detected_msg)
        # pse = Pose2D()
        # pse.x = self.car.x
        # pse.y = self.car.y
        # pse.theta = self.car.theta
        # self.odomPub.publish(pse)
        # Publish command
        cmd_msg = mur_drive_cmd()
        cmd_msg.header.frame_id = "base_link"
        cmd_msg.omega = self.omegaVel
        cmd_msg.vel = self.v
        self.cmdPub.publish(cmd_msg)
        return


class map:
    def __init__(self, x_size, y_size, lm_count, raceTrack=False):
        self.x_range = (-x_size/2, x_size/2)
        self.y_range = (-y_size/2, y_size/2)
        self.generateRandomTrack(lm_count)
        pass

    def generateRandomTrack(self, num_cones):
        self.track = [(1, 1)]
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
        self.x = self.x + dt * (v + rn.gauss(0, 0.01)) * math.cos(self.theta)
        self.y = self.y + dt * (v + rn.gauss(0, 0.01)) * math.sin(self.theta)
        self.theta = self.theta + dt * (omega + rn.gauss(0, 0.01))
        self.theta = pi2pi(self.theta)
        return


if __name__ == "__main__":
    rospy.init_node('slamVal')
    rateHz = 5
    targetLMTopic = "/cones/lidar"
    targetPoseTopic = "/Pose/2D"

    sim = Simulation(targetPoseTopic, targetPoseTopic, rateHz)
    sim.run()
    exit()
