#!/usr/bin/env python
import rospy
from mur_common.msg import cone_msg
from geometry_msgs.msg import Pose2D
import math
import random as rn
import matplotlib.pyplot as plt
from mur_common.msg import mur_drive_cmd


class robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0


class state:
    def __init__(self):
        self.map = []
        self.poseEst = robot()
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

    def plotting(self):
        plt.cla()
        axes = plt.gca()

        x = self.poseEst.x
        y = self.poseEst.y
        theta = self.poseEst.theta
        plt.arrow(x, y,  math.cos(theta),
                  math.sin(theta), head_width=0.05, head_length=0.1, fc='b', ec='b')
        for lm in self.map:
            plt.scatter(lm[0], lm[1], marker="x", c="b")

        plt.grid(True)
        plt.pause(0.0001)


if __name__ == "__main__":
    rospy.init_node('slamOutputVisualization')
    rateHz = 5
    st = state()
    ros_rate = rospy.Rate(5)
    while(not rospy.is_shutdown()):
        st.plotting()
        ros_rate.sleep()
