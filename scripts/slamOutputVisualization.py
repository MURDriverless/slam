#!/usr/bin/env python
import rospy
from mur_common.msg import cone_msg
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math
import random as rn
import matplotlib.pyplot as plt
import os
import csv
import pandas as pd


class robot:
    def __init__(self):
        self.x = []
        self.y = []
        self.theta = 0


class state:
    def __init__(self):
        self.map = []
        self.true_x = []
        self.true_y = []
        self.poseEst = robot()
        self.mapSub = rospy.Subscriber(
            "/mur/slam/cones", cone_msg, self.mapClb, queue_size=1)

        script_dir = os.path.dirname(__file__)
        file_path = script_dir + "/small_track_truth" + "/blue_cone.csv"
        self.df_blue = pd.read_csv(file_path, header=None, usecols=[
                                   0, 1])

        file_path = script_dir + "/small_track_truth" + "/big_cone.csv"
        self.df_big = pd.read_csv(file_path, header=None, usecols=[0, 1])
        file_path = script_dir + "/small_track_truth" + "/yellow_cone.csv"
        self.df_yellow = pd.read_csv(file_path, header=None, usecols=[0, 1])

        self.poseSub = rospy.Subscriber(
            "/mur/slam/Odom", Odometry, self.poseclb, queue_size=1)
        self.poseSub = rospy.Subscriber(
            "/odom", Odometry, self.trueposeclb, queue_size=1)

    def trueposeclb(self, pose):
        self.true_x.append(pose.pose.pose.position.x)
        self.true_y.append(pose.pose.pose.position.y)

    def poseclb(self, pose):
        x_off = -13.0
        y_off = 10.3
        self.poseEst.x.append(pose.pose.pose.position.x + x_off)
        self.poseEst.y.append(pose.pose.pose.position.y + y_off)

    def mapClb(self, map):
        self.map = list(zip(map.x, map.y))
        self.colour = map.colour
        return

    def plotting(self):
        x_off = -13.0
        y_off = 10.3
        plt.figure(1)
        plt.cla()
        axes = plt.gca()
        # plt.arrow(x, y,  math.cos(theta),
        #           math.sin(theta), head_width=0.05, head_length=0.1, fc='b', ec='b')
        self.df_blue.plot.scatter(x=0, y=1, ax=axes)
        self.df_yellow.plot.scatter(x=0, y=1, ax=axes)
        self.df_big.plot.scatter(x=0, y=1, ax=axes)

        for i, lm in enumerate(self.map):
            if self.colour[i] == "BLUE":
                axes.scatter(lm[0] + x_off, lm[1]+y_off, marker="x", c="b")
            if self.colour[i] == "YELLOW":
                axes.scatter(lm[0] + x_off, lm[1]+y_off, marker="x", c="y")
            if self.colour[i] == "BIG" or self.colour[i] == "ORANGE":
                axes.scatter(lm[0] + x_off, lm[1]+y_off,
                             marker="x", color='orange')
            else:
                pass
        plt.plot(self.poseEst.x, self.poseEst.y, c="r")

        plt.plot(self.true_x, self.true_y, c="b")

        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")

        plt.grid(True)
        plt.pause(0.0001)

        time = rospy.Time.now()
        script_dir = os.path.dirname(__file__)
        results_dir = os.path.join(script_dir, 'Results/')
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)

        # print(my_path)
        fileName = results_dir+"SlamOutput" + str(time)+".pdf"
        plt.savefig(fileName)


if __name__ == "__main__":
    rospy.init_node('slamOutputVisualization')
    st = state()
    ros_rate = rospy.Rate(1)
    while(not rospy.is_shutdown()):
        st.plotting()
        ros_rate.sleep()
