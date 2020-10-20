#!/usr/bin/env python
import rospy
from mur_common.msg import cone_msg
from geometry_msgs.msg import Pose2D
import math
import random as rn
import matplotlib.pyplot as plt
import os
import csv
import pandas as pd


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
            "/mur/slam/cones", cone_msg, self.mapClb, queue_size=1)

        script_dir = os.path.dirname(__file__)
        file_path = script_dir + "/small_track_truth" + "/blue_cone.csv"
        self.df_blue = pd.read_csv(file_path, header=None, usecols=[
                                   0, 1])

        file_path = script_dir + "/small_track_truth" + "/big_cone.csv"
        self.df_big = pd.read_csv(file_path, header=None, usecols=[0, 1])
        file_path = script_dir + "/small_track_truth" + "/yellow_cone.csv"
        self.df_yellow = pd.read_csv(file_path, header=None, usecols=[0, 1])

        # self.poseSub = rospy.Subscriber(
        #     "/mur/slam/Odom", Pose2D, self.poseclb, queue_size=1)

    def poseclb(self, pose):
        self.poseEst.x = pose.x
        self.poseEst.y = pose.y
        self.poseEst.theta = pose.theta

    def mapClb(self, map):
        self.map = list(zip(map.x, map.y))
        return

    def plotting(self):
        plt.figure(1)
        plt.cla()
        axes = plt.gca()

        x = self.poseEst.x
        y = self.poseEst.y
        theta = self.poseEst.theta
        # plt.arrow(x, y,  math.cos(theta),
        #           math.sin(theta), head_width=0.05, head_length=0.1, fc='b', ec='b')
        self.df_blue.plot.scatter(x=0, y=1, ax=axes)
        self.df_yellow.plot.scatter(x=0, y=1, ax=axes)
        self.df_big.plot.scatter(x=0, y=1, ax=axes)

        for lm in self.map:
            axes.scatter(lm[0], lm[1], marker="x", c="r")

        plt.grid(True)
        plt.pause(0.0001)

        time = rospy.Time.now()
        script_dir = os.path.dirname(__file__)
        results_dir = os.path.join(script_dir, 'Results/')
        if not os.path.isdir(results_dir):
            os.makedirs(results_dir)

        # print(my_path)
        fileName = results_dir+"SlamOutput" + str(time)+".png"
        plt.savefig(fileName)


if __name__ == "__main__":
    rospy.init_node('slamOutputVisualization')
    st = state()
    ros_rate = rospy.Rate(0.1)
    while(not rospy.is_shutdown()):
        st.plotting()
        ros_rate.sleep()
