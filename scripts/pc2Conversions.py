#!/usr/bin/env python
'''
Python script that generates cone_msgs from point clouds
Might add this to the forked fssim at some point.
'''
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from mur_common.msg import cone_msg
import numpy as np


class pc2Conversions:
    def __init__(self, topic1, topic2):
        self.sub1 = rospy.Subscriber(
            topic1, PointCloud2, self.clbk1, queue_size=1)
        self.sub2 = rospy.Subscriber(
            topic2, PointCloud2, self.clbk2, queue_size=1)

        self.pub1 = rospy.Publisher(
            topic1 + "/cone_msg", cone_msg, queue_size=10)
        self.pub2 = rospy.Publisher(
            topic2 + "/cone_msg", cone_msg, queue_size=10)

    def clbk1(self, msg):
        dat = pc2.read_points(msg,
                              field_names=("x", "y", "z"),
                              skip_nans=True)
        msg = self.convertToMessage(list(dat))
        self.pub1.publish(msg)
        return

    def clbk2(self, msg):
        dat = pc2.read_points(msg,
                              field_names=("x", "y", "z"),
                              skip_nans=True)
        arr = np.array(list(dat))
        msg = self.convertToMessage(list(dat))
        self.pub2.publish(msg)
        return

    def convertToMessage(self, data):
        msg = cone_msg()
        x_list = []
        y_list = []
        colour_list = []
        for i in range(len(data)):
            x_list.append(data[i][0])
            y_list.append(data[i][1])
            colour_list.append("Orange")
        msg.x = x_list
        msg.y = y_list
        msg.colour = colour_list
        msg.header.frame_id = "base_link"
        return msg


if __name__ == '__main__':
    rospy.init_node('pc2Conversions')
    rate = rospy.Rate(10)
    conv = pc2Conversions("/camera/cones", "/lidar/cones")
    while(not rospy.is_shutdown()):
        rate.sleep()
