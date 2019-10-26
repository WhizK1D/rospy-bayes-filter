#!/usr/bin/env python

import rospy
import rosbag
import numpy


def readBagFile():
    rospy.loginfo("Trying to read bag file...")
    bag = rosbag.Bag("../data/grid.bag")

    try:
        for topic, msg, time_stamp in bag.read_messages(topics = ['Movements', 'Observations']):
            if (topic == "Movements"):
                rospy.loginfo("Reading Motion Model...")
                # TODO: Parse different values and log instead of print
                print(msg)
            elif (topic == "Observations"):
                rospy.loginfo("Reading Sensor Model...")
                # TODO: Parse different values and log instead of print
                print(msg)
    finally:
        bag.close()


if __name__ == "__main__":
    rospy.init_node("rosbag_parser", anonymous=False)
    readBagFile()
