#!/usr/bin/env python

import rospy
import rosbag
import numpy
import math
import tf

class ROSBagParser:
    def __init__(self, rosbagPath="../data/grid.bag"):
        rospy.init_node("rosbag_parser")
        self.ROSBag = rosbag.Bag(rosbagPath)

    def __toDegrees(self, quat):
        eul = tf.transformations.euler_from_quaternion((
            quat.x,
            quat.y,
            quat.z,
            quat.w
        ))
        deg = math.degrees(eul[2])
        return deg

    def readBagFile(self):
        '''
        Message from rosbag decomposes as below:
        Movement -> msg.translation, msg.rotation1, msg.rotation2
        Observation -> msg.range, msg.bearing
        '''
        rospy.loginfo("Trying to read bag file...")
        bag = rosbag.Bag("../data/grid.bag")

        try:
            for topic, msg, time_stamp in bag.read_messages(
                    topics = ['Movements', 'Observations']):
                if (topic == "Movements"):
                    rospy.loginfo("Reading Motion Model...")
                    # TODO: Parse different values and log instead of print
                    # Parse msg.rotation1
                    rotation1 = self.__toDegrees(msg.rotation1)
                    rospy.loginfo("Rotation 1: {}".format(str(rotation1)))

                    # Parse msg.rotation2
                    rotation2 = self.__toDegrees(msg.rotation2)
                    rospy.loginfo("Rotation 2: {}".format(str(rotation2)))

                    # Parse msg.translation
                    translation = msg.translation
                    rospy.loginfo("Translation: {}m".format(str(translation)))

                elif (topic == "Observations"):
                    # TODO: Parse different values and log instead of print
                    pass
        finally:
            bag.close()


if __name__ == "__main__":
    rbParser = ROSBagParser()
    rbParser.readBagFile()
