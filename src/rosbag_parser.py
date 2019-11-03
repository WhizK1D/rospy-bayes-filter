#!/usr/bin/env python

import rospy
import rosbag
import numpy
import math
import tf
import timeit


class ROSBagParser:
    '''
    Simple class to parse the bag file and spit out the messages
    from the same for other nodes to consume
    '''
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
        Observation -> msg.range, msg.bearing, msg.tagNum

        Once decomposed, these would be published to respective movements
        and observations topics to be read by motion and sensor models
        respectively.
        '''
        rospy.loginfo("Trying to read bag file...")

        try:
            for topic, msg, _time_stamp in self.ROSBag.read_messages(
                    topics = ['Movements', 'Observations']):

                if (topic == "Movements"):
                    rospy.loginfo("Reading Motion Model...")

                    # Parse msg.rotation1
                    rotation1 = self.__toDegrees(msg.rotation1)
                    rospy.loginfo("Rotation 1: {}".format(str(rotation1)))

                    # Parse msg.rotation2
                    rotation2 = self.__toDegrees(msg.rotation2)
                    rospy.loginfo("Rotation 2: {}".format(str(rotation2)))

                    # Parse msg.translation
                    translation = msg.translation
                    rospy.loginfo("Translation: {} m ({} cm)".format(
                        str(translation), str(translation*100)))

                    # TODO: Publish data to motion model

                elif (topic == "Observations"):
                    rospy.loginfo("Reading Sensor Model...")

                    # Parse tag number
                    tagNumber = msg.tagNum
                    rospy.loginfo("Tag No.: {}".format(str(tagNumber)))

                    # Parse range
                    range = msg.range
                    rospy.loginfo("Range: {}".format(range))

                    # Parse bearing
                    bearing = msg.bearing
                    rospy.loginfo("Bearing:\n{}".format(str(bearing)))

                    # TODO: Publish data to sensor model

        finally:
            self.ROSBag.close()


if __name__ == "__main__":
    rbParser = ROSBagParser()
    timeit.timeit()
