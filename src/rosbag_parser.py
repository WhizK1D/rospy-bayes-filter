#!/usr/bin/env python

import time
import rospy
import rosbag
import numpy
import math
import tf
from rviz_utils import *


class ROSBagParser:
    '''
    Simple class to parse the bag file and spit out the messages
    from the same for other nodes to consume
    '''
    def __init__(self, rosbagPath="../data/grid.bag"):
        rospy.init_node("rosbag_parser")
        self.ROSBag = rosbag.Bag(rosbagPath)
        # self.tagArray = numpy.array([(1.25, 5.25), (1.25, 3.25), (1.25, 1.25),
        #         (4.25, 1.25), (4.25, 3.25), (4.25, 5.25)])
        self.tagArray = numpy.array([(125, 525), (125, 325), (125, 125),
                (425, 125), (425, 325), (425, 525)])
        self.cellSize = 20
        self.headingSize = 90
        self.translationGaussian = (1.0/(numpy.sqrt(2*numpy.pi)*10))
        self.rotationGaussian = (1.0/(numpy.sqrt(2*numpy.pi)*18))
        self.currentPose = numpy.zeros((35, 35, 10))
        self.currentPose[11, 27, 3] = 1.0
        self.prob = 0

    def __toDegrees(self, quat):
        eul = tf.transformations.euler_from_quaternion((
            quat.x,
            quat.y,
            quat.z,
            quat.w
        ))
        return math.degrees(eul[2])

    def __simpleAngle(self, angle):
        if angle > 180:
            return (angle - 360)
        elif angle < (-180):
            return (angle + 360)
        else:
            return (angle)

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
                    # rospy.loginfo("Reading Motion Model...")

                    # Parse msg.rotation1
                    rotation1 = self.__toDegrees(msg.rotation1)
                    # rospy.loginfo("Rotation 1: {}".format(str(rotation1))) #DEBUG

                    # Parse msg.rotation2
                    rotation2 = self.__toDegrees(msg.rotation2)
                    # rospy.loginfo("Rotation 2: {}".format(str(rotation2))) #DEBUG

                    # Parse msg.translation
                    translation = msg.translation
                    # rospy.loginfo("Translation: {} m ({} cm)".format(
                    #     str(translation), str(translation*100))) #DEBUG

                    # start_time = time.time() #DEBUG
                    self.predict(rotation1, translation, rotation2)
                    # print("--- %s seconds ---" % (time.time() - start_time)) #DEBUG

                elif (topic == "Observations"):
                    # rospy.loginfo("Reading Sensor Model...")

                    # Parse tag number
                    tagNumber = msg.tagNum
                    # rospy.loginfo("Tag No.: {}".format(str(tagNumber))) #DEBUG

                    # Parse range
                    range = (msg.range) * 100
                    # rospy.loginfo("Range: {}".format(range)) #DEBUG

                    # Parse bearing
                    bearing = self.__toDegrees(msg.bearing)
                    # rospy.loginfo("Bearing:\n{}".format(str(bearing))) #DEBUG

                    self.update(range, bearing, tagNumber)

        finally:
            self.ROSBag.close()

    def predict(self, rot1, trans, rot2):
        '''
        Implement the prediction step for Bayes filter
        '''
        tempPose = self.currentPose
        updatedPose = numpy.zeros(self.currentPose.shape, dtype=numpy.float)
        self.currentPose = numpy.copy(tempPose)

        finalProb = 0.0
        i, j, k = 0, 0, 0
        for (x, y) in numpy.ndenumerate(self.currentPose):
            if y > 0.1:
                i = x[0]
                j = x[1]
                k = x[2]
                break

        for (x, y) in numpy.ndenumerate(self.currentPose):
            l1 = x[0] * 20 + 10
            m1 = x[1] * 20 + 10
            n1 = x[2] * 36 + 18

            i1 = i*20 + 10
            j1 = j*20 + 10
            k1 = k*36 + 18

            translationDiff = numpy.sqrt((l1 - i1)**2 + (m1 - j1)**2)
            angularDiff = numpy.degrees(numpy.arctan2(m1 - j1, l1 - i1))
            r1 = self.__simpleAngle(n1 - angularDiff)
            # print(r1) #DEBUG
            r2 = self.__simpleAngle(angularDiff - k1)
            # print(r2) #DEBUG

            translationNew = self.translationGaussian * numpy.power(numpy.e,
                    -1.0 * (((translationDiff - trans)**2)/(2.0* 10**2)))
            rotation1New = self.rotationGaussian * numpy.power(numpy.e,
                    -1.0 * (((r1 - rot1)**2)/(2.0* 18**2)))
            rotation2New = self.rotationGaussian * numpy.power(numpy.e,
                    -1.0 * (((r2 - rot2)**2)/(2.0* 18**2)))

            newValue = tempPose[i, j, k] * translationNew * rotation1New * rotation2New
            updatedPose[x[0], x[1], x[2]] = self.currentPose[x[0], x[1], x[2]] + newValue

            finalProb = finalProb + newValue

        self.currentPose = updatedPose / finalProb

    def update(self, range, bearing, tagNumber):
        '''
        Implement the update step for Bayes filter
        range = translation in m
        bearing = rotation in degrees
        '''
        tempPose = numpy.zeros(self.currentPose.shape, dtype=numpy.float)
        sensorProbValue = 0
        finalSensorProb = 0
        sensorPose = self.currentPose

        for (x, y) in numpy.ndenumerate(self.currentPose):
            translationX = x[0] * 20 + 10
            translationY = x[1] * 20 + 10
            rotation = x[2] * 36 + 18

            angle = numpy.degrees(numpy.arctan2(
                self.tagArray[tagNumber, 1] -translationY,
                self.tagArray[tagNumber, 0] - translationX))

            translationDiff = numpy.sqrt(
                (self.tagArray[tagNumber, 1] - translationY)**2 +
                (self.tagArray[tagNumber, 0] - translationX)**2)

            rotationDiff = self.__simpleAngle(angle - rotation)

            translationNewProb = self.translationGaussian * numpy.power(
                numpy.e, -1.0 * (((translationDiff - range)**2)/
                (2.0* 10**2)))

            rotationNewProb = self.rotationGaussian * numpy.power(
                numpy.e, -1.0 * (((rotationDiff - bearing)**2)/
                (2.0* 18**2)))

            sensorProbValue = sensorPose[x[0], x[1], x[2]] * translationNewProb * rotationNewProb

            tempPose[x[0], x[1], x[2]] = sensorProbValue

            finalSensorProb = finalSensorProb + sensorProbValue

        self.currentPose = tempPose / finalSensorProb

        (x, y, z) = numpy.unravel_index(self.currentPose.argmax(),
                self.currentPose.shape)

        print("Grid Cell: %d %d %d" % (x+1, y+1, z))
        print("Probability: %f" % self.currentPose[x, y, z])
        self.prob = self.currentPose[x, y, z]
        #TODO: Write to file and publish markers for Rviz


if __name__ == "__main__":
    rbParser = ROSBagParser()
    start_time = time.time()
    rbParser.readBagFile()
    print("Probability: %f" % (rbParser.prob))
    print("--- %s seconds ---" % (time.time() - start_time))
