#!/usr/bin/env python

import time
import rospy
import rosbag
import numpy
import math
import tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import rospkg
from rviz_utils import *


class ROSBagParser:
    '''
    Simple class to parse the bag file and spit out the messages
    from the same for other nodes to consume
    '''
    def __init__(self, rosbagPath="/data/grid.bag"):
        '''
        Constructor with optional path for specifying grid bag file
        '''

        # Initializations for ROS Bag and landmark tags
        basePath = rospkg.RosPack().get_path('ros_pa3')
        self.ROSBag = rosbag.Bag(basePath + rosbagPath)
        self.outputFile = basePath + "/output/estimations.txt"
        self.tagArray = numpy.array([(125, 525), (125, 325), (125, 125),
                (425, 125), (425, 325), (425, 525)])

        # Constant initializations for Bayes filter
        self.CELL_SIZE = 20
        self.ANGULAR_DISCRETIZATION = 24
        self.PROBABILITY_THRESHOLD = 0.1
        self.ANGULAR_SIZE = float(360)/self.ANGULAR_DISCRETIZATION

        self.TRANSLATION_MEAN = self.CELL_SIZE/2
        self.ANGULAR_MEAN = self.ANGULAR_SIZE/2

        self.translationGaussian = (1.0/(numpy.sqrt(2*numpy.pi) * self.TRANSLATION_MEAN))
        self.rotationGaussian = (1.0/(numpy.sqrt(2*numpy.pi) * self.ANGULAR_MEAN))
        self.currentPose = numpy.zeros((35, 35, self.ANGULAR_DISCRETIZATION))
        self.currentPose[11, 27, 3] = 1.0
        self.prob = 0.0

        # RViz related initializations
        rospy.init_node('bayes_localization', anonymous=False)
        self.rviz_draw_publisher = rospy.Publisher("rviz_draw", Marker,
                queue_size=1)
        self.rvizMarkerList = [(2.3, 5.5)]
        self.addToMarkerList((12, 28))
        self.drawRvizMarkers()

        # Display important initializations
        rospy.loginfo("Cell Size: {}".format(str(self.CELL_SIZE)))
        rospy.loginfo("Angular Discretization: {}".format(str(self.ANGULAR_DISCRETIZATION)))
        rospy.loginfo("Angular Size: {}".format(str(self.ANGULAR_SIZE)))
        rospy.loginfo("Translation Mean: {}".format(str(self.TRANSLATION_MEAN)))
        rospy.loginfo("Angular Mean: {}".format(str(self.ANGULAR_MEAN)))
        rospy.loginfo("Grid Bag File Info:\n{}".format(str(self.ROSBag)))
        rospy.loginfo("Output File Path: {}".format(self.outputFile))

    def __toDegrees(self, quat):
        '''
        Private function to convert quaternion to Euler form and
        return degrees on z-axis
        '''
        eul = tf.transformations.euler_from_quaternion((
            quat.x,
            quat.y,
            quat.z,
            quat.w
        ))
        return math.degrees(eul[2])

    def __simpleAngle(self, angle):
        '''
        Return a simplistic form of angle in degrees; convert
        [0, 360] domain to [-180, 180] domain
        '''
        if angle > 180:
            return (angle - 360)
        elif angle < (-180):
            return (angle + 360)
        else:
            return (angle)

    def addToMarkerList(self, point):
        '''
        Pass a tuple (x, y) that represents a point to be added
        into the RViz Marker List
        '''

        # Convert to continuous domain before publishing to RViz
        x = float(point[0] - 0.5)/5.0
        y = float(point[1] - 0.5)/5.0
        self.rvizMarkerList.append((x, y))

    def writeToOutputFile(self, x, y, theta, type):
        '''
        x = X coordinate in m
        y = Y coordinate in m
        theta = head pose in degrees
        type = "P"/"U"
        '''

        # Convert X, Y, Theta to continuous domain before writing
        xm = float(x - 0.5)/5.0 # x in metres
        ym = float(y - 0.5)/5.0 # y in metres
        thetad = (theta - 1) * self.ANGULAR_SIZE + self.ANGULAR_MEAN
        output = "{}: ({}, {}, {})\n".format(type, str(xm), str(ym),
                str(self.__simpleAngle(thetad)))
        file = open(self.outputFile, "a")
        file.write(output)
        file.close()

    def drawRvizMarkers(self):
        '''
        Publishes the Rviz markers at any given stage to RViz
        '''
        draw(self.rvizMarkerList, self.rviz_draw_publisher)

    def readBagFile(self):
        '''
        Message from rosbag decomposes as below:
        Movement -> msg.translation, msg.rotation1, msg.rotation2
        Observation -> msg.range, msg.bearing, msg.tagNum

        Once decomposed, these would be published to respective movements
        and observations topics to be read by motion and sensor models
        respectively.
        '''
        rospy.loginfo("Processing bag file...")

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
                    # print("Probability of current cell: %f" % self.prob) #DEBUG

        finally:
            self.ROSBag.close()

    def predict(self, rot1, trans, rot2):
        '''
        Implement the prediction step (motion model) for Bayes filter
        '''
        tempPose = self.currentPose
        updatedPose = numpy.zeros(self.currentPose.shape, dtype=numpy.float)
        self.currentPose = numpy.copy(tempPose)

        finalProb = 0.0
        i, j, k = 0, 0, 0
        for (x, y) in numpy.ndenumerate(self.currentPose):
            if y > self.PROBABILITY_THRESHOLD:
                i = x[0]
                j = x[1]
                k = x[2]
                break

        for (x, y) in numpy.ndenumerate(self.currentPose):
            l1 = x[0] * self.CELL_SIZE + self.TRANSLATION_MEAN
            m1 = x[1] * self.CELL_SIZE + self.TRANSLATION_MEAN
            n1 = x[2] * self.ANGULAR_SIZE + self.ANGULAR_MEAN

            i1 = i*self.CELL_SIZE + self.TRANSLATION_MEAN
            j1 = j*self.CELL_SIZE + self.TRANSLATION_MEAN
            k1 = k*self.ANGULAR_SIZE + self.ANGULAR_MEAN

            translationDiff = numpy.sqrt((l1 - i1)**2 + (m1 - j1)**2)
            angularDiff = numpy.degrees(numpy.arctan2(m1 - j1, l1 - i1))
            r1 = self.__simpleAngle(n1 - angularDiff)
            r2 = self.__simpleAngle(angularDiff - k1)

            translationNew = self.translationGaussian * numpy.power(numpy.e,
                    -1.0 * (((translationDiff - trans)**2)/(2.0* self.TRANSLATION_MEAN**2)))
            rotation1New = self.rotationGaussian * numpy.power(numpy.e,
                    -1.0 * (((r1 - rot1)**2)/(2.0* self.ANGULAR_MEAN**2)))
            rotation2New = self.rotationGaussian * numpy.power(numpy.e,
                    -1.0 * (((r2 - rot2)**2)/(2.0* self.ANGULAR_MEAN**2)))

            newValue = tempPose[i, j, k] * translationNew * rotation1New * rotation2New
            updatedPose[x[0], x[1], x[2]] = self.currentPose[x[0], x[1], x[2]] + newValue

            finalProb = finalProb + newValue
            if y > self.PROBABILITY_THRESHOLD:
                X = x[0]
                Y = x[1]
                Z = x[2]

        self.currentPose = updatedPose / finalProb
        self.prob = self.currentPose[X, Y, Z]

        # Note: Only write to file since predict causes more disbelief :P
        self.writeToOutputFile(X+1, Y+1, Z+1, "P")
        self.drawRvizMarkers()

    def update(self, range, bearing, tagNumber):
        '''
        Implement the update step (sensor model) for Bayes filter
        range = translation in m
        bearing = rotation in degrees
        '''
        tempPose = numpy.zeros(self.currentPose.shape, dtype=numpy.float)
        sensorProbValue = 0
        finalSensorProb = 0
        sensorPose = self.currentPose

        for (x, y) in numpy.ndenumerate(self.currentPose):
            translationX = x[0] * self.CELL_SIZE + self.TRANSLATION_MEAN
            translationY = x[1] * self.CELL_SIZE + self.TRANSLATION_MEAN
            rotation = x[2] * self.ANGULAR_SIZE + self.ANGULAR_MEAN

            angle = numpy.degrees(numpy.arctan2(
                self.tagArray[tagNumber, 1] - translationY,
                self.tagArray[tagNumber, 0] - translationX))

            translationDiff = numpy.sqrt(
                (self.tagArray[tagNumber, 1] - translationY)**2 +
                (self.tagArray[tagNumber, 0] - translationX)**2)

            rotationDiff = self.__simpleAngle(angle - rotation)

            translationNewProb = self.translationGaussian * numpy.power(
                numpy.e, -1.0 * (((translationDiff - range)**2)/
                (2.0* self.TRANSLATION_MEAN**2)))

            rotationNewProb = self.rotationGaussian * numpy.power(
                numpy.e, -1.0 * (((rotationDiff - bearing)**2)/
                (2.0* self.ANGULAR_MEAN**2)))

            sensorProbValue = sensorPose[x[0], x[1], x[2]] * translationNewProb * rotationNewProb

            tempPose[x[0], x[1], x[2]] = sensorProbValue

            finalSensorProb = finalSensorProb + sensorProbValue

        self.currentPose = tempPose / finalSensorProb

        (x, y, z) = numpy.unravel_index(self.currentPose.argmax(),
                self.currentPose.shape)

        self.prob = self.currentPose[x, y, z]

        # Publish to RViz and write to output file
        self.addToMarkerList((x+1, y+1, z))
        self.addToMarkerList((x+1, y+1, z))
        self.writeToOutputFile(x+1, y+1, z+1, "U")
        self.drawRvizMarkers()


if __name__ == "__main__":
    rbParser = ROSBagParser()
    start_time = time.time()
    rbParser.readBagFile()
    rospy.loginfo("Probability of last cell: {}".format(str(rbParser.prob)))
    rospy.loginfo("Total Runtime: {}".format(str(time.time() - start_time)))
