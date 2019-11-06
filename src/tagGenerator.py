#! /usr/bin/env python

import time
import numpy
import rospy
from rviz_utils import draw
from visualization_msgs.msg import Marker

# tags = numpy.array([[1.25,5.25],[1.25,3.25],[1.25,1.25],[4.25,1.25],
#         [4.25,3.25],[4.25,5.25]])

tags = [(1.25, 5.25), (1.25, 3.25), (1.25, 1.25), (4.25, 1.25), (4.25, 3.25),
            (4.25, 5.25)]

rospy.init_node("rviz_tag_generator")
rviz_draw_publisher = rospy.Publisher("rviz_draw", Marker, queue_size=1)

while not rospy.is_shutdown():
    rospy.loginfo("Publishing landmark tags to Rviz")
    draw(tags, rviz_draw_publisher, type=1, color=(1.0,1.0,0.0,0.0))
    time.sleep(2)