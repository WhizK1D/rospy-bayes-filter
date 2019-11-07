#! /usr/bin/env python

import time
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

'''
Simple utils file for publishing RViz markers with some customizations.
Based on basic example provided in scripts/markers_example.py
'''


def draw(points, viz_publisher, type=0, color=(1.0, 0.0, 1.0, 0.5)):
    '''
    Function to publish messages for drawing a line between 2 points

    type = 0 for line (default)
    type = 1 for cube
    '''

    # Initialize the marker to publish
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.id = type # Different ID so that cubes are not overwritten by lines
    if type == 1:
        marker.type = marker.CUBE_LIST
    else:
        marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # Set marker scales
    if type == 1:
        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15
    else:
        marker.scale.x = 0.02
        marker.scale.y = 0.02
        marker.scale.z = 0.01

    # Set marker colors
    marker.color.a = color[0]
    marker.color.r = color[1]
    marker.color.g = color[2]
    marker.color.b = color[3]

    # Set marker pose
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    # Generate the list of markers
    marker.points = []
    for point in points:
        marker_point = Point()
        marker_point.x = point[0]
        marker_point.y = point[1]
        marker_point.z = 0.0
        marker.points.append(marker_point)

    viz_publisher.publish(marker)

if __name__ == "__main__":
    # Example Input: Set of points to draw a square
    line_points = [(0,0), (0,0.5), (0,0.5), (0.5,0.5), (0.5,0.5), (0.5,0), (0.5,0), (0,0)]

    # Example Input: Set of cubes at four positions
    cube_points = [(0,0), (0,1), (1,0), (1,1)]

    rospy.init_node("rviz_utils_tester", anonymous=False)
    rviz_draw_publisher = rospy.Publisher('rviz_draw', Marker, queue_size=1)
    rviz_cube_publisher = rospy.Publisher('rviz_draw', Marker, queue_size=1)
    while not rospy.is_shutdown():
        rospy.loginfo("Publishing line list")
        draw(line_points, rviz_draw_publisher)
        time.sleep(5)
        rospy.loginfo("Publishing cube list")
        draw(cube_points, rviz_cube_publisher, type=1, color=(1.0, 1.0, 0.0, 0.0))
        time.sleep(5)