#!/usr/bin/env python
# -*- coding: utf-8 -*-

# visualization.py: code for skeleton visualization
# Author: Ravi Joshi
# Date: 2017/12/21

# import modules
import rospy
import numpy as np
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray

'''
The skeleton is considered as a combination of line strips.
Hence, the skeleton is decomposed into three LINE_STRIP as following:
	1) upper_body : [In Red Color] from head to spine base
	2) hands : [In Green Color] from left-hand tip to right-hand tip
	3) legs : [In Blue Color] from left foot to right foot

See the link below to get the id of each joint as defined in Kinect v2
source: https://msdn.microsoft.com/en-us/library/microsoft.kinect.jointtype.aspx

upper_body:
	head 3, neck 2, spine shoulder 20,
	spine mid 1, spine base 0

hands:
	hand tip left 21, hand left 7, wrist left 6, elbow left 5
	shoulder left 4, shoulder right 8, elbow right 9
	wrist right 10, hand right 11, hand tip right 23

legs:
	foot left 15, ankle left 14, knee left 13
	hip left 12, spine base 0, hip right 16
	knee right 17, ankle right 18, foot right 19
'''

class Visualization():
    def __init__(self, ns, skeleton_frame, body_id_text_size, skeleton_line_width, file_name):
        self.ns = ns
        skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=2)

        # define the colors
        colors = [ColorRGBA(0.98, 0.30, 0.30, 1.00),
                  ColorRGBA(0.12, 0.63, 0.42, 1.00),
                  ColorRGBA(0.26, 0.09, 0.91, 1.00),
                  ColorRGBA(0.77, 0.44, 0.14, 1.00),
                  ColorRGBA(0.92, 0.73, 0.14, 1.00),
                  ColorRGBA(0.00, 0.61, 0.88, 1.00),
                  ColorRGBA(1.00, 0.65, 0.60, 1.00),
                  ColorRGBA(0.59, 0.00, 0.56, 1.00)]

        body_id_color = ColorRGBA(0.62, 0.93, 0.14, 1.00)

        upper_body_ids = [3, 2, 20, 1, 0]
        hands_ids = [21, 7, 6, 5, 4, 20, 8, 9, 10, 11, 23]
        legs_ids = [15, 14, 13, 12, 0, 16, 17, 18, 19]

        # define other joint ids
        head_id = 3

		# for the demonstration, I am just using one person recorded data
		# however, it can be supplied here in real-time accquired using Kinect etc
        skeleton_joints = np.loadtxt(file_name, delimiter=',', skiprows=1)
        bodies = [skeleton_joints.tolist()]

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            marker_index = 0
            person_index = 1
            marker_array = MarkerArray()

            for body in bodies:
                now = rospy.Time.now()
                upper_body = self.create_marker(marker_index, colors[person_index], Marker.LINE_STRIP, skeleton_line_width, now, skeleton_frame)

                marker_index += 1
                hands = self.create_marker(marker_index, colors[person_index], Marker.LINE_STRIP, skeleton_line_width, now, skeleton_frame)

                marker_index += 1
                legs = self.create_marker(marker_index, colors[person_index], Marker.LINE_STRIP, skeleton_line_width, now, skeleton_frame)

                all_joints = body
                upper_body.points = [Point(all_joints[id][0], all_joints[id][1], all_joints[id][2]) for id in upper_body_ids]
                hands.points = [Point(all_joints[id][0], all_joints[id][1], all_joints[id][2]) for id in hands_ids]
                legs.points = [Point(all_joints[id][0], all_joints[id][1], all_joints[id][2]) for id in legs_ids]

                marker_index += 1
                head_id_marker = self.create_marker(marker_index, body_id_color, Marker.TEXT_VIEW_FACING, body_id_text_size, now, skeleton_frame)
                head_id_marker.text = str(person_index)
                head_id_marker.pose.position = Point(all_joints[head_id][0], all_joints[head_id][1], all_joints[head_id][2])

                marker_array.markers.append(head_id_marker)
                marker_array.markers.append(upper_body)
                marker_array.markers.append(hands)
                marker_array.markers.append(legs)

                person_index += 1

            skeleton_pub.publish(marker_array)
            rate.sleep()

    def create_marker(self, index, color, marker_type, size, time, frame_id):
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.header.stamp = time
        marker.header.frame_id = frame_id
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker

if __name__ == '__main__':
    # define some constants
    ns = 'visualization'
    skeleton_frame = 'base'
    body_id_text_size = 0.2
    skeleton_line_width = 0.02

    # initialize ros node
    rospy.init_node(ns, anonymous=True)
    Visualization(ns, skeleton_frame, body_id_text_size, skeleton_line_width, rospy.get_param('~file'))
