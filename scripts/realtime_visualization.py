#!/usr/bin/env python
# -*- coding: utf-8 -*-

# realtime_visualization.py: rviz visualization
# Author: Ravi Joshi
# Date: 2018/02/07

# import modules
import rospy
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3
from kinect_anywhere.msg import BodyFrame
from visualization_msgs.msg import Marker, MarkerArray


class RealtimeVisualization():
    def __init__(self, ns, body_topic, skeleton_frame, body_id_text_size, skeleton_line_width):
        self.ns = ns
        self.skeleton_frame = skeleton_frame

        self.body_id_text_size = float(body_id_text_size)
        self.skeleton_line_width = float(skeleton_line_width)

        self.skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=2)

        # define the colors
        self.colors = [ColorRGBA(0.98, 0.30, 0.30, 1.00),
                       ColorRGBA(0.12, 0.63, 0.42, 1.00),
                       ColorRGBA(0.26, 0.09, 0.91, 1.00),
                       ColorRGBA(0.77, 0.44, 0.14, 1.00),
                       ColorRGBA(0.92, 0.73, 0.14, 1.00),
                       ColorRGBA(0.00, 0.61, 0.88, 1.00),
                       ColorRGBA(1.00, 0.65, 0.60, 1.00),
                       ColorRGBA(0.59, 0.00, 0.56, 1.00)]

        self.body_id_color = ColorRGBA(0.62, 0.93, 0.14, 1.00)

        '''
        The skeleton is considered as a combination of line strips.
        Hence, the skeleton is decomposed into three LINE_STRIP as following:
            1) upper_body : from head to spine base
            2) hands : from left-hand tip to right-hand tip
            3) legs : from left foot to right foot

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
        self.upper_body_ids = [3, 2, 20, 1, 0]
        self.hands_ids = [21, 7, 6, 5, 4, 20, 8, 9, 10, 11, 23]
        self.legs_ids = [15, 14, 13, 12, 0, 16, 17, 18, 19]

        # define other joint ids
        self.head_id = 3

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(body_topic, BodyFrame, self.receive_skeleton_callback)
        rospy.spin()

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

    def receive_skeleton_callback(self, data):
        marker_index = 0
        person_index = 1
        marker_array = MarkerArray()

        if not data.bodies:
            rospy.logdebug('No body tracked')
        else:
            for body in data.bodies:
                now = rospy.Time.now()

                marker_index += 1
                upper_body = self.create_marker(
                    marker_index,
                    self.colors[person_index],
                    Marker.LINE_STRIP,
                    self.skeleton_line_width,
                    now,
                    self.skeleton_frame)

                marker_index += 1
                hands = self.create_marker(
                    marker_index,
                    self.colors[person_index],
                    Marker.LINE_STRIP,
                    self.skeleton_line_width,
                    now,
                    self.skeleton_frame)

                marker_index += 1
                legs = self.create_marker(
                    marker_index,
                    self.colors[person_index],
                    Marker.LINE_STRIP,
                    self.skeleton_line_width,
                    now,
                    self.skeleton_frame)

                upper_body.points = [
                    body.jointPositions[id].position for id in self.upper_body_ids]
                hands.points = [
                    body.jointPositions[id].position for id in self.hands_ids]
                legs.points = [
                    body.jointPositions[id].position for id in self.legs_ids]

                marker_index += 1
                head_id_marker = self.create_marker(
                    marker_index,
                    self.body_id_color,
                    Marker.TEXT_VIEW_FACING,
                    self.body_id_text_size,
                    now,
                    self.skeleton_frame)
                head_id_marker.text = str(person_index)
                head_id_marker.pose.position = body.jointPositions[self.head_id].position

                marker_array.markers.append(head_id_marker)
                marker_array.markers.append(upper_body)
                marker_array.markers.append(hands)
                marker_array.markers.append(legs)

                person_index += 1
        self.skeleton_pub.publish(marker_array)

    def __del__(self):
        self.skeleton_pub.unregister()


if __name__ == '__main__':
    # define some constants
    ns = 'realtime_visualization'

    # initialize ros node
    rospy.init_node('rviz_visualization', anonymous=True)
    body_topic = rospy.get_param('~body_topic')
    skeleton_frame = rospy.get_param('~skeleton_frame')
    body_id_text_size = rospy.get_param('~body_id_text_size')
    skeleton_line_width = rospy.get_param('~skeleton_line_width')

    RealtimeVisualization(ns, body_topic, skeleton_frame,
                          body_id_text_size, skeleton_line_width)
