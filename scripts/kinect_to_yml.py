#!/usr/bin/env python
# -*- coding: utf-8 -*-

# kinect_to_yml.py: store kinect tracking data into yml file
# Author: Ravi Joshi
# Date: 2018/11/30

# import modules
import yaml
import rospy
import rospkg
import argparse
from os.path import join
from kinect_anywhere.msg import BodyFrame

parser = argparse.ArgumentParser()
parser.add_argument('--count', type=int, default=0,
                    help='counter to create dynamic file name [default: 0]')
parser.add_argument('--topic', default='/irex/kinect2/body_frame/bodies',
                    help='name of the rostopic [default: /irex/kinect2/body_frame/bodies]')
args = parser.parse_args()

topic_name = args.topic
count = args.count

# joints info is taken from kinect website as mentioned below
# https://docs.microsoft.com/en-us/previous-versions/windows/kinect/dn758662%28v%3dieb.10%29
joints_name_id = {'AnkleLeft': 14,
                  'AnkleRight': 18,
                  'ElbowLeft': 5,
                  'ElbowRight': 9,
                  'FootLeft': 15,
                  'FootRight': 19,
                  'HandLeft': 7,
                  'HandRight': 11,
                  'HandTipLeft': 21,
                  'HandTipRight': 23,
                  'Head': 3,
                  'HipLeft': 12,
                  'HipRight': 16,
                  'KneeLeft': 13,
                  'KneeRight': 17,
                  'Neck': 2,
                  'ShoulderLeft': 4,
                  'ShoulderRight': 8,
                  'SpineBase': 0,
                  'SpineMid': 1,
                  'SpineShoulder': 20,
                  'ThumbLeft': 22,
                  'ThumbRight': 24,
                  'WristLeft': 6,
                  'WristRight': 10}

# create a dictionry where key is joint id and value is joint name
joints_id_name = {joints_name_id[name]: name for name in joints_name_id}


def callback(data):
    # considering only one person
    body = data.bodies[0]
    # create a dictionry where key is joint name and value is joint position
    joint_info = {joints_id_name[joint.jointType]:
                  [joint.position.x, joint.position.y, joint.position.z]
                  for joint in body.jointPositions}

    # write every 10th frame to a yaml file
    global count
    if count % 10 == 0:
        file_name = join(project_location, 'files', 'skeleton_%d.yml' % count)
        with open(file_name, 'w') as yml_file:
            yaml.dump(joint_info, yml_file)

    # increment the counter
    count += 1


if __name__ == '__main__':
    rospy.init_node('kinect_to_yml', anonymous=True)
    project_location = rospkg.RosPack().get_path('rviz_skeleton_visualization')
    rospy.Subscriber(topic_name, BodyFrame, callback)
    rospy.spin()
