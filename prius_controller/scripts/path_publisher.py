#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
import csv
import math
import os


x_g = []
y_g = []
psi_g = []


def main():
    global x_g
    global y_g
    global psi_g

    x_offset = 0
    y_offset = 0
    print("Start")
    rospy.init_node('waypoint_publisher')

    path_pub = rospy.Publisher('way_path', Path, queue_size=100)
    path = Path()

    path.header.frame_id = rospy.get_param('~output_frame', 'map')
    radius = rospy.get_param('~radius', 50.0)  # radius of path
    # constant jump value for parameter
    resolution = rospy.get_param('~resolution', 0.1)
    holonomic = rospy.get_param('~holonomic', False)
    # get x offset from params
    offset_x = rospy.get_param('~offset_x', x_offset)
    # get y offset from params
    offset_y = rospy.get_param('~offset_y', y_offset)
    update_rate = rospy.get_param(
        '~update_rate', 100)  # rate of path publishing
    has_initialize = True
    # loop to get the path coordinates
    file_path = os.path.abspath(os.path.dirname(__file__))
    print("Enter world number like 1 or 2")
    world_number = input("Enter World Name: ")
    world_name = "world" + str(world_number) +".csv"
    path_to_csv = os.path.realpath(os.path.join(file_path, "..", "..","maps", world_name ))
    print(path_to_csv)
    f = open(path_to_csv, 'r')
    c = []
    for p in csv.reader(f):
        c.append(p)
    # for t in frange(0, 200, resolution):
    for t in range(len(c)):
        x = int(offset_x) + t
        y = 2 * math.sin(x/20) + int(offset_y)
        if has_initialize:
            old_x = x
            old_y = y
            has_initialize = False

        pose = PoseStamped()
        pose.pose.position.x = float(c[t][0])
        pose.pose.position.y = float(c[t][1])

        yaw = 0.0
        if holonomic:
            yaw = -math.sin(t) / math.cos(t)
        else:
            if (-math.pi/2 <= t <= 0) or (math.pi/2 <= t <= math.pi):
                yaw = math.atan2(old_y - y, old_x - x)
            else:
                yaw = math.atan2(y - old_y, x - old_x)

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        path.poses.append(pose)
        x_g.append(pose.pose.position.x)
        y_g.append(pose.pose.position.y)
        psi_g.append(yaw)

        old_x = x
        old_y = y

    r = rospy.Rate(update_rate)
    while not rospy.is_shutdown():
        path.header.stamp = rospy.get_rostime()
        path_pub.publish(path)

        r.sleep()


if __name__ == '__main__':
    main()
