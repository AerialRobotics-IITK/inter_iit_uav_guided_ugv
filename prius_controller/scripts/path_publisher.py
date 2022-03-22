#!/usr/bin/env python


import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import pandas as pd
import csv
import math


def frange(x, y, jump):
    '''
    finds the set of points for path generation using fixed jump parameters.
    :params x : initial parameter value
    :params y : final parameter value
    :params jump : constant jump in parameter
    '''
    while x < y:
        yield x
        x += jump


def set_params(x_offset, y_offset):
    '''
    Function to set params for path offset optionally

    :params x_offset : Offset for x coordinate
    :params y_offset : Offset for y coordinate

    '''
    print('Do you want to change the path offset')
    if raw_input('Press y to change else press n : ') == 'y':
        x_offset = raw_input('enter x offset:')
        y_offset = raw_input('enter y offset:')

    return x_offset, y_offset


x_g = []
y_g = []
psi_g = []


def main():
    '''
    gets path coordinates and publishes them in form of an array.

    '''
    global x_g
    global y_g
    global psi_g

    x_offset = 0
    y_offset = 0
    x_offset, y_offset = set_params(x_offset, y_offset)
    rospy.init_node('astroid_curve_publisher')

    path_pub = rospy.Publisher('astroid_path', Path, queue_size=100)
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

    f = open("./world2.csv", 'r')
    c =[]
    for p in csv.reader(f):
        c.append(p)
    # for t in frange(0, 200, resolution):
    for t in range(len(c)):
        # if t < 100:
        #     # some offset can be used to ensure according to test conditions
        #     x = int(offset_x) + t
        #     y = int(offset_y)
        # else:
        #     y = y + t - 100
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
    xnp = np.asarray(x_g)
    psinp = np.asarray(psi_g)
    ynp = np.asarray(y_g)
    pd.DataFrame(xnp).to_csv("xnp.csv",  mode='w', header=False)
    pd.DataFrame(psinp).to_csv("psinp.csv",  mode='w', header=False)
    pd.DataFrame(ynp).to_csv("ynp.csv",  mode='w', header=False)
    print("SAVING TO FILE")
