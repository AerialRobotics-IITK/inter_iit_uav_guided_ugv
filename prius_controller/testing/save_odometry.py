#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
import csv

csv_file = open("world1.csv", "w+")
c = csv.writer(csv_file)
time = 0


def callback_feedback(msg):
    # index = 0    # while not msg.name[index] == "husky":
    #   index +=1
    global time
    time += 1
    if time % 80 == 0:
        index = 1
        c.writerow([msg.pose[index].position.x,
                   msg.pose[index].position.y, msg.pose[index].position.z])
        print([msg.pose[index].position.x,
              msg.pose[index].position.y, msg.pose[index].position.z])
    # platform_odom_.pose.pose.position.x = msg.pose[index - 1].position.x
    # platform_odom_.pose.pose.position.y = msg.pose[index - 1].position.y


def start():
    rospy.init_node('controls', anonymous=True)

    rospy.Subscriber("/gazebo/model_states",
                     ModelStates, callback_feedback)
    rospy.spin()


if __name__ == '__main__':
    start()
