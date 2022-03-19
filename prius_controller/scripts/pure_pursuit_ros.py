#!/usr/bin/env python


import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path
from prius_msgs.msg import Control
from std_msgs.msg import String


# node name: path_tracking
# Publish Topic: cmd_delta
# Subscribe Topic: base_pose_ground_truth, astroid_path

max_vel = 13.0  # maximum linear velocity
global steer
k = 0.5  # constant for relating look ahead distance and velocity
wheelbase = 1.983  # wheel base for the vehicle
d_lookahead = 0.1  # look ahead distance to calculate target point on path
global n
global ep_max
global ep_sum
global ep_avg
global q

# print(("start"))
q = 0
n = 0
ep_avg = 0
ep_sum = 0
ep_max = 0
curr_steer = 0  # assuming wrt mean path
x_bot = 0
y_bot = 0
#  to be tuned
Kp = 0.1  # proportional gain
Ki = 0  # integral gain
Kpp = 0.9  # as Ki + Kpp + Kp = 1
w_current = 0.5  # to be tuned
l = 19
w_prev = 0.027  # considering l=19
# to be tuned
sum_e = 0  # for integral controller
delta_prev = []  # for storing the previous steer


def callback_feedback(data):
    '''
    Assigns the position of the robot to global variables from odometry.
    :param x_bot [float]
    :param y_bot [float]
    :param yaw [float]
    :param vel [float]
    '''
    global x_bot
    global y_bot
    global yaw
    global vel
    global curr_steer
    x_bot = data.pose.pose.position.x
    y_bot = data.pose.pose.position.y
    # quarternion to euler conversion
    siny = +2.0 * (data.pose.pose.orientation.w *
                   data.pose.pose.orientation.z +
                   data.pose.pose.orientation.x *
                   data.pose.pose.orientation.y)
    cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y *
                         data.pose.pose.orientation.y +
                         data.pose.pose.orientation.z *
                         data.pose.pose.orientation.z)
    yaw = math.atan2(siny, cosy)  # yaw in radians
    vel = data.twist.twist.linear.x * \
        math.cos(yaw) + data.twist.twist.linear.y * math.sin(yaw)
    # printing the odometry readings
    curr_steer = yaw
    # print('x of car:', x_bot)
    # print('y of car:', y_bot)
    # print('angle of car:', yaw)
    # print('vel of car:', data.twist.twist.linear.x,data.twist.twist.linear.y, data.twist.twist.linear.z)
    # print('c')


def dist(a, x, y):
    '''
    Calculates distance between two points.
    :param a [float]
    :param x [float]
    :param y [float]
    '''
    # calculate distance
    return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5


def path_length_distance(a, b):
    return (((a.pose.position.x - b.pose.position.x)**2) + ((a.pose.position.y - b.pose.position.y)**2))**0.5


def calc_path_length(data):
    global path_length
    path_length = []

    for i in range(len(data.poses)):
        if i == 0:
            path_length.append(0)

        else:
            path_length.append(
                path_length[i - 1] + path_length_distance(data.poses[i], data.poses[i - 1]))


def callback_path(data):
    '''
    calculates target path point and the steering angle
    :param data [Path]
    :param ep [float]
    :param cp [int]
    '''
    global ep  # min distance
    global cp  # index of closest point
    global ep_max
    global ep_sum
    global ep_avg
    global n
    global cp1
    global path_length
    global yaw

    cross_err = Twist()
    x_p = data
    # calculate minimum distance
    calc_path_length(x_p)

    distances = []
    for i in range(len(x_p.poses)):
        a = x_p.poses[i]
        distances += [dist(a, x_bot, y_bot)]
    ep = min(distances)
    ep1 = ep

    if (ep > ep_max):
        ep_max = ep

    n = n + 1
    ep_sum = ep_sum + ep
    ep_avg = ep_sum / n

    cp = distances.index(ep)
    publish_data = str(cp) + "+" + str(len(distances))
    pub3.publish(publish_data)
    cp1 = cp
    cross2 = [(x_bot - data.poses[cp1].pose.position.x),
              (y_bot - data.poses[cp1].pose.position.y)]
    cross = [math.cos(yaw), math.sin(yaw)]
    cross_prod = cross[0] * cross2[1] - cross[1] * cross2[0]
    if (cross_prod > 0):
        ep1 = -ep1

    # print('ep_sum: ', ep_sum)
    # print('ep_avg: ', ep_avg)
    cross_err.linear.x = ep1
    cross_err.angular.x = ep_max
    cross_err.angular.y = ep_avg

    # print('old index:', cp)
    # calculate index of target point on path
    cmd = Twist()
    cmd1 = Twist()
    prius_vel = Control()
    L = 0
    Lf = k * max_vel + d_lookahead

    while Lf > L and (cp + 1) < len(x_p.poses):
        dx = data.poses[cp + 1].pose.position.x - \
            data.poses[cp].pose.position.x
        dy = data.poses[cp + 1].pose.position.y - \
            data.poses[cp].pose.position.y
        L += math.sqrt(dx ** 2 + dy ** 2)
        cp = cp + 1
    # print(len(x_p.poses))
    # print('new index is:', cp)

    goal_point = [x_p.poses[cp].pose.position.x,
                  x_p.poses[cp].pose.position.y]

    # getting previous point
    if cp > 1:
        prev_point = [x_p.poses[cp-1].pose.position.x,
                      x_p.poses[cp-1].pose.position.y]
    # print('current goal is:', goal_point)
    # print("prev_point is: ", prev_point)
    error = [goal_point[0] - x_bot, goal_point[1] - y_bot]
    # print(error)
    steer_angle = pure_pursuit(goal_point, prev_point)

    siny = +2.0 * (x_p.poses[cp].pose.orientation.w *
                   x_p.poses[cp].pose.orientation.z +
                   x_p.poses[cp].pose.orientation.x *
                   x_p.poses[cp].pose.orientation.y)

    cosy = +1.0 - 2.0 * (x_p.poses[cp].pose.orientation.y *
                         x_p.poses[cp].pose.orientation.y +
                         x_p.poses[cp].pose.orientation.z *
                         x_p.poses[cp].pose.orientation.z)

    steer_path = math.atan2(siny, cosy)
    steer_err = (yaw - steer_path)
    cross_err.linear.y = (-1) * (yaw - steer_path)

    # print("steer_angle :", steer_angle * 180 / math.pi)
    cmd.angular.z = min(30, max(-30, steer_angle * 180 / math.pi))
    cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2)
    # print('omega:', cmd.angular.z)
    cross_err.linear.z = path_length[cp]

    #r = rospy.Rate(100)
    # while not rospy.is_shutdown():
    pub1.publish(cmd)
    pub2.publish(cross_err)
    #	r.sleep()

    # print("cmd published")

    print((ep))
    # print(x_p.poses[cp].pose.orientation)


def pure_pursuit(goal_point, prev_point):
    '''
    Calculates the steering angle required for path tracking
    :params goal_point [float,float] goal point coordinates
    :params Delta [float] steering angle in radians 
    '''
    # global x_bot, y_bot, curr_steer, wheelbase, w_prev, sum_e, delta_prev, Ki, Kp, Kpp,
    global curr_steer
    global x_bot
    global y_bot
    global Kp
    global Ki
    global Kpp
    global w_current
    global w_prev
    global sum_e
    global delta_prev
    global yaw
    tx = goal_point[0]
    ty = goal_point[1]
    px = prev_point[0]
    py = prev_point[1]
    # print('yaw:', yaw)
    # measuring the slope of path point
    # print('slope:', math.atan2(ty - y_bot, tx - x_bot))
    # measuring heading angle
    alpha = math.atan2(ty - y_bot, tx - x_bot) - yaw
    # print('alpha:', alpha)
    Lf = k * max_vel + d_lookahead
    # measuring the steering angle using pure pursuit controller
    Delta_pp = math.atan2(2.0 * wheelbase * math.sin(alpha) / Lf, 1)

    # adding the feedback part
    # adding e for integral controller
    sum_e += get_e([x_bot, y_bot], goal_point, prev_point)
    e_la = get_e([x_bot, y_bot], goal_point, prev_point) + \
        (wheelbase/2 + d_lookahead) * math.sin(Delta_pp - curr_steer)
    print("e: ", get_e([x_bot, y_bot], goal_point, prev_point))
    delta_p = Kp * e_la
    print("delta_p: ", delta_p)
    delta_i = Ki * sum_e
    print("delta_i: ", delta_i)
    delta_curr = Kpp * Delta_pp + delta_p + delta_i
    # delta_curr = Delta_pp
    print("delta_curr: ", delta_curr)
    delta = 0
    # if len(delta_prev) == l-1 :
    #     for prev in delta_prev:
    #         delta+=  w_prev * prev + w_current * delta_curr
    # else:
    #     delta = Delta_pp
    
    # if len(delta_prev) < l-1:
    #     delta_prev.append(delta)
    # elif len(delta_prev) >= l-1:
    #     delta_prev.pop(0)
    #     delta_prev.append(delta)
    # delta = w_prev * delta_prev + w_current * delta_curr
    delta = delta_curr
    print("w_curr: ", w_current)
    print("w_prev: ", w_prev)
    # delta_prev = delta
    print('delta', delta)
    pub4.publish(Delta_pp)
    return Delta_pp


def get_e(car_cord, goal_point, prev_point):
    [x1, y1] = prev_point
    [x2, y2] = goal_point
    [x_c, y_c] = car_cord
    A = (y2-y1)/(x2-x1)
    B = -1
    C = -x1*(y2-y1)/(x2-x1) + y1
    e = (A*x_c + B * y_c + C)/(math.sqrt(A**2 + B**2))
    return e


def start():
    global pub1
    global pub2
    global pub3
    global pub4
    rospy.init_node('path_tracking', anonymous=True)
    pub2 = rospy.Publisher('cross_track_error', Twist, queue_size=100)
    pub1 = rospy.Publisher('cmd_delta', Twist, queue_size=100)
    pub3 = rospy.Publisher("list", String, queue_size=100)
    pub4 = rospy.Publisher("cmd_vel_frenet", String, queue_size=10)
    rospy.Subscriber("base_pose_ground_truth", Odometry, callback_feedback)
    rospy.Subscriber("astroid_path", Path, callback_path)

    rospy.spin()


if __name__ == '__main__':
    start()