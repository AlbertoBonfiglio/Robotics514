#!/usr/bin/env python

import rospy
import math
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

key_mapping = {'w': [0, 1], 'x': [0, -1], 'a': [1, 0], 'd': [-1, 0], 's': [0, 0], 'q': [0, 0]}
g_twist_pub = None
g_target_twist = None
g_last_twist = None
g_last_send_time = None
g_vel_scales = [0.1, 0.1]  # default to very slow
g_vel_ramps = [1, 1]  # units: meters per second^2

g_home_pose = None

_scanArc = 30
g_scan_distance = 0.5  # defaults to stop if 50cm from obstacle


def ramped_vel(v_prev, v_target, t_prev, t_now, ramp_rate):
    # compute maximum velocity step
    step = ramp_rate * (t_now - t_prev).to_sec()
    sign = 1 if (v_target > v_prev) else -1
    error = math.fabs(v_target - v_prev)
    if error < step:  # we can get there within this timestep. we're done.
        return v_target
    else:
        return v_prev + sign * step  # take a step towards the target


def ramped_twist(prev, target, t_prev, t_now, ramps):
    tw = Twist()
    tw.angular.z = ramped_vel(prev.angular.z, target.angular.z, t_prev, t_now, ramps[0])
    tw.linear.x = ramped_vel(prev.linear.x, target.linear.x, t_prev, t_now, ramps[1])
    return tw


def send_twist():
    global g_last_twist_send_time, g_target_twist, g_last_twist, g_vel_scales, g_vel_ramps, g_twist_pub
    t_now = rospy.Time.now()
    g_last_twist = ramped_twist(g_last_twist, g_target_twist, g_last_twist_send_time, t_now, g_vel_ramps)
    g_last_twist_send_time = t_now
    g_twist_pub.publish(g_last_twist)


def keys_cb(msg):
    global g_target_twist, g_last_twist, g_vel_scales
    if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
        return  # unknown key.

    print "Key pressed " + str(msg.data[0])
    if msg.data[0] == 'q':
        # set in autonomous mode
        go_home()
        print "I like to move it move it.... autonomously"
    elif msg.data[0] == 's':
        brake()
    else:
        vels = key_mapping[msg.data[0]]
        g_target_twist.angular.z += (vels[0] * g_vel_scales[0])
        if not obstacle_detected():
            g_target_twist.linear.x += (vels[1] * g_vel_scales[1])
        print "new vel sent > " + str(g_target_twist.linear) + " " + str(g_target_twist.angular)
        # send_twist()


def fetch_param(name, default):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
        return default


def scan_callback(msg):
    global g_range_ahead
    global g_range_bearing
    global g_range_closest

    g_range_ahead = msg.ranges[len(msg.ranges)/2]
    #g_range_bearing = msg.angle_min + i * msg.angle_max / len(msg.ranges)

    #print "range " + str(g_range_ahead)


def brake():  # Brakes to a complete stop gradually
    global g_target_twist, g_last_twist, g_vel_scales
    stop = "St"
    while (g_target_twist.angular.z !=0) | (g_target_twist.linear.x != 0):

        zSign = 1 if (g_target_twist.angular.z > 0) else -1
        xSign = 1 if (g_target_twist.linear.x > 0) else -1

        if g_target_twist.angular.z != 0:
            g_target_twist.angular.z -= (zSign * g_vel_scales[0])

        if g_target_twist.linear.x != 0:
            g_target_twist.linear.x -= (xSign * g_vel_scales[1])

        send_twist()
        rate.sleep()

        # if absolute velocity is less than 0.1ms set it to zero
        if abs(g_target_twist.angular.z) < 0.1 :
            g_target_twist.angular.z = 0

        if abs(g_target_twist.linear.x) < 0.1 :
            g_target_twist.linear.x = 0

        stop += "o"
        print stop + "p!!!"

    send_twist() # sends the full stop twist


def brakelinear(fastbrake):  # Brakes to a complete stop gradually
    global g_target_twist, g_last_twist, g_vel_scales
    stop = "Linear St"

    brakefactor = g_vel_scales[1]
    if fastbrake is True:
        brakefactor = 1

    while g_target_twist.linear.x != 0:
        xSign = 1 if (g_target_twist.linear.x > 0) else -1

        if g_target_twist.linear.x != 0:
            g_target_twist.linear.x -= (xSign * brakefactor)

        send_twist()
        rate.sleep()

        # if absolute velocity is less than 0.1ms set it to zero
        if abs(g_target_twist.linear.x) < 0.1:
            g_target_twist.linear.x = 0

        stop += "o"
        print stop + "p!!!"

    send_twist() # sends the full stop twist


def EvaluateCollisionPotential():  # brakes if possible collision
    global g_scan_distance
    speed = g_last_twist.linear.x
    if speed > 0 and g_range_ahead <= g_scan_distance:
        brakelinear((speed > 0.5))  # if more than 0.5ms does a hard brake


def obstacle_detected():
    return g_range_ahead <= g_scan_distance


def pose_callback(data):
    global g_home_pose
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                              data.pose.pose.orientation.y,
                                              data.pose.pose.orientation.z,
                                              data.pose.pose.orientation.w])

    if g_home_pose is None:  # only fills home pose once
        g_home_pose = get_home_pose(x, y, yaw)

    print 'Pose ' + str(x)


def get_home_pose(x, y, yaw):
    pose = MoveBaseGoal()
    pose.target_pose.header.frame_id = "/map"
    pose.target_pose.pose.position.x = x
    pose.target_pose.pose.position.y = y
    pose.target_pose.pose.position.z = 0.0

    angle = radians(yaw)
    quaternion = quaternion_from_euler(0.0, 0.0, angle)
    pose.target_pose.pose.orientation = Quaternion(*quaternion.tolist())

    return pose


def go_home():
    client.send_goal(g_home_pose)
    client.wait_for_result()


if __name__ == '__main__':
    rospy.init_node('rampytwister')
    rate = rospy.Rate(20)

    g_last_twist_send_time = rospy.Time.now()
    g_twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('keys', String, keys_cb)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    scan_subscription = rospy.Subscriber('scan', LaserScan, scan_callback)


    # gets home base coordinates
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    while g_home_pose is None:
        rate.sleep()


    g_target_twist = Twist()  # initializes to zero
    g_last_twist = Twist()
    g_vel_scales[0] = fetch_param('~angular_scale', 0.1)
    g_vel_scales[1] = fetch_param('~linear_scale', 0.1)
    g_vel_ramps[0] = fetch_param('~angular_accel', 1.0)
    g_vel_ramps[1] = fetch_param('~linear_accel', 1.0)

    g_scan_distance = fetch_param('~scan_distance', 0.8)
    g_initial_linear_speed = fetch_param('~initial_speed', 0.2)
    g_target_twist.linear.x = g_initial_linear_speed

    g_range_ahead = 1

    while not rospy.is_shutdown():
        print "Scan Distance" + str(g_scan_distance)
        EvaluateCollisionPotential()  # brakes if possible collision

        #TODO put some logic here to allow pausing and resuming of send_twist whenrobot is navigating autonomously
        send_twist()

        rate.sleep()