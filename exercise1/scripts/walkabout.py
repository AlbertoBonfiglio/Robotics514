#!/usr/bin/env python

import rospy
import math
import actionlib
import random
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

# Main function initializes class Walkabout
#   Walkabout initializes Walkabout node
#       subscribes to receive cmd_vel, amcl_pose, and scan topic messages
#       initializes a publisher to redirect twist messages received after manipulation
#       initialize an action client to issue move_base commands

#       when pose is received
#           if home_pose is empty fill it
#           store latest received pose for later use if necessary (recovery mechanism)

#       when twist is received
#           evaluate if forward and in crashing range
#           if not, propagate twist and set obstacle count to 0
#           else stop and increase obstacle found to deal with stuck robot
#               if obstaclecount >3 then
#                   initiate recovery
#                   rotate randomly to different heading

#       when scan is received
#           calculate distance in a 60 degrees arc
#           evaluate if forward and in crashing range
#           if it is alter last twist to stop



class Walkabout:
    # region Private Variables
    __twister = None  # used to send twist commands
    __lastTwist = None
    __client = None  # used to retrieve pose and navigate back home

    __poser = None
    __home_pose = None  # stores the initial pose
    __currentPose = None

    __scanner = None
    __scanArc = 30  # stores the arc to scan
    __maxDistance = 0.5  # defaults to stop if 50cm from obstacle
    __rangeAhead = 0
    __redirectYaw = 0
    __defaultLinearSpeed = 0.0
    __defaultAngularSpeed = 0.0

    __rate = None  # rate pointer

    __isRecovering = False
    __obstaclesCount = 0
    __savedTwist = None
    # endregion


    # region Constructor
    def __init__(self):
        rospy.init_node('walkabout')
        self.__twister = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        #self.__twister = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)

        self.__twistReceiver = rospy.Subscriber('/cmd_vel_walkabout', Twist, self.__twistCallback)
        self.__poser = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.__poseCallback)
        self.__scanner = rospy.Subscriber('scan', LaserScan, self.__scanCallback)

        # gets home base coordinates
        __client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        __client.wait_for_server()

    # endregion


    # region Private Methods
    def __poseCallback(self, data):
        if self.__home_pose is None:  # only fills home pose once
            self.__home_pose = data

        self.__currentPose = data

    def __twistCallback(self, data):
        self.__lastTwist = data
        self.__twister.publish(data)

    def __scanCallback(self, msg):
        self.__rangeAhead = min(float(r) for r in msg.ranges)
        #print  " min range: ",  min(float(r) for r in msg.ranges)
        #print  " mean range: ",  msg.ranges[len(msg.ranges) / 2]
        # g_range_bearing = msg.angle_min + i * msg.angle_max / len(msg.ranges)
        # print "range " + str(g_range_ahead)

    def __getGoal(self, data):

        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([data.pose.pose.orientation.x,
                                                  data.pose.pose.orientation.y,
                                                  data.pose.pose.orientation.z,
                                                  data.pose.pose.orientation.w])

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        angle = radians(yaw)
        quaternion = quaternion_from_euler(0.0, 0.0, angle)
        goal.target_pose.pose.orientation = Quaternion(*quaternion.tolist())

        return goal

    def __getRandomYaw(self):
        _sign = 1
        #_rand = random.randint(0, 1)
        #if _rand == 0:
        #    _sign = -1
        _vel = 0.2 * _sign
        return _vel

    #def __getCurrentPose(self):
    #    while self.__home_pose is None:
    ##        self.__rate.sleep()
    #    return self.__home_pose

    # endregion


    #region Properties
    def setRate(self, x):
        self.__rate = rospy.Rate(x)


    def setMaxDistance(self, x):
        self.__maxDistance = x


    def setDefaultSpeed(self, x, z):
        self.__defaultLinearSpeed = x
        self.__defaultAngularSpeed = z


    def setRedirectionYaw(self, yaw):
        self.__redirectYaw = yaw


    def isInCollisionRange(self):
        _retval = self.__rangeAhead <= self.__maxDistance

        if (_retval == False) & (self.__isRecovering == True):   #resets the recovery mechanism
            self.__isRecovering == False
            self.__lastTwist = self.__savedTwist

        return _retval

    #endregion


    #region Public Methods

    def stopAndTurn(self):
        if self.__isRecovering != True :
            self.__lastTwist.linear.x = 0
            self.__lastTwist.angular.z = 0
            self.__isRecovering = False

        _twist = Twist()
        _twist.linear.x = 0
        _twist.angular.z = self.__getRandomYaw()
        self.__lastTwist = _twist
        self.__twister.publish(_twist)


    def wasRecovering(self):
        return self.__recovering


    def waitForHomePose(self):
        while self.__home_pose is None:
            self.__rate.sleep()


    def start(self):
        _twist = Twist()
        _twist.linear.x = self.__defaultLinearSpeed
        _twist.angular.z = self.__defaultAngularSpeed
        self.__lastTwist = _twist
        self.__twister.publish(_twist)


    def resendLastTwist(self):
        if self.isInCollisionRange():
            self.stopAndTurn()
        else:
            self.__twister.publish(self.__lastTwist)


    def goHome(self):
        self.__client.send_goal(self.__getGoal(self.__home_pose))
        self.__client.wait_for_result()


    def isShutdown(self):
        return rospy.is_shutdown()


    def sleepWalk(self):
        self.__rate.sleep()


    def fetch_param(self, name, default):
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            print "parameter [%s] not defined. Defaulting to %.3f" % (name, default)
            return default

    #endregion






# Main Routine
if __name__ == '__main__':

    node = Walkabout()
    node.setRate(20)
    node.setMaxDistance(node.fetch_param('~max_distance', 0.5))
    node.setRedirectionYaw(node.fetch_param('~yaw', 20))
    node.setDefaultSpeed(node.fetch_param('~linear_speed', 0.2), node.fetch_param('~angular_speed', 0.2))
    node.waitForHomePose()  # makes sure a home pose is stored

    if node.fetch_param('~autostart', 1) == 1:
        node.start()

    while not node.isShutdown():
        node.resendLastTwist()

        # TODO put some logic here to allow pausing and resuming of send_twist whenrobot is navigating autonomously
        # send_twist()
        node.sleepWalk()
