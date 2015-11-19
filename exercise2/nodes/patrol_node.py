#!/usr/bin/env python
import rospy
import csv
import pkg_resources
import actionlib
import random

from std_msgs.msg import String
from generic_node import GenericNode
from copy import copy
from marvin import Marvin

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker, MarkerArray


class PatrolController(GenericNode):
    __waypoints = []
    __poser = None
    __homeGoal = None
    __actionClient = None

    __keyListener = None

    __isPatrolling = False
    __stopFlag = False
    __shuffleFlag = False

    __awaitingGoalCompletion = False

    __marvin = None
    __markerPublisher = None
    __markerArray = None

    def __init__(self):
        super(PatrolController, self).__init__('PatrolServer')

        self.__keyListener = rospy.Subscriber('keys', String, self.__keysCallback)

        self.__actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.__actionClient.wait_for_server()

        self.__waypoints = self.__loadWaypointsFromCsv()
        self.__homeGoal = self.__getGoal(self.__waypoints[0])

        self.__marvin = Marvin()

        self.__markerPublisher = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)
        self.__markerArray = MarkerArray()


    def startPatrol(self):
        try:
            rospy.loginfo('Starting new patrol')
            waypoints = self.__preparePatrol(self.__shuffleFlag)

            self.__setMarkers(waypoints)

            self.__stopFlag = False
            self.__shuffleFlag = False
            self.__isPatrolling = True
            for n in range(len(waypoints)):
                if not self.__stopFlag:
                    rospy.logwarn('PatrolNode.patrol - Patrolling to node {0}'.format(str(n)))

                    self.__actionClient.send_goal(self.__getGoal(waypoints[n]),
                                                  active_cb=self.__OnGoalStarted(),
                                                  done_cb=self.__OnGoalDone())
                    self.__awaitingGoalCompletion = True
                    self.__actionClient.wait_for_result()

                    rospy.logwarn('PatrolNode.patrol - Arrived at node {0}'.format(str(n)))

            self.__isPatrolling = False

        except Exception as ex:
            self.__isPatrolling = False
            rospy.logwarn('PatrolNode.patrol - ', ex.message)


    def stopPatrol(self):
        try:
            rospy.logwarn('Requested Stop Patrol')
            self.__actionClient.cancel_all_goals()
            self.returnHome()

        except Exception as ex:
            rospy.logwarn('PatrolNode.endPatrolAndRestart - ', ex.message)


    def returnHome(self):
        try:
            rospy.loginfo('Heading home')
            _waypoint = self.__homeGoal
            self.__actionClient.send_goal(_waypoint)
            self.__actionClient.wait_for_result()
            rospy.loginfo('Arrived safe and sound')
            return 'succeeded'

        except Exception as ex:
            rospy.logwarn('PatrolNode.__goHome - ', ex.message)
            return 'failed'


    def isPatrolling(self):
        return self.__isPatrolling


    def __loadWaypointsFromCsv(self):
        _retVal = []

        try:
            _filename = pkg_resources.resource_filename('resources', 'test.csv')

            with open(_filename, 'rb') as f:
                _reader = csv.reader(f)
                for _row in _reader:
                    _buffer = []
                    for _col in range(len(_row)):
                        _buffer.append(float(_row[_col]))

                    _retVal.append(_buffer)

        except Exception as ex:
            import sys
            print "System path= ", sys.path
            print"Error! --> ", ex.message

        return _retVal


    def __preparePatrol(self, randomize = False):
        try:

            if randomize == True:
                waypoints = copy(self.__waypoints)
                random.shuffle(waypoints)
                rospy.loginfo("Waypoints are now shuffled")
                rospy.loginfo(waypoints[0])
                return waypoints
            else:
                return self.__waypoints

        except Exception as ex:
            rospy.logwarn('PatrolNode.PreparePatrol - ', ex.message)


    def __keysCallback(self, msg):
        try:
            rospy.logwarn('Received keypress "{0}"'.format(msg.data))
            if len(msg.data) == 0:
                rospy.logwarn('Received unmapped keypress')
                return  # unknown key.

            if msg.data == 'h':
                rospy.logwarn('Go home and restart patrol')
                self.__stopFlag = True
                self.stopPatrol()
                self.__isPatrolling = False

            elif msg.data == 'r':
                rospy.logwarn('Start Randomized Patrol')
                self.__stopFlag = True
                self.__shuffleFlag = True
                self.stopPatrol()
                self.__isPatrolling = False

        except Exception as ex:
            rospy.logwarn('PatrolNode.__keysCallback - ', ex.message)


    def __getGoal(self, data):
        _waypoint = MoveBaseGoal()
        _waypoint.target_pose.header.frame_id = 'map'
        _waypoint.target_pose.pose.position.x = data[0]
        _waypoint.target_pose.pose.position.y = data[1]
        _waypoint.target_pose.pose.position.z = data[2]
        _waypoint.target_pose.pose.orientation.x = data[3]
        _waypoint.target_pose.pose.orientation.y = data[4]
        _waypoint.target_pose.pose.orientation.z = data[5]
        _waypoint.target_pose.pose.orientation.w = data[6]

        return _waypoint


    def __OnGoalDone(self):
        self.__marvin.sayRandomSentence(False)
        self.__awaitingGoalCompletion = False


    def __OnGoalStarted(self):
        self.__awaitingGoalCompletion = True
        rospy.loginfo('Goal is Started')


    def __OnGoalFeedback(self):
        rospy.loginfo('Goal is giving feedback')

    def __setMarkers(self, waypoints):
        try:
            #self.__markerArray.markers
            rospy.logwarn('Preparing to deploy Markers')
            for n in range(len(waypoints)):
                waypoint = self.__getGoal(waypoints[n])

                marker = self.__setMarker(n, waypoint, [1.0, random.uniform(0.0, 1.0), random.uniform(0.0, 1.0), random.uniform(0.0, 1.0)])
                markers = len(self.__markerArray.markers)
                if markers > len(waypoints):
                        self.__markerArray.markers.pop(0)
                self.__markerArray.markers.append(marker)



            self.__markerPublisher.publish(self.__markerArray)

            rospy.logwarn('Markers are set')

        except Exception as ex:
            rospy.logwarn('PatrolNode.__setMarkers - ', ex.message)


    def __setMarker(self, id, waypoint, colors = [1,0,0,0]):
        try:
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'patrol'
            marker.id = id
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = colors[0]
            marker.color.r = colors[1]
            marker.color.b = colors[2]
            marker.color.g = colors[3]

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint.target_pose.pose.position.x
            marker.pose.position.y = waypoint.target_pose.pose.position.y
            marker.pose.position.z = waypoint.target_pose.pose.position.z

            return marker

        except Exception as ex:
            rospy.logwarn('PatrolNode.__setMarker- ', ex.message)

    def __setTextMarker(self, id, waypoint, colors = [1,0,0,0]):
        try:
            marker = Marker()
            marker.header.frame_id = '/map'
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'patrol'
            marker.id = id + len(self.__waypoints)
            marker.type = marker.TEXT_VIEW_FACING
            marker.action = marker.ADD
            marker.text = str(id)
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = colors[0]
            marker.color.r = colors[1]
            marker.color.b = colors[2]
            marker.color.g = colors[3]

            marker.pose.orientation.w = 1.0
            marker.pose.position.x = waypoint.target_pose.pose.position.x
            marker.pose.position.y = waypoint.target_pose.pose.position.y
            marker.pose.position.z = waypoint.target_pose.pose.position.z

            return marker

        except Exception as ex:
            rospy.logwarn('PatrolNode.__setMarker- ', ex.message)
