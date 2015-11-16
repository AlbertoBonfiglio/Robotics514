#!/usr/bin/env python
import rospy
import csv
import pkg_resources

import actionlib
import random

from std_msgs.msg import String
from generic_node import GenericNode
from smach import StateMachine
from patrol_states import PatrolNavigateState, PatrolMarkCheckpointState
from copy import copy

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



class PatrolController(GenericNode):
    __waypoints = []
    __poser = None
    __homeGoal = None
    __actionClient = None
    __StateMachine = None

    __keyListener = None

    __isPatrolling = False
    __restartFlag = False
    __shuffleFlag = False

    def __init__(self):
        super(PatrolController, self).__init__('PatrolServer')

        self.__keyListener = rospy.Subscriber('keys', String, self.__keysCallback)
        #TODO make sure shuffling works and thet the first node stays the same
        # Or try a flag to block the reissue of patrol

        self.__waypoints = self.__loadWaypointsFromCsv()
        self.__homeGoal = self.__getGoal(self.__waypoints[0])

        self.__StateMachine = StateMachine(['succeded', 'aborted', 'preempted'])


    def preparePatrol(self, randomize = False):
        try:
            if self.__StateMachine.is_opened():
                self.__StateMachine.close()
                rospy.loginfo('Statemachine is closed')

            self.__StateMachine.open()
            rospy.loginfo('Statemachine is now open closed')

            waypoints = copy(self.__waypoints)

            if randomize == True:
                waypoints = random.shuffle(waypoints)
                rospy.loginfo("Waypoints are now shuffled")

            for n in range(len(waypoints)):
                waypoint = waypoints[n]
                _waypointId = 'WP' + str(n)
                _waypointActionId = 'WPA' + str(n)
                _nextWaypointId = 'WP' + str((n+1) % len(waypoints))
                _nextWaypointActionId = 'WPA' + str((n+1) % len(waypoints))

                StateMachine.add(_waypointId,
                                        PatrolNavigateState(waypoint),
                                        transitions={'succeeded': _nextWaypointActionId})

                StateMachine.add(_waypointActionId,
                                        PatrolMarkCheckpointState(waypoint),
                                        transitions={'succeeded': _nextWaypointId})

                rospy.loginfo('Waypoint {0} set!'.format(str(n)))

            return True

        except Exception as ex:
            rospy.logwarn('PatrolNode.PreparePatrol - ', ex.message)
            return False


    def patrol(self):
        try:
            self.__isPatrolling = True
            self.__StateMachine.execute()
            self.__isPatrolling = False
            self.__restartFlag = False
            self.__shuffleFlag = False
            rospy.logwarn('PatrolNode.patrol - Patrol routine has started')


        except Exception as ex:
            self.__isPatrolling = False
            rospy.logwarn('PatrolNode.patrol - ', ex.message)


    def endPatrolAndRestart(self, shuffle = False):
        try:
            rospy.logwarn('Requested Patrol Restart')
            self.__StateMachine.request_preempt()

            self.goHome()

            if shuffle: self.preparePatrol(True)

            self.__isPatrolling = False

        except Exception as ex:
            rospy.logwarn('PatrolNode.endPatrolAndRestart - ', ex.message)


    def goHome(self):
        try:
            _waypoint = self.__homeGoal
            _actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            _actionClient.wait_for_server()
            _actionClient.cancel_all_goals()
            _actionClient.send_goal(_waypoint)
            _actionClient.wait_for_result()

        except Exception as ex:
            rospy.logwarn('PatrolNode.__goHome - ', ex.message)


    def isPatrolling(self):
        return self.__isPatrolling


    def restartRequestIssued(self):
        return self.__restartFlag


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


    def __keysCallback(self, msg):
        try:
            rospy.logwarn('Received keypress "{0}"'.format(msg.data))
            if len(msg.data) == 0:
                rospy.logwarn('Received unmapped keypress')
                return  # unknown key.

            if msg.data == 'h':
                self.__restartFlag = True
                rospy.logwarn('Restart Patrol')

            elif msg.data == 'r':
                self.__shuffleFlag == True
                rospy.logwarn('Start Randomized Patrol')

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
