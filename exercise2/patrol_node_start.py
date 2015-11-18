#!/usr/bin/env python
import rospy

from nodes.patrol_node import PatrolController

if __name__ == '__main__':

    _patrolControl = PatrolController()

    while not _patrolControl.isShutDown():
        rospy.logwarn(_patrolControl.isPatrolling())

        if not _patrolControl.isPatrolling():
            _patrolControl.startPatrol()

        _patrolControl.sleep()

