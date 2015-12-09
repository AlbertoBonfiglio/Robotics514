#!/usr/bin/env python

from nodes.patrol_node import PatrolController

if __name__ == '__main__':
    _patrolControl = PatrolController()

    while not _patrolControl.isShutDown():
        if not _patrolControl.isPatrolling():
            pass

        _patrolControl.sleep()