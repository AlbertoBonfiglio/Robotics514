#!/usr/bin/env python
import rospy


class GenericNode(object):
    __rate = None

    def __init__(self, node='myNode', rate=10):
        rospy.init_node(node)
        self.setRate(rate)


    def setRate(self, x):
        self.__rate = rospy.Rate(x)


    def isShutDown(self):
        return rospy.is_shutdown()

    def sleep(self):
        self.__rate.sleep()

    def spin(self):
        rospy.spin()

