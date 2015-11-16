#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

class TfListener(object):
    __rate = None
    __velPublisher = None
    __spawner = None
    __transformListener = None

    def __init__(self):
        rospy.init_node('tf_turtle')
        self.__rate = rospy.Rate(10)

        self.__transformListener = tf.TransformListener()

        rospy.wait_for_service('spawn')

        self.__spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
        self.__spawner(4, 2, 0, 'turtle2')

        self.__velPublisher = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist, queue_size= 1)


    def ListenAndTransform(self):
        trans = []
        try:
            (trans, rot) = self.__transformListener.lookupTransform('/turtle2', '/carrot1', rospy.Time(0))

        except Exception as ex:
            print "Error! {0} ".format(ex.message)
        else:
            angular = 4 * math.atan2(trans[1], trans[0])
            linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            cmd = geometry_msgs.msg.Twist()
            cmd.linear.x = linear
            cmd.angular.z = angular

            self.__velPublisher.publish(cmd)


    def isShutdown(self):
        return rospy.is_shutdown()

    def Sleep(self):
        self.__rate.sleep()


if __name__ == '__main__':
    _listener = TfListener()

    while not _listener.isShutdown():
        _listener.ListenAndTransform()
        _listener.Sleep()