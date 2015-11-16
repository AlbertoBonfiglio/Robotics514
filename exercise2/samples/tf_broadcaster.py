#!/usr/bin/env python
import roslib
import rospy
import tf
import turtlesim.msg

roslib.load_manifest('exercise2')


class TfBroadcaster(object):
    __turtleName = ''

    def __init__(self):
       rospy.init_node('turtle_tf_broadcaster')
       __turtleName = rospy.get_param('~turtle')
       rospy.Subscriber('/%s/pose' % __turtleName, turtlesim.msg.Pose,
                         self.__pose_callback, __turtleName)



    def __pose_callback(self, msg, name):
        _quaternion = tf.transformations.quaternion_from_euler(0, 0, msg.theta)
        _br = tf.TransformBroadcaster()
        _br.sendTransform((msg.x, msg.y, 0),
                          _quaternion,
                          rospy.Time.now(),
                          name,
                          "world")

    def Spin(self):
        rospy.spin()


# Main Routine
if __name__ == '__main__':
    _broadcaster = TfBroadcaster()
    _broadcaster.Spin()


