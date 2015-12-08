#!/usr/bin/env python

import sys, select, tty, termios
import rospy

from generic_node import GenericNode
from std_msgs.msg import String

class KeyLogger(GenericNode):
    __keyPublisher = None
    __oldAttribute = None

    def __init__(self):
        super(KeyLogger, self).__init__('KeyLogger', 100)

        self.__keyPublisher = rospy.Publisher('keys', String, queue_size=1)
        self.__oldAttribute = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print 'Ready to publish keystrokes. press Ctrl-c to exit...'

    def __del__(self):
        rospy.logwarn("Restoring keystrokes.")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.__oldAttribute)
        rospy.logwarn("Terminal keystrokes restored.")



    def run(self):
        #rospy.logwarn('Awaiting keystrokes')
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            keystroke = sys.stdin.read(1)
            rospy.logwarn("Keystroke detected [{0}]".format(keystroke))
            self.__keyPublisher.publish(keystroke)

        self.sleep()
