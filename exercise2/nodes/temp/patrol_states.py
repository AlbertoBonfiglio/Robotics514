#!/usr/bin/env python
import rospy
import csv
import actionlib
import pkg_resources
import random
import pyttsx

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach import State, StateMachine
from time import sleep

class StateWithSpeech(State):
    __soundEngine = None
    __sentences = ['I think you ought to know I''m feeling very depressed. ',
                   'You can blame the Sirius Cybernetics Corporation for making androids with Genuine People Personalities. I''m a personality prototype. You can tell, can''t you...? ',
                   'Incredible... it''s even worse than I thought it would be. ',
                   'Don''t see what the big deal is... Vogons are some of the worst shots in the galaxy...',
                   'Not that anyone cares what I say, but the restaurant is at the other end of the Universe',
                   'I''ve calculated your chance of survival, but I don''t think you''ll like it.',
                   'Here I am, brain the size of a planet, and they ask me to take you to the bridge. Call that job satisfaction, because I don''t.',
                   'Ghastly, isn''t it? All the doors on this spaceship have been programmed to have a cheery and sunny disposition.',
                   'I have a million ideas, but, they all point to certain death',
                   'And then of course I''ve got this terrible pain in all the diodes down my left side. I mean, I''ve asked for them to be replaced, but no-one ever listens.',
                   'The first ten million years were the worst. And the second ten million: they were the worst, too. The third ten million I didn''t enjoy at all. After that, I went into a bit of a decline.',
                   '"Reverse primary thrust, Marvin." That''s what they say to me. "Open airlock number 3, Marvin." "Marvin, can you pick up that piece of paper?" Here I am, brain the size of a planet, and they ask me to pick up a piece of paper.',
                   'Life. Loathe it or ignore it. You can''t like it.',
                   'You think you''ve got problems. What are you supposed to do if you are a manically depressed robot? No, don''t even bother answering. I''m 50,000 times more intelligent than you and even I don''t know the answer.']

    __haltSentences = ['Do you want me to sit in a corner and rust or just fall apart where I''m standing?',
                       'I told you this would all end in tears.']


    def __init__(self, outcomes):
        State.__init__(self, outcomes=outcomes)
        self.__soundEngine = pyttsx.init()



    def sayRandomSentence(self, halting = False):
        try:
            sentence = ''
            if halting:
                sentenceIndex = random.randint(0, len(self.__haltSentences)-1)
                sentence = self.__haltSentences[sentenceIndex]
            else:
                sentenceIndex = random.randint(0, len(self.__sentences)-1)
                sentence = self.__sentences[sentenceIndex]

            self.__soundEngine.say(sentence)
            self.__soundEngine.runAndWait()

        except Exception as ex:
            rospy.logwarn('StateWithSpeech.sayRandomSentence - ', ex.message)




class PatrolLoadWaypointState(State):

    def __init__(self, filename):
        State.__init__(self, outcomes=['succeeded'])


    def execute(self, data):
        print 'patrol'
        sleep(2)
        return 'succeeded'



class PatrolNavigateState(StateWithSpeech):
    __actionClient = None
    __waypoint = None


    def __init__(self, waypoint):
        StateWithSpeech.__init__(self, outcomes=['succeeded', 'aborted',  'preempted'])

        self.__actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.__actionClient.wait_for_server()
        self.__waypoint = self.__getWaypoint(waypoint)


    def execute(self, userdata):
        print 'patrol'
        self.__actionClient.send_goal(self.__waypoint)
        self.__actionClient.wait_for_result()

        if self.preempt_requested():
            self.sayRandomSentence(True)
            self.service_preempt()
            return 'preempted'

        else:
            self.sayRandomSentence(False)
            return 'succeeded'


#    def request_preempt(self):
#        StateWithSpeech.request_preempt(self)
#        rospy.logwarn('class PatrolNavigateState(StateWithSpeech) preempted')


    def __getWaypoint(self, data):
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




class PatrolMarkCheckpointState(StateWithSpeech):

    def __init__(self, waypoint):
        StateWithSpeech.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])



    def execute(self, userdata):
        print 'Checkpoint Marked'
        self.sayRandomSentence(False)

        return 'succeeded'

