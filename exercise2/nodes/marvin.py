#!/usr/bin/env python

import rospy
import csv
import pkg_resources

import actionlib
import random
import pyttsx

class Marvin(object):
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


    def __init__(self):
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
            rospy.loginfo(sentence)

        except Exception as ex:
            rospy.logwarn('MArvin.sayRandomSentence - ', ex.message)