# -*- coding: utf-8 -*-
import os

import rospy
import actionlib

from std_msgs.msg import String
from xf_ros.msg import *

from ..base import RobotPart


class Eye(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(robot)

        self.sub_topic = self.config.eye_sub_topic


class Mouth(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self.pub = rospy.Publisher(self.config.speak_pub_topic, String, queue_size=0)
        self.tts_action = actionlib.SimpleActionClient(self.config.tts_action_topic, TTSAction)

    def connect_subscribe(self):
        r = rospy.Rate(100)
        while self.pub.get_num_connections() == 0:
            r.sleep()

    def speak_via_pub(self, msg):
        self.connect_subscribe()
        self.pub.publish(msg)

    def speak(self, s):
        self.tts_action.wait_for_server(rospy.Duration(3))

        goal = TTSGoal()

        msg = String()
        msg.data = s
        goal.tts_str = msg

        self.tts_action.send_goal(goal)

        finished_in_time = self.tts_action.wait_for_result(rospy.Duration(10))
        rospy.loginfo("finished in time: %s", finished_in_time)
        if not finished_in_time:
            # retry for just once
            self.tts_action.wait_for_result(rospy.Duration(10))

        audio_path = self.tts_action.get_result().audio_path.data
        return self.speak_with_wav(audio_path)

    def speak_with_wav(self, wav_path):
        os.system(" ".join([self.config.play_command, wav_path]))
        return True


class Ear(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self.xf_asr = actionlib.SimpleActionClient(self.config.asr_action_topic, HomeRecognizeAction)

    def get_asr(self, asr_class):
        self.xf_asr.wait_for_server(rospy.Duration(3))
        continue_time = self.config.asr_continue_time

        job = None
        while job is None:
            goal = HomeRecognizeGoal()

            msg = String()
            msg.data = "home"
            goal.bnf_name = msg
            goal.continue_time = continue_time

            self.xf_asr.send_goal(goal)

            finished_in_time = self.xf_asr.wait_for_result(rospy.Duration(continue_time))
            if not finished_in_time:
                rospy.loginfo("[get_job] start again")
                self.robot.speak_with_wav(self.config.again_wav)
                continue

            msg = self.xf_asr.get_result().msg
            job = asr_class.parser(msg.data)

        return job
