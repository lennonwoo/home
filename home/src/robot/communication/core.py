# -*- coding: utf-8 -*-
import rospy
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import Image
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

    def connect_subscribe(self):
        r = rospy.Rate(100)
        while self.pub.get_num_connections() == 0:
            r.sleep()

    def speak(self, msg):
        # TODO lennon how to check the msg publish successfully?
        # use srv?
        self.connect_subscribe()
        self.pub.publish(msg)

        return True


class Ear(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self._job_parser = self.config.job_parser

        self.xf_asr = actionlib.SimpleActionClient(self.config.asr_action_topic, HomeRecognizeAction)

    def get_job(self):
        self.xf_asr.wait_for_server(rospy.Duration(30))
        continue_time = 12

        job = None
        while job is None:
            goal = HomeRecognizeGoal()

            msg = String()
            msg.data = "home"
            goal.bnf_name = msg
            goal.continue_time = continue_time

            self.xf_asr.send_goal(goal)

            finished_in_time = self.xf_asr.wait_for_result(rospy.Duration(continue_time))
            rospy.loginfo("[get_job] finish in time ", finished_in_time)
            if not finished_in_time:
                rospy.loginfo("[get_job] start again")
                self.robot.speak_with_wav(self.config.again_wav)
                continue

            msg = self.xf_asr.get_result().msg
            job = self._job_parser(msg.data)

        return job
