# -*- coding: utf-8 -*-
import time

import rospy
import smach


class MeetGuest(smach.State):
    def __init__(self, robot, guest_num, enable_debug=False):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.guest_num = guest_num
        self.enable_debug = enable_debug

    def execute(self, userdata):
        for i in range(self.guest_num):
            self.robot.speak("请稍微蹲下来看着我，并告诉我你名字以及想让我拿什么东西")
            self.robot.remember_job()
            self.robot.confirm_job()
            time.sleep(3)

        if self.enable_debug:
            self.robot.debug_job()

        return 'finished'
