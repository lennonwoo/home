# -*- coding: utf-8 -*-
import smach


class MeetGuest(smach.State):
    def __init__(self, robot, guest_num):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.guest_num = guest_num

    def execute(self, userdata):
        self.robot.self_intro()
        for i in range(self.guest_num):
            self.robot.remember_job()
            self.robot.confirm_job()
            self.robot.next_guest()

        return 'finished'
