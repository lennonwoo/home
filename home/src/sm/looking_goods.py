import rospy
import smach
import cv_bridge


class LookingGoods(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata):
        return 'finished'
