import rospy
import smach
import cv_bridge


class FindPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.bridge = cv_bridge.CvBridge()

    def execute(self, userdata):
        pass
