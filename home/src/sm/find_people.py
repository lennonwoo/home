# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses


class FindPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished', 'retry'])

        self.robot = robot
        self.people_num = self.robot.config.people_num
        self.retried = False if self.robot.config.find_people_retry else True
        self.people_founded = 0

    def execute(self, userdata):
        self.robot.prepare_find_people()

        place_list = sorted([k for k in self.robot._nav_pose_dict.keys() if k.startswith('find_people_location')])
        for place in place_list:
            self.robot.nav_by_place_name(str(place))
            poses = self.robot.find_obj_poses("people")

            if poses is None:
                continue

            rospy.loginfo("找到%d个人", len(poses))

            if len(poses) > 0:
                poses = get_sorted_poses(poses)

                self.robot.config.decrease_costmap()
                for pose in poses:
                    if self.robot.move(pose):
                        self.robot.recognize()
                    self.people_founded += 1
                self.robot.config.increase_costmap()

            if self.people_founded >= self.people_num:
                return 'finished'

        if self.retried:
            return 'finished'
        else:
            self.retried = True
            return 'retry'
