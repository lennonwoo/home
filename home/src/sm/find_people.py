# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses
from utils import decrease_costmap, increase_costmap


class FindPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished', 'retry'])

        self.robot = robot
        self.people_num = self.robot.config.people_num
        self.people_founded = 0

    def execute(self, userdata):
        # return self.test()

        self.robot.init_face_db()
        self.robot.nav_by_place_name("middle")

        place_list = self.robot.get_place_list('find_people_location')
        for place in place_list:
            self.robot.nav_by_place_name(str(place))
            poses = self.robot.find_obj_poses("people")

            if poses is None:
                continue

            rospy.loginfo("找到%d个人", len(poses))

            if len(poses) > 0:
                # poses = self.robot.filter_poses(poses)
                poses = get_sorted_poses(poses)

                decrease_costmap()
                for pose in poses:
                    if self.robot.move(pose):
                        self.robot.recognize()
                    self.people_founded += 1
                increase_costmap()

            if self.people_founded >= self.people_num:
                return 'finished'

    def test(self):
        self.robot.init_face_db()

        for _ in range(5):
            self.robot.recognize()

        return 'finished'
