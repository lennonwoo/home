# -*- coding: utf-8 -*-
import smach
import rospy


class FindPeople(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished', 'retry'])

        self.robot = robot
        self.people_num = self.robot.config.people_num
        self.retried = False
        self.people_founded = 0

    def execute(self, userdata):
        self.robot.prepare_find_people()

        self.robot.config.change_costmap_params(robot_radius=0.1, inflation_radius=0.1)
        place_list = sorted([k for k in self.robot._nav_pose_dict.keys() if k.startswith('people_place')])
        for place in place_list:
            self.robot.nav_by_place_name(str(place))
            self.robot.find_obj_poses("people")

            rospy.loginfo("找到%d个人", len(self.robot.last_poses))

            for pose in self.robot.last_poses:
                self.robot.move(pose)
                self.robot.recognize()
                self.people_founded += 1

            if self.people_founded >= self.people_num:
                return self.finish()

        if self.retried:
            return self.finish()
        else:
            self.retried = True
            return 'retry'

    def finish(self):
        self.robot.config.change_costmap_params(robot_radius=0.18, inflation_radius=0.30)
        return 'finished'

