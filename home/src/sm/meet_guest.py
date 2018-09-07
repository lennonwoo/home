# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses
from utils import decrease_costmap, increase_costmap


class MeetGuest(smach.State):
    def __init__(self, robot, guest_num):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.guest_num = guest_num
        self.people_founded = 0

    def execute(self, userdata):
        # return self.test()

        place_list = self.robot.get_place_list('meet_guest_location')
        for place in place_list:
            self.robot.nav_by_place_name(place)
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
                        self.robot.remember_job()
                        self.robot.broadcast_heard_job()
                    self.people_founded += 1
                increase_costmap()

            if self.people_founded >= self.guest_num:
                return 'finished'

        return 'finished'

    def test(self):
        for _ in range(self.guest_num):
            self.robot.remember_job()
            self.robot.broadcast_heard_job()

        return "finished"
