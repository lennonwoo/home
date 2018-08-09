# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses


# class MeetGuest(smach.State):
#     def __init__(self, robot, guest_num):
#         smach.State.__init__(self, outcomes=['finished'])

#         self.robot = robot
#         self.guest_num = guest_num

#     def execute(self, userdata):
#         self.robot.speak_body_down()
#         for i in range(self.guest_num):
#             self.robot.remember_job()
#             self.robot.broadcast_heard_job()
#             # self.robot.confirm_job()

#             if i != self.guest_num - 1:
#                 self.robot.speak_next_guest()

#         return 'finished'


class MeetGuest(smach.State):
    def __init__(self, robot, guest_num):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot
        self.guest_num = guest_num
        self.retried = False if self.robot.config.find_people_retry else True
        self.people_founded = 0

    def execute(self, userdata):
        place_list = sorted([k for k in self.robot._nav_pose_dict.keys() if k.startswith('meet_guest_location')])

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
                    self.robot.move(pose)
                    self.robot.remember_job()
                    self.robot.broadcast_heard_job()
                    # self.robot.confirm_job()
                    self.people_founded += 1
                self.robot.config.increase_costmap()

            if self.people_founded >= self.guest_num:
                return 'finished'

        if self.retried:
            return 'finished'
        else:
            self.retried = True
            return 'retry'
