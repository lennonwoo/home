# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses
from utils import decrease_costmap, increase_costmap


class LookingGoods(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata):
        try:
            self.robot.nav_by_place_name("obj_place")
            return 'finished'
        except Exception as e:
            return 'finished'

        place_list = self.robot.get_place_list('obj_place')
        for place in place_list:
            self.robot.nav_by_place_name(place)
            self.try_grasp_obj()

        return 'finished'

    def try_grasp_obj(self):
        try:
            decrease_costmap()

            # get three dict
            obj_en2cn = self.robot.config.get_obj_en2cn()
            obj_cn2en = self.robot.config.get_obj_cn2en()
            pose_name_dict = {}

            # first we get all obj's pose
            poses = []
            for job in self.robot.get_jobs():
                obj_en = obj_cn2en[job.obj_name]
                rospy.loginfo("one obj: %s", obj_en)
                pose = self.robot.find_obj_poses(obj_en)

                if pose is None:
                    continue

                if type(pose) == list:
                    pose = pose[0]

                # cannot direct give pose obj_name attr
                # pose.target_obj_name = job.obj_name    == will be error ==
                pose_name_dict[pose] = job.obj_name
                poses.append(pose)

            # then sort poses and move robot
            if len(poses) > 0:
                # when take food, do not sort it
                # because when 2 objs close to much, robot will hold position
                # when move to the second obj
                # poses = get_sorted_poses(poses)

                for pose in poses:
                    # need the robot arrived first
                    if self.robot.move(pose) and self.robot.config.enable_arm:
                        # in case the arm out of control
                        try:
                            self.robot._arm.grasp(pose_name_dict[pose])
                        except Exception as e:
                            print e

            increase_costmap()
        except Exception as e:
            print(e)

        return 'finished'
