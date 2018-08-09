# -*- coding: utf-8 -*-
import smach
import rospy

from utils import get_sorted_poses


class LookingGoods(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata):
        self.robot.nav_by_place_name("obj_place")
        self.robot.config.decrease_costmap()

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
            poses = get_sorted_poses(poses)
            # move to the obj
            for pose in poses:
                self.robot.move(pose)
                wav_path = self.robot.config.obj_wav_path_format % pose_name_dict[pose]
                self.robot.speak_with_wav(wav_path)
                # self.robot._arm.grasp(pose_name_dict[pose])

        self.robot.config.increase_costmap()

        return 'finished'
