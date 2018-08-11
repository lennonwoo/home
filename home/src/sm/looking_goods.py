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
            # when take food, do not sort it
            # when 2 objs close to much, robot will hold position when move to the second obj
            # poses = get_sorted_poses(poses)

            for pose in poses:
                # need the robot arrived first
                if self.robot.move(pose):
                    if self.robot.config.enable_arm:
                        # in case the arm out of control
                        try:
                            self.robot._arm.grasp(pose_name_dict[pose])
                        except Exception as e:
                            print e
                    else:
                        wav_path = self.robot.config.obj_wav_path_format % pose_name_dict[pose]
                        self.robot.speak_with_wav(wav_path)

        self.robot.config.increase_costmap()

        return 'finished'
