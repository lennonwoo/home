# -*- coding: utf-8 -*-
import smach
import rospy


class LookingGoods(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['finished'])

        self.robot = robot

    def execute(self, userdata):
        self.robot.nav_by_place_name("obj_place")
        self.robot.config.decrease_costmap()

        # three dict
        obj_pose = self.robot.config.get_obj_pose()
        obj_en2cn = self.robot.config.get_obj_en2cn()
        obj_cn2en = self.robot.config.get_obj_cn2en()

        # TODO too dirty try to fix it...
        for job in self.robot.get_jobs():
            obj_en = obj_cn2en[job.obj_name]
            rospy.loginfo("one obj: %s", obj_en)
            obj_pose[obj_en] = self.robot.find_obj_poses(obj_en)

        for job in self.robot.get_jobs():
            obj_en = obj_cn2en[job.obj_name]
            pose = obj_pose[obj_en]

            if pose is None:
                continue
            if type(pose) == list:
                pose = pose[0]

            if self.robot.config.debug:
                print(obj_en, pose)

            self.robot.move(pose)
            # TODO change speak logic to use arm to get object
            wav_path = self.robot.config.obj_wav_path_format % job.obj_name
            self.robot.speak_with_wav(wav_path)

        self.robot.config.increase_costmap()

        return 'finished'
