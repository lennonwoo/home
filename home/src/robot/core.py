# -*- coding: utf-8 -*-
import os

import rospy

from body import Leg, Arm
from cerebrum import Memory
from perception import Perception
from communication import Mouth, Ear
from utils import get_nav_pose, get_poses, display_with_box, robot_sleep


class Robot:
    def __init__(self, config):
        self.config = config
        self._nav_pose_list_index = 0
        self._nav_pose_list, self._nav_pose_dict = get_nav_pose(self.config.poses_file_path)

        self._memory = Memory(self)
        self._ear = Ear(self)
        self._leg = Leg(self)
        self._arm = Arm(self)
        self._perception = Perception(self)
        self._mouth = Mouth(self)

        self.last_poses = []

    def speak(self, msg):
        return self._mouth.speak(msg)

    def speak_with_wav(self, wav_path):
        os.system(" ".join([self.config.play_command, wav_path]))
        return True

    def nav_by_place_name(self, name):
        assert type(name) == str
        pose = self._nav_pose_dict[name]

        return self._leg.move(pose)

    def move(self, pose):
        self._leg.move(pose)

    def self_intro(self):
        self.speak_with_wav(self.config.self_intro_wav)

    def next_guest(self):
        self.speak_with_wav(self.config.next_guest_wav)

    def remember_job(self):
        face = self._perception.get_face()
        self.speak_with_wav(self.config.meet_guest_hint_intro_next_wav)

        rospy.loginfo("[remember_job] start get job")
        job = self._ear.get_job()

        job.set_face(face)
        self._memory.add_job(job)
        rospy.loginfo("[remember_job] get job!")

    def confirm_job(self):
        job = self._memory.get_job()
        msg = "你的名字是%s,我要找%s给你" % (job.people_name, job.obj_name)
        self.speak(msg)
        robot_sleep(3)

    def find_obj(self, obj_name):
        boxes, raw_image = self._perception.get_obj(obj_name)

        if boxes is None:
            return False

        rospy.loginfo("[find obj] %s", obj_name)
        self.last_poses = get_poses(boxes, self.config.distance[obj_name])

        if self.config.debug:
            display_with_box(boxes[0], raw_image, obj_name)
        return True

    def prepare_find_people(self):
        imgs = []
        for job in self._memory.get_job():
            imgs.append((job.people_img, job.people_name))

        self._perception.init_face_db(imgs)

    def recognize(self):
        # TODO
        pass

    def debug(self):
        if not self.config.debug:
            return

        for job in self._memory.get_job():
            job.debug()
