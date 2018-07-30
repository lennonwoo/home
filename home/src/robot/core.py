# -*- coding: utf-8 -*-
import cv2
import time

from utils import get_nav_pose
from body import Leg, Arm
from communication import Mouth, Ear
from perception import Perception
from cerebrum import Memory, Job


class Robot:
    def __init__(self, poses_file_path=None):
        self._nav_pose_list_index = 0
        self._nav_pose_list, self._nav_pose_dict = get_nav_pose(poses_file_path)

        self._memory = Memory()
        self._ear = Ear(self._memory, heard_parser_func=Job.job_parser)
        self._leg = Leg()
        self._arm = Arm()
        self._perception = Perception(self._memory)
        self._mouth = Mouth()

    def speak(self, msg):
        return self._mouth.speak(msg)

    def nav_by_place_name(self, name):
        assert type(name) == str
        pose = self._nav_pose_dict[name]

        return self._leg.move(pose)

    def remember_job(self):
        face = self._perception.get_face()
        job = self._ear.get_job()

        job.people_img = face
        self._memory.add_job(job)

    def confirm_job(self):
        job = self._memory.get_job()
        msg = "你的名字是%s,我要到%s找%s给你" % (job.people_name, job.obj_location, job.obj_name)
        self.speak(msg)

    def debug_job(self):
        for job in self._memory._job_list:
            print(job)
            cv2.imshow("debug_job", job.people_img)
            while cv2.waitKey(30) != 27:
                time.sleep(0.1)

