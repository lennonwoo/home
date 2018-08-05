# -*- coding: utf-8 -*-
import rospy

from body import Leg, Arm
from cerebrum import Memory
from perception import Perception
from communication import Mouth, Ear
from utils import get_nav_pose, get_poses, display_with_box, save_img_with_name


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
        self._mouth.speak(msg)
        return True

    def speak_with_wav(self, wav_path):
        self._mouth.speak_with_wav(wav_path)

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

    def body_down(self):
        self.speak_with_wav(self.config.body_down_wav)

    def remember_job(self):
        face = self._perception.get_face()
        self.speak_with_wav(self.config.hello_name_job_wav)

        rospy.loginfo("[remember_job] start get job")
        job = self._ear.get_asr(self.config.asr_job_class)

        job.set_face(face)
        self._memory.add_job(job)
        rospy.loginfo("[remember_job] get job!")

    def confirm_job(self):
        job = self._memory.get_jobs()[-1]
        msg = "你的名字是%s,我要找%s给你,是吗" % (job.people_name, job.obj_name)
        self.speak(msg)
        # use asr_confirm_class to return AsrConfirm Class
        confirmed = self._ear.get_asr(self.config.asr_confirm_class).confirmed
        if confirmed:
            return True
        else:
            self._memory.delete_last_job()
            self.remember_job()
            # want cofirm until true?
            # self.confirm_job()

    def find_obj_poses(self, obj_name):
        """
        obj_name: str the object name in yolo config that we want to find
        return: pose in the map
        """
        boxes, raw_image = self._perception.get_obj(obj_name)

        if boxes is None:
            return None

        rospy.loginfo("[find obj] %s", obj_name)
        poses = get_poses(boxes, self.config.distance[obj_name])

        return poses

    def prepare_find_people(self):
        imgs = []
        for job in self._memory.get_jobs():
            imgs.append((job.people_img, job.people_name))

        self._perception.init_face_db(imgs)

    def recognize(self):
        face = self._perception.get_face()
        name = self._perception.identify(face)
        self.speak(name)

    def debug(self):
        if not self.config.debug:
            return

        for job in self._memory.get_jobs():
            name = "".join([self.config.save_base_path, job.people_name, '.jpg'])
            save_img_with_name(name, job.people_img)
            job.debug()
