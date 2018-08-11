# -*- coding: utf-8 -*-
from collections import defaultdict

import rospy

from body import Leg, Arm
from cerebrum import Memory
from perception import Perception
from communication import Mouth, Ear
from utils import get_nav_pose, save_img, save_img_with_box
from utils import get_poses, get_poses_by_base_link_xy


class Robot:
    def __init__(self, config):
        self.config = config
        self._nav_pose_list, self._nav_pose_dict = get_nav_pose(self.config.poses_file_path)

        self._memory = Memory(self)
        self._ear = Ear(self)
        self._leg = Leg(self)
        self._arm = Arm(self)
        self._perception = Perception(self)
        self._mouth = Mouth(self)

        self.last_poses = []
        self.find_obj_times = defaultdict(int)

    def speak(self, msg):
        self._mouth.speak_via_action(msg)
        return True

    def speak_with_wav(self, wav_path):
        self._mouth.speak_with_wav(wav_path)

    def nav_by_place_name(self, name):
        assert type(name) == str
        pose = self._nav_pose_dict[name]

        return self._leg.move(pose)

    def move(self, pose, frame_id="map"):
        # return True if the robot arrived in time
        return self._leg.move(pose, frame_id)

    def speak_self_intro(self):
        self.speak_with_wav(self.config.self_intro_wav)

    def speak_next_guest(self):
        self.speak_with_wav(self.config.next_guest_wav)

    def speak_body_down(self):
        self.speak_with_wav(self.config.body_down_wav)

    def speak_short_body_down(self):
        self.speak_with_wav(self.config.short_body_down_wav)

    def remember_job(self):
        self.speak_short_body_down()

        for i in range(self.config.facenet_each_person_face_num):
            face = self._perception.get_face()
            self._memory.add_face(face)
        self.speak_with_wav(self.config.hint_speak_name_job_wav)

        rospy.loginfo("[remember_job] start get job")
        try:
            job = self._ear.get_asr(self.config.asr_job_class)
        except Exception as e:
            print(e)
            self.speak_with_wav(self.config.again_wav)
            job = self._ear.get_asr(self.config.asr_job_class)
        rospy.loginfo("[remember_job] finish get job!")

        self.add_job_with_faces(job, self._memory.get_last_faces_by_config())
        rospy.loginfo("[remember_job] job added!")

    def broadcast_heard_job(self):
        job = self._memory.get_last_job()
        wav_path = self.config.broadcast_job_wav_path_format % (job.people_name, job.obj_name)
        self.speak_with_wav(wav_path)

    def back(self):
        pose = get_poses_by_base_link_xy(-0.3, 0)
        return self._leg.move(pose)

    def confirm_job(self):
        job = self._memory.get_last_job()

        wav_path = self.config.confirm_job_wav_path_format % (job.people_name, job.obj_name)
        self.speak_with_wav(wav_path)

        try:
            confirmed = self._ear.get_asr(self.config.asr_confirm_class).confirmed
        except Exception as e:
            # TODO just return True if asr get wrong xml?
            print(e)
            return True

        if confirmed:
            return True
        else:
            self._memory.delete_last_job()
            self.speak_with_wav(self.config.fault_again_wav)

            rospy.loginfo("[remember_job] start get job")
            job = self._ear.get_asr(self.config.asr_job_class)
            self.add_job_with_faces(job, self._memory.get_last_faces_by_config())
            rospy.loginfo("[remember_job] get job!")

            # want cofirm until true?
            self.confirm_job()

    def add_job_with_faces(self, job, faces):
        job.add_faces(faces)
        if self.config.debug:
            for face in faces:
                dict_key = "".join([job.people_name, job.obj_name])
                path = "".join([self.config.debug_path,
                                job.people_name, job.obj_name,
                                str(self.find_obj_times[dict_key]),
                                '.jpg'])
                save_img(path, face)
                self.find_obj_times[dict_key] += 1

        self._memory.add_job(job)

    def find_obj_poses(self, obj_name):
        """
        find obj poses by odom!
        obj_name: str the object name in yolo config that we want to find
        return: pose in the map or None
        """
        boxes, raw_image = self._perception.get_obj(obj_name)

        if boxes is None:
            return None

        if self.config.debug:
            path = "".join([self.config.debug_path, obj_name,
                            str(self.find_obj_times[obj_name]), '.jpg'])
            save_img_with_box(path, raw_image, obj_name, boxes)
            self.find_obj_times[obj_name] += 1

        rospy.loginfo("[find obj] %s", obj_name)
        poses = get_poses(boxes, self.config.distance[obj_name])

        return poses

    def prepare_find_people(self):
        imgs = []
        for job in self._memory.get_jobs():
            for face in job.get_faces():
                imgs.append((face, job.people_name))

        self._perception.init_face_db(imgs)

    def recognize(self):
        self.speak_short_body_down()
        face = self._perception.get_face()
        name = self._perception.identify(face)

        if self._memory.exist_name(name):
            job = self._memory.get_job_by_name(name)
            wav_path = self.config.hello_job_wav_path_format % (job.people_name,
                                                                job.obj_name)
            self.speak_with_wav(wav_path)
        else:
            self.speak_with_wav(self.config.stranger_wav)

    def get_jobs(self):
        return self._memory.get_jobs()

    def debug(self):
        for job in self._memory.get_jobs():
            for i, face in enumerate(job.get_faces()):
                path = "".join([self.config.final_debug_path,
                                job.people_name, job.obj_name,
                                str(i), '.jpg'])
                save_img(path, face)

        for job in self._memory.get_jobs():
            job.debug()
