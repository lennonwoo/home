# -*- coding: utf-8 -*-
import os

import rospy
import cv2

from robot.core import Robot
from config import Config


def run_follow_people(robot):
    while True:
        print("find people")
        pose = robot.find_obj_poses("people")
        if pose is None:
            continue

        if type(pose) == list:
            pose = pose[0]
        print(pose)
        robot.move(pose)
        print("arrive")
        # robot.speak("arrive")


def run_xf(robot):
    job = robot._ear.get_asr(Config.asr_job_class)
    print(job)


def robot_init_db(robot):
    base_dir = os.path.expanduser("~/Desktop/debug/")

    imgs = []
    for jpg_file in os.listdir(base_dir):
        if jpg_file.endswith("jpg"):
            print(jpg_file)
            img = cv2.imread(base_dir + jpg_file)
            name = jpg_file[:-4]
            imgs.append((img, name))

    robot._perception.init_face_db(imgs)


def run_recognize(robot):
    robot_init_db(robot)
    res = robot.recognize()
    print(res)


def run_arm(robot):
    robot._arm.grasp()
    robot._arm.grasp()
    robot._arm.grasp()


if __name__ == '__main__':
    rospy.init_node('test')
    model_path = os.path.expanduser("~/home_ws/src/home/models/facenet/20180402-114759/20180402-114759.pb")
    rospy.set_param("~model_path", model_path)
    robot = Robot(Config)

    # run_follow_people()
    # run_xf(robot)
    # run_recognize(robot)
    run_arm(robot)
