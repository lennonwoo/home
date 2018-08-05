# -*- coding: utf-8 -*-
import dynamic_reconfigure.client

from job import Job, JobTest


class Config:
    distance = {
        'people': 0.5,
        'face': 0.5,

        'wanglaoji': 0.5,
        'fenda': 0.5,
        'kele': 0.5,
        'xuebi': 0.5,
        'shui': 0.5,
        'hongcha': 0.5,
        'shupian': 0.5,
    }
    box_threshold = {
        'people': 0.7,
        'face': 0.9,

        'wanglaoji': 0.8,
        'fenda': 0.8,
        'kele': 0.8,
        'xuebi': 0.8,
        'shui': 0.8,
        'hongcha': 0.8,
        'shupian': 0.8,
    }

    poses_file_path = "/home/lennon/home_ws/.cache/poses_20180801.xml"

    play_command = "aplay"
    self_intro_wav = "/home/lennon/Desktop/self_intro.wav"
    body_down_wav = "/home/lennon/Desktop/body_down.wav"
    hello_name_job_wav = "/home/lennon/Desktop/hello_name_job.wav"
    next_guest_wav = "/home/lennon/Desktop/next_guest.wav"
    again_wav = "/home/lennon/Desktop/again.wav"

    people_num = 4

    speak_pub_topic = "/xf/tts/words"
    eye_sub_topic = "/camera/color/image_raw"

    asr_action_topic = "/xf_asr/home_recognize"
    tts_action_topic = "/xf_tts/tts_generate"
    yolo_action_topic = "/darknet_ros/check_for_objects"

    debug = True

    # job_class = Job
    job_class = JobTest

    save_base_path = "/home/lennon/Desktop/"

    asr_continue_time = 8

    @staticmethod
    def change_inflation(robot_radius=0.18, inflation_radius=0.3):
        client = dynamic_reconfigure.client.Client("move_base/global_costmap", timeout=10)
        client.update_configuration({"robot_radius": robot_radius})
        client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer", timeout=10)
        client.update_configuration({"inflation_radius": inflation_radius})
        client = dynamic_reconfigure.client.Client("move_base/local_costmap", timeout=10)
        client.update_configuration({"robot_radius": robot_radius})
        client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer", timeout=10)
        client.update_configuration({"inflation_radius": inflation_radius})
