# -*- coding: utf-8 -*-
from job import Job


class Config:
    distance = {
        'people': 0.5,
        'face': 0.5,

        'wanglaoji': 0.4,
        'fenda': 0.4,
        'kele': 0.4,
        'xuebi': 0.4,
        'shui': 0.4,
        'hongcha': 0.4,
        'shupian': 0.4,
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

    people_num = 2

    speak_pub_topic = "/xf/tts/words"
    eye_sub_topic = "/camera/color/image_raw"

    asr_action_topic = "/xf_asr/home_recognize"
    tts_action_topic = "/xf_tts/tts_generate"
    yolo_action_topic = "/darknet_ros/check_for_objects"

    debug = True

    job_class = Job

    save_base_path = "/home/lennon/Desktop/"
