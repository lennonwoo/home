# -*- coding: utf-8 -*-
import dynamic_reconfigure.client

from asr import AsrJobComplicative, AsrJobNameObj, AsrConfirm


class Config:
    distance = {
        'people': 1.1,
        'face': 0.5,

        'wanglaoji': 0.6,
        'fenda': 0.6,
        'kele': 0.6,
        'xuebi': 0.6,
        'shui': 0.6,
        'hongcha': 0.6,
        'shupian': 0.6,
    }

    box_threshold = {
        'people': 0.6,
        'face': 0.9,

        'wanglaoji': 0.6,
        'fenda': 0.6,
        'kele': 0.6,
        'xuebi': 0.6,
        'shui': 0.6,
        'hongcha': 0.6,
        'shupian': 0.6,
    }

    names_cn = [
        "拉文",
        "丹尼尔",
        "迈克尔",
        "杰克",
        "费希尔",
        "凯文",
        "露丝",
        "约翰",
        "玛丽",
        "亚当",
    ]

    objs_cn = [
        "芬达",
        "薯片",
        "可乐",
        "冰红茶",
        "矿泉水",
        "雪碧",
        "王老吉",
    ]

    objs_en = [
        "fenda",
        "shupian",
        "kele",
        "hongcha",
        "shui",
        "xuebi",
        "wanglaoji",
    ]

    @staticmethod
    def get_obj_pose():
        dic = {}
        for obj in Config.objs_en:
            dic[obj] = None
        return dic

    @staticmethod
    def get_obj_en2cn():
        dic = {}
        for obj_en, obj_cn in zip(Config.objs_en, Config.objs_cn):
            dic[obj_en] = obj_cn
        return dic

    @staticmethod
    def get_obj_cn2en():
        dic = {}
        for obj_en, obj_cn in zip(Config.objs_en, Config.objs_cn):
            dic[obj_cn] = obj_en
        return dic

    poses_file_path = "/home/lennon/home_ws/.cache/poses_20180801.xml"

    # meet guest part
    people_num = 1
    find_people_retry = False

    speak_pub_topic = "/xf/tts/words"
    asr_action_topic = "/xf_asr/home_recognize"
    tts_action_topic = "/xf_tts/tts_generate"
    yolo_action_topic = "/darknet_ros/check_for_objects"

    asr_job_class = AsrJobNameObj
    asr_confirm_class = AsrConfirm
    asr_continue_time = 6

    # wav part
    play_command = "play"
    wav_speed_up = "1.2"  # Notes: use str

    base_path = "/home/lennon/Desktop/"
    wav_base_path = base_path + "wav/"

    self_intro_wav     = wav_base_path + "self_intro.wav"
    body_down_wav      = wav_base_path + "body_down.wav"
    short_body_down_wav= wav_base_path + "short_body_down.wav"
    hello_name_job_wav = wav_base_path + "hello_name_job.wav"
    next_guest_wav     = wav_base_path + "next_guest.wav"
    again_wav          = wav_base_path + "again.wav"
    then_again_wav     = wav_base_path + "then_again.wav"
    stranger_wav       = wav_base_path + "stranger.wav"

    broadcast_job_wav_base_path = wav_base_path + "broadcast_job/"
    broadcast_job_wav_msg_format = "你是%s，我要拿%s"
    broadcast_job_wav_path_format = broadcast_job_wav_base_path + broadcast_job_wav_msg_format +  ".wav"

    confirm_job_wav_base_path = wav_base_path + "confirm_job/"
    confirm_job_wav_msg_format = "你是%s，我要拿%s，是吗"
    confirm_job_wav_path_format = confirm_job_wav_base_path + confirm_job_wav_msg_format + ".wav"

    hello_job_wav_base_path = wav_base_path + "hello_job/"
    hello_job_wav_msg_format = "你好%s，你的%s"
    hello_job_wav_path_format = hello_job_wav_base_path + hello_job_wav_msg_format + ".wav"

    obj_wav_base_path = wav_base_path + "obj/"
    obj_wav_msg_format = "%s"
    obj_wav_path_format = obj_wav_base_path + obj_wav_msg_format + ".wav"

    # dynamic costmap param change part
    @staticmethod
    def change_costmap_params(robot_radius=0.18, inflation_radius=0.3):
        client = dynamic_reconfigure.client.Client("move_base/global_costmap", timeout=10)
        client.update_configuration({"robot_radius": robot_radius})
        client = dynamic_reconfigure.client.Client("move_base/global_costmap/inflation_layer", timeout=10)
        client.update_configuration({"inflation_radius": inflation_radius})
        client = dynamic_reconfigure.client.Client("move_base/local_costmap", timeout=10)
        client.update_configuration({"robot_radius": robot_radius})
        client = dynamic_reconfigure.client.Client("move_base/local_costmap/inflation_layer", timeout=10)
        client.update_configuration({"inflation_radius": inflation_radius})

    @staticmethod
    def decrease_costmap():
        Config.change_costmap_params(0.1, 0.1)
        pass

    @staticmethod
    def increase_costmap():
        Config.change_costmap_params(0.18, 0.3)
        pass

    # arm part
    arm_port_name = '/dev/ttyUSB0'
    arm_baud = 57600

    # debug part
    debug = True
    debug_path = base_path + "debug/"
    final_debug_path = base_path + "final_debug/"

