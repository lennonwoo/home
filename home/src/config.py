# -*- coding: utf-8 -*-
import os

from config_dict import ConfigDict
from asr import AsrJobComplicative, AsrJobNameObj, AsrConfirm


class Config(ConfigDict):
    poses_file_path = os.path.expanduser("~/home_ws/.cache/poses_20180808.xml")

    # meet guest part
    people_num = 1

    speak_pub_topic = "/xf/tts/words"
    asr_action_topic = "/xf_asr/home_recognize"
    tts_action_topic = "/xf_tts/tts_generate"
    yolo_action_topic = "/darknet_ros/check_for_objects"

    asr_job_class = AsrJobNameObj
    mock_job = AsrJobNameObj(None, ConfigDict.names_cn[0], ConfigDict.objs_cn[0], "")
    asr_confirm_class = AsrConfirm
    asr_continue_time = 6

    # wav part
    play_command = "play"
    wav_speed_up = "1.2"  # Notes: use str

    base_path = os.path.expanduser("~/Desktop")
    wav_base_path = base_path + "wav/"

    self_intro_wav          = wav_base_path + "self_intro.wav"
    body_down_wav           = wav_base_path + "body_down.wav"
    short_body_down_wav     = wav_base_path + "short_body_down.wav"
    hint_speak_name_job_wav = wav_base_path + "hint_speak_name_job.wav"
    next_guest_wav          = wav_base_path + "next_guest.wav"
    again_wav               = wav_base_path + "again.wav"
    fault_again_wav         = wav_base_path + "fault_again.wav"
    stranger_wav            = wav_base_path + "stranger.wav"

    broadcast_job_wav_base_path = wav_base_path + "broadcast_job/"
    broadcast_job_wav_msg_format = "你是%s，我要拿%s"
    broadcast_job_wav_path_format = broadcast_job_wav_base_path + broadcast_job_wav_msg_format +  ".wav"

    confirm_job_wav_base_path = wav_base_path + "confirm_job/"
    confirm_job_wav_msg_format = "你是%s，我要拿%s，是吗，请回答正确或错误"
    confirm_job_wav_path_format = confirm_job_wav_base_path + confirm_job_wav_msg_format + ".wav"

    hello_job_wav_base_path = wav_base_path + "hello_job/"
    hello_job_wav_msg_format = "你好%s，你的%s"
    hello_job_wav_path_format = hello_job_wav_base_path + hello_job_wav_msg_format + ".wav"

    obj_wav_base_path = wav_base_path + "obj/"
    obj_wav_msg_format = "%s"
    obj_wav_path_format = obj_wav_base_path + obj_wav_msg_format + ".wav"

    # arm part
    arm_port_name = '/dev/arm'
    arm_baud = 115200

    ARM_CONNECT_COMMAND = [0x40]
    ARM_CLOSE_OBJ_COMMAND = [0xAB, 0x01, 0xEF]
    ARM_GRASP_OBJ_COMMAND = [0xAB, 0x01, 0xEF]
    ARM_RESET_COMMAND = [0xAB, 0x02, 0xEF]
    ARM_TAKE_OBJ_COMMAND = [0xAB, 0x01, 0xEF]

    arm_ok_list = [
        "ok",
    ]
    # enable_arm = True
    enable_arm = False

    # debug part
    debug = True
    debug_path = base_path + "debug/"
    final_debug_path = base_path + "final_debug/"

    # facenet part
    enable_facenet = True
    # enable_facenet = False
    facenet_each_person_face_num = 3

    # map related
    xmin = 0.347
    ymin = -1.43
    xmax = 9.25
    ymax = 10.9

    # TODO(lennon) is move 30 second enough, need test in final home environment
    move_time_limit = 30
