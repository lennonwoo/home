# -*- coding: utf-8 -*-
import os

import rospy
import actionlib

from std_msgs.msg import String
from xf_ros.msg import *

from config import Config


class GenerateWav:
    def __init__(self, generator=None):
        self.tts_action = actionlib.SimpleActionClient("/xf_tts/tts_generate", TTSAction)
        self.tts_action.wait_for_server(rospy.Duration(3))
        self.generator = generator

    def generate_wav(self):
        for msg, path in self.generator:
            audio_path = self._generate_wav_via_tts(msg)
            print(msg, audio_path)

            cmd = " ".join(['mv', audio_path, path])
            print(msg, cmd)
            os.system(cmd)

    def change_format(self, generator):
        self.generator = generator

    def _generate_wav_via_tts(self, s):
        goal = TTSGoal()
        msg = String()
        msg.data = s
        goal.tts_str = msg

        self.tts_action.send_goal(goal)
        self.tts_action.wait_for_result(rospy.Duration(20))

        audio_path = self.tts_action.get_result().audio_path.data
        return audio_path


class BasicGenerator:
    def __init__(self, msg_format, path_format):
        self.msg_format = msg_format
        self.path_format = path_format


class NameObjMsgPathGenerator(BasicGenerator):
    def __init__(self, msg_format, path_format):
        BasicGenerator.__init__(self, msg_format, path_format)

    def __iter__(self):
        # based on home's 10 people names and 10 obj names
        for name in Config.names_cn:
            for obj in Config.objs_cn:
                msg = self.msg_format % (name, obj)
                path = self.path_format % (name, obj)
                yield msg, path


class ObjMsgPathGenerator(BasicGenerator):
    def __init__(self, msg_format, path_format):
        BasicGenerator.__init__(self, msg_format, path_format)

    def __iter__(self):
        for obj in Config.objs_cn:
            msg = self.msg_format % obj
            path = self.path_format % obj
            yield msg, path


def main():
    # cd ~/home_ws && source devel/setup.zsh && roslaunch xf_ros tts.launch
    os.system("""tmux new-window -c ~/home_ws "source devel/setup.zsh && roslaunch xf_ros tts.launch" """)
    rospy.init_node('generate_name_obj_wav')

    gw = GenerateWav()

    gw.change_format(NameObjMsgPathGenerator(Config.broadcast_job_wav_msg_format,
                                             Config.broadcast_job_wav_path_format))
    gw.generate_wav()

    # gw.change_format(NameObjMsgPathGenerator(Config.confirm_job_wav_msg_format,
    #                                          Config.confirm_job_wav_path_format))
    # gw.generate_wav()

    gw.change_format(NameObjMsgPathGenerator(Config.hello_job_wav_msg_format,
                                             Config.hello_job_wav_path_format))
    gw.generate_wav()

    gw.change_format(ObjMsgPathGenerator(Config.obj_wav_msg_format,
                                         Config.obj_wav_path_format))
    gw.generate_wav()


if __name__ == '__main__':
    main()
