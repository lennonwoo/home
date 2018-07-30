import rospy
import actionlib

from std_msgs.msg import String
from sensor_msgs.msg import Image
from xf_ros.msg import *


class Eye:
    def __init__(self, memory, sub_topic="/camera/color/image_raw", callback_type=None, callback=None):
        self.memory = memory
        self.sub_topic = sub_topic
        self.callback_type = callback_type
        self.callback = callback
        self.start_sub()

    def stop_sub(self):
        self._sub.unregister()

    def start_sub(self):
        if self.callback and self.callback_type:
            self._sub = rospy.Subscriber(self.sub_topic, self.callback_type, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber(self.sub_topic, Image, self.callback, queue_size=1)

    def callback(self, img):
        self.memory.add_watched(img)


class Mouth:
    def __init__(self, speak_topic="/xf/tts/words"):
        self.pub = rospy.Publisher(speak_topic, String, queue_size=0)

    def connect_subscribe(self):
        r = rospy.Rate(100)
        while self.pub.get_num_connections() == 0:
            r.sleep()

    def speak(self, msg):
        # TODO lennon how to check the msg publish successfully?
        # use srv?
        self.connect_subscribe()
        self.pub.publish(msg)

        return True


class Ear:
    def __init__(self, memory, heard_parser_func=None, enable_debug=False):
        self.memory = memory
        self._heard_parser_func = heard_parser_func
        self.debug = enable_debug

        self.xf_asr = actionlib.SimpleActionClient("/xf_asr/home_recognize", HomeRecognizeAction)

    def get_job(self):
        self.xf_asr.wait_for_server(rospy.Duration(30))
        continue_time = 8

        job = None
        while job is None:
            goal = HomeRecognizeGoal()

            msg = String()
            msg.data = "home"
            goal.bnf_name = msg
            goal.continue_time = continue_time

            self.xf_asr.send_goal(goal)

            finishe_in_time = self.xf_asr.wait_for_result(rospy.Duration(continue_time))
            print(finishe_in_time)
            if not finishe_in_time:
                # print(self.xf_asr.get_result().msg)
                print("[debug] start again")
                continue

            msg = self.xf_asr.get_result().msg
            job = self._heard_parser_func(msg.data)

        return job

    # @property.setter
    # def heard_parser_func(self, func):
    #     self._heard_parser_func = func

    def callback(self, result):
        print(self._heard_parser_func)
        if self._heard_parser_func:
            print("heard parse func")
            self.memory.add_heard(result.data)
            self.memory.add_job(self._heard_parser_func(result.data))
        else:
            print("something else")
            self.memory.add_heard(result.data)

        if self.debug:
            print(result.data)


if __name__ == '__main__':
    from ..cerebrum import Job

    rospy.init_node("test_communication")

    result = """
    
    """
    ear = Ear(heard_parser_func=Job.job_parser)

    rospy.spin()
