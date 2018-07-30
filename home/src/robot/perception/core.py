#!/usr/bin/env python
from operator import mul
from collections import defaultdict

import cv2
import rospy
import cv_bridge
import actionlib

from darknet_ros_msgs.msg import *
from std_msgs.msg import String
from facenet_home import Recognition


BOX_THRESHOLD = 0.8
FACE_RECORD_THRESHOLD = 160


class Perception:
    def __init__(self, memory):
        # model_path = rospy.get_param("~model_path")
        # subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
        # rospy.Subscriber(subscribe_bounding_boxes_topic, BoundingBoxes, self.callback_bounding_boxes, queue_size=1)

        # self.recognition = Recognition(model_path)
        self.bridge = cv_bridge.CvBridge()
        self.memory = memory

        self.yolo_detect = actionlib.SimpleActionClient("/darknet_ros/check_for_objects", CheckForObjectsAction)

    def get_face(self):
        self.yolo_detect.wait_for_server(rospy.Duration(30))

        face = None
        while face is None:
            goal = CheckForObjectsGoal()

            msg = String()
            msg.data = "face"
            goal.obj_name = msg

            self.yolo_detect.send_goal(goal)

            finishe_in_time = self.yolo_detect.wait_for_result(rospy.Duration(300))
            if not finishe_in_time:
                continue

            result = self.yolo_detect.get_result()
            bounding_boxes_msg = result.bounding_boxes

            for box in bounding_boxes_msg.bounding_boxes:
                if box.Class == "face":
                    print("face found")
                    face = self.handle_face(box, bounding_boxes_msg)

        return face

    # def callback_bounding_boxes(self, bounding_boxes_msg):
    #     box_dict = defaultdict(list)
    #     for box in bounding_boxes_msg.bounding_boxes:
    #         if box.Class == "face" and box.probability > BOX_THRESHOLD:
    #             box_dict['face'].append(box)
    #         elif box.Class == "people" and box.probability > BOX_THRESHOLD:
    #             box_dict['people'].append(box)
    #
    #     if 'face' in box_dict:
    #         for box in box_dict['face']:
    #             self.handle_face(box, bounding_boxes_msg)
    #     if 'people' in box_dict:
    #         self.handle_people(box_dict['people'])
    #
    def handle_face(self, box, msg):
        x1 = box.xmin
        x2 = box.xmax
        y1 = box.ymin
        y2 = box.ymax

        cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")

        height = y2 - y1
        width = x2 - x1

        if min(height, width) > FACE_RECORD_THRESHOLD and box.probability > BOX_THRESHOLD:
            return cv_image[y1:y2, x1:x2]
        else:
            return None

    # def handle_people(self, boxes, msg):
    #     people_list = [(box.xmin, box.xmax, box.ymin, box.ymax) for box in boxes]
    #     self.memory.add_people()


    def identify(self, img):
        res = self.recognition.identify(img)

        if res:
            print(res.name)
        else:
            print("stranger")

