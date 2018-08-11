# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy
import cv_bridge
import actionlib

from darknet_ros_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from facenet_home import Recognition

from ..base import RobotPart


FACE_RECORD_THRESHOLD = 160


class Perception(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        model_path = rospy.get_param("~model_path")
        self.recognition = Recognition(model_path) if self.config.enable_facenet else None

        self.bridge = cv_bridge.CvBridge()
        self.yolo_detect = actionlib.SimpleActionClient(self.config.yolo_action_topic, CheckForObjectsAction)

    def init_face_db(self, imgs):
        # imgs need be [(img1, name1), (img2, name2)]
        self.recognition.init_db(imgs)

    # 一定要拿到脸部的图像
    def get_face(self):
        self.yolo_detect.wait_for_server(rospy.Duration(3))

        face = None
        while face is None:
            goal = CheckForObjectsGoal()

            msg = String()
            msg.data = "face"
            goal.obj_name = msg

            self.yolo_detect.send_goal(goal)

            finishe_in_time = self.yolo_detect.wait_for_result(rospy.Duration(3))
            if not finishe_in_time:
                continue

            result = self.yolo_detect.get_result()
            bounding_boxes_msg = result.bounding_boxes

            for box in bounding_boxes_msg.bounding_boxes:
                if box.Class == "face":
                    face = self.handle_face(box, bounding_boxes_msg)

        return face

    def handle_face(self, box, msg):
        x1 = box.xmin
        x2 = box.xmax
        y1 = box.ymin
        y2 = box.ymax

        height = y2 - y1
        width = x2 - x1

        if min(height, width) > FACE_RECORD_THRESHOLD and box.probability > self.config.box_threshold['face']:
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, "bgr8")
            return cv_image[y1:y2, x1:x2]
        else:
            return None

    def get_obj(self, obj_name="people"):
        self.yolo_detect.wait_for_server(rospy.Duration(3))

        goal = CheckForObjectsGoal()

        msg = String()
        msg.data = obj_name
        goal.obj_name = msg

        self.yolo_detect.send_goal(goal)

        finishe_in_time = self.yolo_detect.wait_for_result(rospy.Duration(3))
        if not finishe_in_time:
            return None, None

        result = self.yolo_detect.get_result()
        bounding_boxes_msg = result.bounding_boxes

        obj_boxes = []
        for box in bounding_boxes_msg.bounding_boxes:
            if box.Class == obj_name and box.probability > self.config.box_threshold[obj_name]:
                x1 = box.xmin
                x2 = box.xmax
                y1 = box.ymin
                y2 = box.ymax
                obj_boxes.append((x1, x2, y1, y2))

        if len(obj_boxes) == 0:
            return None, None

        raw_image = self.bridge.imgmsg_to_cv2(bounding_boxes_msg.image, "bgr8")

        return obj_boxes, raw_image

    def identify(self, img):
        known_face = self.recognition.identify(img)

        if known_face:
            return known_face.name
        else:
            return "陌生人"
