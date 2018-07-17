#!/usr/bin/env python
import os

import rospy
import cv_bridge

from darknet_ros_msgs.msg import *
from facenet_home import *


NODE_NAME = "facenet_home"
BOX_THRESHOLD = 0.8


class ImageRecognition:
    def __init__(self, sub_bounding_boxes_topic, database, model_path):
        self.recognition = Recognition(model_path, self.get_imgs_db(database))
        self.bridge = cv_bridge.CvBridge()

        rospy.Subscriber(sub_bounding_boxes_topic, BoundingBoxes, self.callback_bounding_boxes, queue_size=1)

    def callback_bounding_boxes(self, bounding_boxes_msg):
        for box in bounding_boxes_msg.bounding_boxes:
            if box.Class == "face" and box.probability > BOX_THRESHOLD:
                self.handle_face(box, bounding_boxes_msg)
            elif box.Class == "people" and box.probability > BOX_THRESHOLD:
                self.handle_face(box, bounding_boxes_msg)

    def handle_face(self, boxes, msg):
        x1 = boxes.xmin
        x2 = boxes.xmax
        y1 = boxes.ymin
        y2 = boxes.ymax

        img_msg = msg.image
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        res = self.recognition.identify(cv_image[y1:y2, x1:x2])

        if res:
            print(res.name)
        else:
            print("stranger")

    def handle_people(self, box, msg):
        pass

    def get_imgs_db(self, database):
        imgs_db = []

        for f in os.listdir(database):
            if f.endswith("jpg"):
                name = f[:-4]
                img = cv2.imread(database + f)
                img = cv2.resize(img, (160, 160), interpolation=cv2.INTER_CUBIC)
                imgs_db.append((img, name))

        return imgs_db


if __name__ == "__main__":
    rospy.init_node(NODE_NAME, anonymous=False)
    model_path = rospy.get_param("~model_path")
    database = rospy.get_param("~database")
    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    subscribe_bounding_boxes_topic = rospy.get_param("~subscribe_bounding_boxes_topic")
    action_detection_topic = rospy.get_param("~action_detection_topic")

    ir = ImageRecognition(subscribe_bounding_boxes_topic, database, model_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

    cv2.destroyAllWindows()
