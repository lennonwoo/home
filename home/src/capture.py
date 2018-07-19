#!/usr/bin/env python
import os

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


CV_WINDOW_NAME = "Saver"
NODE_NAME = "Home capture"


class Capture:
    def __init__(self, sub_image_topic, save_path):
        self.path = save_path
        self.count = 0
        self.bridge = CvBridge()

        rospy.Subscriber(sub_image_topic, Image, self.callback_image, queue_size=1)

    def callback_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow(CV_WINDOW_NAME, cv_image)
        key = cv2.waitKey(30)
        if key == 27:
            cv2.imwrite(''.join([self.path, str(self.count), ".jpg"]), cv_image)
            self.count += 1


def main():
    rospy.init_node(NODE_NAME, anonymous=False)
    subscribe_image_topic = rospy.get_param("~subscribe_image_topic")
    save_path = rospy.get_param("~save_path")

    Capture(subscribe_image_topic, save_path)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image save module"

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
