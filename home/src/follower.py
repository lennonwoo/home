#!/usr/bin/env python
import sys
from math import atan, log, pi

import cv2
import cv_bridge
import rospy
from std_msgs.msg import Bool
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point, Twist

from asms_ros.msg import *
from robot import Robot
from config import Config


def build_bbox(box, rgb_image):
    bbox = BoundingBox()
    bbox.xmin = box[0]
    bbox.xmax = box[1]
    bbox.ymin = box[2]
    bbox.ymax = box[3]
    bbox.image = rgb_image
    return bbox


def display(box, img):
    x1 = box[0]
    x2 = box[1]
    y1 = box[2]
    y2 = box[3]
    img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0))
    cv2.imshow("DEBUG", img)
    cv2.waitKey(30)


class ObjTracking:
    def __init__(self, target):
        Config.enable_arm = False
        Config.enable_facenet = False
        self.robot = Robot(Config)
        self.bridge = cv_bridge.CvBridge()

        self.subscriber = rospy.Subscriber('/asms/bbox_pub', BoundingBox, self.bbox_callback)
        self.bbox_pub = rospy.Publisher('/asms/bbox_sub', BoundingBox, queue_size=1)
        self.enable_track_pub = rospy.Publisher('/asms/enable_track', Bool)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=1)

        self.bbox_count = 0
        self.target = target

    def init_track(self):
        self.enable_track_pub.publish(False)
        boxes = None
        while boxes is None:
            boxes, image = self.robot._perception.get_obj(self.target, convert2cv2=False)

        bbox = build_bbox(boxes[0], image)
        self.bbox_pub.publish(bbox)
        rospy.loginfo("Starting people tracking")

    def get_obj_location(self, pc2_msg, box):
        """
        :param pc2_msg
        :return: x, y, z in 3D space
        """
        xmin = box[0]
        xmax = box[1]
        ymin = box[2]
        ymax = box[3]
        xstart = (xmin + xmax) / 2
        ystart = (ymin + ymax) / 2

        uvs = []
        for x in range(xstart-5, xstart+5):
            for y in range(ystart-5, ystart+5):
                uvs.append((x, y))

        data_out = pc2.read_points(pc2_msg, field_names=['x', 'y', 'z'], skip_nans=True, uvs=uvs)
        # check if data is valid
        if len(list(data_out)) == 0:
            return 0, 0, 0
        else:
            return [sum(l)/float(len(l)) for l in zip(*data_out)]

    def get_twist_msg(self, x, z):
        """
        For Intel D435, z is depth info => robot speed, x => robot angular
        y is height relationship about obj and camera, not helpful for following here
        :param x: the x axis in Cartesian coordinate system (unit: m)
        :param z: the y axis in Cartesian coordinate system (unit: m)
        :return: cmd_vel msg to follow obj
        """
        msg = Twist()
        if z < Config.follow_distance:
            return msg

        msg.linear.x = max((log(z/Config.follow_distance) + 1), 0.2) * Config.follow_base_speed
        msg.angular.z = -pi * atan(x/z) / (msg.linear.x / (z-Config.follow_distance))
        return msg

    def bbox_callback(self, bbox):
        # pass the too long bbox callback
        if (rospy.Time.now() - bbox.header.stamp).to_sec() > 0.5:
            return

        box = [bbox.xmin, bbox.xmax, bbox.ymin, bbox.ymax]
        print("arrived bbox", bbox.probability, box)
        if bbox.probability > Config.follow_confidence_threshold:
            # used for debug
            # image = self.bridge.imgmsg_to_cv2(bbox.image, "bgr8")
            # display(box, image)

            x, y, z = self.get_obj_location(bbox.pc2, box)
            print("The x: %f y: %f z: %f" % (x, y, z))
            self.cmd_vel_pub.publish(self.get_twist_msg(x, z))
        else:
            # stop tracking and stop robot
            self.cmd_vel_pub.publish(Twist())
            self.init_track()


def main():
    rospy.init_node('obj_tracking')

    obj_tracking = ObjTracking(target="people")
    # obj_tracking = ObjTracking(target="face")
    obj_tracking.init_track()

    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())
