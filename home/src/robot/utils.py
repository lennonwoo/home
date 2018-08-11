# -*- coding: utf-8 -*-
from math import acos, asin, cos, sin, isnan

import tf
import cv2
import rospy
import sensor_msgs.point_cloud2 as pc2

from bs4 import BeautifulSoup
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, Point, Quaternion, PointStamped


PI = 3.1415926535897931


def get_nav_pose(file_path):
    lst, dic = list(), dict()
    with open(file_path) as f:
        soup = BeautifulSoup(f, 'lxml')
        for pose in soup.findAll("pose"):
            x, y, z = (float(i) for i in pose.position.text.split(' '))
            point = Point(x, y, z)

            x, y, z, w = (float(i) for i in pose.orientation.text.split(' '))
            quaternion = Quaternion(x, y, z, w)

            lst.append(Pose(point, quaternion))
            dic[pose.nav_name.text] = Pose(point, quaternion)

    return lst, dic


def display(name, img, wait_time=0):
    cv2.imshow(name, img)
    cv2.waitKey(wait_time)


class PointCloudTransformer:
    def __init__(self, pc2_frame_id, distance):
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("base_link", pc2_frame_id, rospy.Time(0), rospy.Duration(4.0))
        self.listener.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(4.0))

        self.ps = PointStamped()
        self.ps.header.stamp = rospy.Time(0)

        self.pc2_frame_id = pc2_frame_id
        self.distance = distance

    def transform(self, point):
        # 离物体要有一定距离
        self.ps.point = point
        self.ps.header.frame_id = self.pc2_frame_id
        p = self.listener.transformPoint("base_link", self.ps).point
        rospy.loginfo("base_link's pose: %s", p)

        p.x -= self.distance
        rospy.loginfo("after distance's pose: %s", p)

        self.ps.point = p
        self.ps.header.frame_id = "base_link"
        p = self.listener.transformPoint("map", self.ps).point
        rospy.loginfo("map's pose: %s", p)

        p.z = 0
        return p


def get_poses_by_base_link_xy(x=0.1, y=0.0):
    odom_msg = rospy.wait_for_message("/odom", Odometry)
    current_quaternition = odom_msg.pose.pose.orientation

    listener = tf.TransformListener()
    listener.waitForTransform("base_link", "map", rospy.Time(0), rospy.Duration(4.0))

    ps = PointStamped()
    ps.header.stamp = rospy.Time(0)
    ps.header.frame_id = "base_link"
    ps.point = Point(x, y, 0)
    p = listener.transformPoint("map", ps).point

    return Pose(p, current_quaternition)


def get_poses(boxes, distance=0.5):
    poses_result = []

    pc2_msg = rospy.wait_for_message("/camera/depth_registered/points", PointCloud2)
    pc2_frame_id = pc2_msg.header.frame_id
    rospy.loginfo("frame_id is: %s", pc2_frame_id)

    odom_msg = rospy.wait_for_message("/odom", Odometry)
    current_quaternition = odom_msg.pose.pose.orientation

    pt = PointCloudTransformer(pc2_frame_id, distance)

    poses_tried_times = 0
    while not poses_valid(poses_result) and poses_tried_times < 10:
        # need reset poses first
        poses_tried_times += 1
        poses_result = []
        for box in boxes:
            xmin = box[0]
            xmax = box[1]
            ymin = box[2]
            ymax = box[3]

            uvs = []
            xstart = (xmin + xmax) / 2
            ystart = (ymin + ymax) / 2
            for x in range(xstart-5, xstart+5):
                for y in range(ystart-5, ystart+5):
                    uvs.append((x, y))

            data_out = pc2.read_points(pc2_msg, field_names=['x', 'y', 'z'], skip_nans=True, uvs=uvs)
            count = 0
            p = Point(0, 0, 0)
            for data in data_out:
                p.x += data[0]
                p.y += data[1]
                p.z += data[2]
                count += 1

            p.x /= count
            p.y /= count
            p.z /= count

            p = pt.transform(p)
            pose = Pose(p, current_quaternition)
            rospy.loginfo("pose is: %s",  pose)
            poses_result.append(pose)

    return poses_result


def pose_valid(pose):
    # avoid deep image inaccurate, you need to change num limit by the map
    x = abs(pose.position.x)
    y = abs(pose.position.y)
    return x < 20 and y < 20 and not isnan(x) and not isnan(y)


def poses_valid(poses):
    if len(poses) == 0:
        return False

    return all(pose_valid(pose) for pose in poses)


def save_img(path, img):
    cv2.imwrite(path, img)


def save_img_with_box(path, img, obj_name, boxes):
    for box in boxes:
        x1 = box[0]
        x2 = box[1]
        y1 = box[2]
        y2 = box[3]
        img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0))
        cv2.putText(img, obj_name, (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

    cv2.imwrite(path, img)


