#!/usr/bin/env python
import rospy

from geometry_msgs.msg import PoseStamped
from xml.dom.minidom import Document


class Pos2XML():
    def __init__(self):
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)

        self.doc = Document()
        self.nav_poses = self.doc.createElement("nav_poses")
        self.doc.appendChild(self.nav_poses)

    def get_text_node(self, s):
        return self.doc.createTextNode(s)

    def generate_position_element(self, position, quat):
        pose = self.doc.createElement("pose")

        name = self.doc.createElement('name')
        name.appendChild(self.get_text_node("")

        position_ = self.doc.createElement("position")
        position_.appendChild(self.get_text_node(" ".join(str(i)
                                       for i in [position.x, position.y, position.z])))

        orientation_ = self.doc.createElement("orientation")
        orientation_.appendChild(self.get_text_node(" ".join(str(i)
                                          for i in [quat.x, quat.y, quat.z, quat.w])))

        pose.appendChild(name)
        pose.appendChild(position_)
        pose.appendChild(orientation_)

        return pose

    def callback(self, msg):
        position = msg.pose.position
        quat = msg.pose.orientation
        self.nav_poses.appendChild(self.generate_position_element(position, quat))

        self.debug(position, quat)

    def debug(self, position, quat):
        rospy.loginfo("Point Position: [ %f, %f, %f ]"%(position.x, position.y, position.z))
        rospy.loginfo("Quat Orientation: [ %f, %f, %f, %f]"%(quat.x, quat.y, quat.z, quat.w))
        print(self.doc.toprettyxml())
        print('-'*20)

    def write_xml(self):
        print("write_xml finally")
        filename = ".cache/poses.xml"
        f = open(filename, "w")
        f.write(self.doc.toprettyxml(indent="  "))
        f.close()


if __name__ == '__main__':
    rospy.init_node('pos_listener', disable_signals=True)

    pos2xml = Pos2XML()


    try:
        while not rospy.core.is_shutdown():
            rospy.rostime.wallsleep(0.5)
    except KeyboardInterrupt:
        pos2xml.write_xml()
        rospy.core.signal_shutdown('keyboard interrupt')
