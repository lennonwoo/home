"""
Body include something can move
"""
import serial

import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from base import RobotPart


class Leg(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def move(self, pose, frame_id='map'):
        time_limit = self.config.move_time_limit
        self.move_base.wait_for_server(rospy.Duration(time_limit))

        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(goal)

        finished_within_time = self.move_base.wait_for_result(rospy.Duration(time_limit))

        if finished_within_time:
            rospy.loginfo("Arrived at Pose:\nframe_id: %s\n %s", frame_id, pose)
            return True
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False


class Arm(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        port = self.config.arm_port_name
        baud = self.config.arm_baud

        if self.config.enable_arm:
            self.ser = serial.Serial(port, baud)
            self.ser.bytesize = serial.EIGHTBITS
            self.ser.timeout = 0.5

            self.connect_arm()

    def connect_arm(self):
        print("connect arm")
        self.ser.write(self.config.ARM_CONNECT_COMMAND)
        self.wait_until_arm_ready()

    def wait_until_arm_ready(self, wait_close=False):
        sleep_count = 0
        sleep_limit = 14 if wait_close else 25
        while sleep_count < sleep_limit:
            content_read = self.ser.read(2)
            print("sleep count: ", sleep_count, "  content read ", content_read)
            if content_read == "ok":
                return True
            else:
                sleep_count += 1
                # we already have serial read timeout = 0.5
                # rospy.sleep(0.5)

        print("wait arm ready time out, you need reset arm")
        return False

    def reset_arm(self):
        self.ser.write(self.config.ARM_RESET_COMMAND)
        print("wait reset_ok")
        return self.wait_until_arm_ready()

    def close_obj(self):
        self.ser.write(self.config.ARM_CLOSE_OBJ_COMMAND)
        print("wait close_ok")
        return self.wait_until_arm_ready(wait_close=True)

    def grasp_obj(self):
        self.ser.write(self.config.ARM_GRASP_OBJ_COMMAND)
        print("wait grasp_ok")
        return self.wait_until_arm_ready()

    def take_obj(self):
        self.ser.write(self.config.ARM_TAKE_OBJ_COMMAND)
        print("wait take_back_ok")
        return self.wait_until_arm_ready()

    def grasp(self, obj_name="kele"):
        if not self.config.enable_arm and obj_name not in self.config.arm_grasp_obj_list:
            return

        print("close arm")
        if not self.close_obj():
            print("I will reset arm for not close obj")
            return self.reset_arm()

        if obj_name in self.config.arm_grasp_obj_list:
            self.grasp_obj()
            # TODO(lennon) need root back?
            # self.robot.back()
            self.take_obj()
        else:
            self.reset_arm()
