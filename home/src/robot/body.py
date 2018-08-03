"""
Body include something can move
"""
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

from base import RobotPart


class Leg(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def move(self, pose, frame_id='map'):
        self.move_base.wait_for_server(rospy.Duration(30))

        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(goal)

        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

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
