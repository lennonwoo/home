"""
Body include something can move
"""
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class Leg:
    def __init__(self):
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    def move(self, pose):
        self.move_base.wait_for_server(rospy.Duration(30))

        goal = MoveBaseGoal()
        goal.target_pose.pose = pose
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()

        self.move_base.send_goal(goal)

        finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

        if finished_within_time:
            rospy.loginfo("Arrived at Pose: %s" % pose)
            return 'arrived'
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return 'retry'


class Arm:
    def __init__(self):
        pass
