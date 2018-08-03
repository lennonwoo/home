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
            rospy.loginfo("Arrived at Pose:\nframe_id: %s\n %s" % (frame_id, pose))
            return True
        else:
            self.move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
            return False

    def debug(self):
        if not self.config.debug:
            return

        msg = rospy.wait_for_message("/odom", Odometry)
        rospy.loginfo(msg)


class Arm(RobotPart):
    def __init__(self, robot):
        RobotPart.__init__(self, robot)


if __name__ == '__main__':
    rospy.init_node('leg_test')
    from core import Robot
    robot = Robot()

    place_list = sorted([k for k in robot._nav_pose_dict.keys() if k.startswith('people_place')])
    for place in place_list:
        robot.nav_by_place_name(str(place))
        rospy.loginfo(place)
    rospy.spin()
