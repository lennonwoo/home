#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <csc_nav2d_navigator/MoveToPosition2DAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <csc_nav2d_navigator/commands.h>

typedef actionlib::SimpleActionClient<csc_nav2d_navigator::MoveToPosition2DAction> MoveClient;

MoveClient* gMoveClient;

//这个节点只管转发goal到server，不管server的返回信息。
//后期是将这个部分替换掉，变成我们自己的转发，实现目标点发送
void receiveGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	csc_nav2d_navigator::MoveToPosition2DGoal goal;
	goal.target_pose.x = msg->pose.position.x;
	goal.target_pose.y = msg->pose.position.y;
	goal.target_pose.theta = tf::getYaw(msg->pose.orientation);
	goal.target_distance = 1.0;//目标点的容忍距离
	goal.target_angle = 0.1;//目标点的容忍角度 rad
	
	gMoveClient->sendGoal(goal);  // 客户端发送任务目标
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "SetGoal");
	ros::NodeHandle n;
	
	ros::Subscriber goalSubscriber = n.subscribe("/move_base_simple/goal", 1, &receiveGoal);//原始是goal，但是rviz发布的是/move_base_simple/goal
	gMoveClient = new MoveClient(NAV_MOVE_ACTION, true);
	gMoveClient->waitForServer();
	
	ros::spin();
	
	delete gMoveClient;
	return 0;
}
