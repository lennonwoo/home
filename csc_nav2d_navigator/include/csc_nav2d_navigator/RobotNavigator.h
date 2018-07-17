#include <ros/ros.h>
#include <nav_msgs/GetMap.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.h>
#include <csc_nav2d_navigator/MoveToPosition2DAction.h>
#include <csc_nav2d_navigator/GridMap.h>
#include <csc_nav2d_navigator/commands.h>
#include <csc_nav2d_navigator/MapInflationTool.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <queue>
#include <set>
#include <map>

typedef actionlib::SimpleActionServer<csc_nav2d_navigator::MoveToPosition2DAction> MoveActionServer;

typedef struct {
	double world_x;
	double world_y;
	double world_th;
	unsigned int index;
}plan_point;

class RobotNavigator
{
public:
	RobotNavigator();
	~RobotNavigator();

	bool receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	bool receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
	void receiveMoveGoal(const csc_nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal);

private:
	bool isLocalized();
	bool setCurrentPosition();
	bool setLocalCurrentPosition();
	bool getMap();
	bool getLocalMap(double originX, double originY, unsigned int width, unsigned int height);//xjh add
	void stop();
	void pause();
	bool correctGoalPose();
	bool generateLocalCommand();
	bool preparePlan();
	bool createLocalMap();
	bool createPlan();
	bool createLocalPlan();
	bool createLocalGoal();
	void publishPlan();
	void publishLocalPlan();
	void publishDJGridMap();
	bool GridMapWorldToLocalCell(unsigned int worldInCellX, unsigned int worldInCellY, unsigned int &localInCellX, unsigned int &localInCellY);
	bool GridMapLocalToWorldCell(unsigned int localInCellX, unsigned int localInCellY, unsigned int &worldInCellX, unsigned int &worldInCellY);
	bool GridMapLocalToWorldMeter(double localInMeterX, double localInMeterY, double &worldInMeterX, double &worldInMeterY);
	bool GridMapWorldToLocalCellCheck(unsigned int worldInCellX, unsigned int worldInCellY);
	bool GridMapLocalToWorldId(unsigned int localId, unsigned int &worldId);
	bool GridMapWorldToLocalId(unsigned int worldId, unsigned int &localId);

	bool RayTrackisClean(unsigned int endPointIndex);

	// Everything related to ROS
	tf::TransformListener mTfListener;
	ros::ServiceClient mGetMapClient;
	ros::Subscriber mGoalSubscriber;
	ros::Publisher mPlanPublisher;
	ros::Publisher mLocalPlanPublisher;
	ros::Publisher mMarkerPublisher;
	ros::Publisher mLocalMarkerPublisher;
	ros::Publisher mLocalDirectionMarkerPublisher;
	ros::Publisher mGridMapPublisher;//xjh add
	ros::Publisher mLocalGridMapPublisher;//xjh add
	ros::Publisher mLocalDJGridMapPublisher;
	ros::Publisher mLocalPlanPointsPublisher; // Zzhen add
	ros::Publisher mCommandPublisher;
	ros::ServiceServer mStopServer;
	ros::ServiceServer mPauseServer;

	std::string mMapFrame;
	std::string mRobotFrame;
	std::string mMoveActionTopic;

	MoveActionServer* mMoveActionServer;

	// Current status and goals
	bool mHasNewMap;
	bool mIsPaused;
	bool mIsStopped;
	bool mIsNavPaused;
	int mLocalPlanMissCounter;
	int mStatus;  // 目前导航的状态

	double mGoalTheta;  // 全局导航目标点的方向
	unsigned int mStartPoint;  // 全局导航的开始点
	unsigned int mGoalPoint;
	unsigned int mLocalStartPoint;  // 局部导航的开始点
	unsigned int mLocalGoalPoint;
	std::vector<plan_point> mglobalPlan_points;  // 全局规划的路径点(包含坐标和方向角)
	double mCurrentDirection;
	double mCurrentPositionX;
	double mCurrentPositionY;

	// Everything related to the global map and plan
	MapInflationTool mInflationTool;
	GridMap mCurrentMap;  // 全局地图
	double* mCurrentPlan;

	//在计算地图点是否是free时会用到mRobotRadius mInflationRadius，要求机器人半径比障碍半径小!
	double mInflationRadius;//障碍物膨胀半径 米
	double mRobotRadius;//机器人半径 米
	double mRobotRadius_a;//机器人半径 米
	unsigned int mCellInflationRadius;//障碍物膨胀半径对应costmap的格子数
	unsigned int mCellRobotRadius;//机器人半径对应costmap的格子数

	char mCostObstacle;  //初始化为100，占用栅格地图中表示占用
	char mCostLethal;
	char mCostMiddle;

	double mNavigationGoalDistance;//goal点的容忍误差 距离值 米
	double mNavigationGoalAngle;//goal点的容忍误差 角度值 rad
	double mNavigationHomingDistance;//到达goal周围时，发布的cmd不带避障

	double mRasterSize;

	double mDebug_show_height;

	//xjh local plan
	costmap_2d::Costmap2DROS* mLocalMap;
	costmap_2d::Costmap2D* mCostmap;
	MapInflationTool mLocalInflationTool;
	bool mLocalHasNewMap;
	GridMap mLocalCurrentMap;
	double* mLocalCurrentPlan;
	std::vector<plan_point> mlocalPlan_points;  // 局部规划的路径点(包含坐标和方向角)
	std::vector<unsigned int> mLocalPlanPointsInWorldCell;

	GridMap mLocalLastMap;//mLocalCurrentMap更新后并不代表就能找到局部路径，当局部路径查找失败时，还想让车体运动，就必须保存最近一次规划成功的map
	double* mLocalLastPlan;
};
