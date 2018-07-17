#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <csc_nav2d_operator/cmd.h>
#include <csc_nav2d_navigator/RobotNavigator.h>
#include <csc_nav2d_msgs/localPlanPoints.h>


#define PI 3.14159265
#define FREQUENCY 10.0

using namespace ros;
using namespace tf;
using namespace std;

/* 构造函数 */
RobotNavigator::RobotNavigator()
{	
    ROS_WARN("Navigator initialize 1.");
    cout << "test" << endl;
    //xjh
    mLocalMap = new costmap_2d::Costmap2DROS("local_costmap", mTfListener);//这里相当于开启了一个costmap的node
    mRasterSize = mLocalMap->getCostmap()->getResolution();

    NodeHandle robotNode;

    std::string serviceName;
    robotNode.param("map_service", serviceName, std::string("get_map"));
    mGetMapClient = robotNode.serviceClient<nav_msgs::GetMap>(serviceName);  // mapserver的客户端

    robotNode.param("map_frame", mMapFrame, std::string("map"));
    robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
    robotNode.param("move_action_topic", mMoveActionTopic, std::string(NAV_MOVE_ACTION));
    robotNode.param("navigation_goal_distance", mNavigationGoalDistance, 1.0);
    robotNode.param("navigation_goal_angle", mNavigationGoalAngle, 0.1);
    robotNode.param("navigation_homing_distance", mNavigationHomingDistance, 3.0);

    ROS_INFO("Navigator initialize 2.");
    // 发布的话题
    mGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("globalGridMap",1);  // 全局地图(膨胀的)
    mLocalGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localGridMap",1);  // 局部地图()
    mLocalDJGridMapPublisher = robotNode.advertise<nav_msgs::OccupancyGrid>("localDJGridMap",1);
    mLocalPlanPointsPublisher = robotNode.advertise<csc_nav2d_msgs::localPlanPoints>("localPlanPoints", 10);  // 局部规划的路径点
    mCommandPublisher = robotNode.advertise<csc_nav2d_operator::cmd>("cmd", 1);

    // 服务
    mStopServer = robotNode.advertiseService(NAV_STOP_SERVICE, &RobotNavigator::receiveStop, this);
    mPauseServer = robotNode.advertiseService(NAV_PAUSE_SERVICE, &RobotNavigator::receivePause, this);
    mCurrentPlan = NULL;
    mLocalCurrentPlan = NULL;
    mLocalLastPlan = NULL;

    ROS_INFO("Navigator initialize 3.");
    NodeHandle navigatorNode("~/global_costmap/");  // 定义私有节点
    mPlanPublisher = navigatorNode.advertise<sensor_msgs::PointCloud>("globalPlan", 5);
    mLocalPlanPublisher = navigatorNode.advertise<sensor_msgs::PointCloud>("localPlan", 5);
    mMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("markers", 1, true);
    mLocalMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("localMarkers", 1, true);
    mLocalDirectionMarkerPublisher = navigatorNode.advertise<visualization_msgs::Marker>("LocalDirectionMarkers", 1, true);

    // Get parameters
    navigatorNode.param("map_inflation_radius", mInflationRadius, 1.0);
    navigatorNode.param("robot_radius", mRobotRadius, 0.4);
    navigatorNode.param("robot_radius_a", mRobotRadius_a, 0.4);

    ROS_INFO("Navigator initialize 4.");
    // 根据机器人半径和膨胀半径设定一个0~100的阈值，用于判断某个地图点是否是free(无碰撞)，必须要求机器人半径小于膨胀半径！
    mCostObstacle = 100;
    mCostLethal = (1.0 - (mRobotRadius / mInflationRadius)) * (double)mCostObstacle;
    mCostMiddle = mCostLethal*0.5;

    // Apply tf_prefix to all used frame-id's
    mRobotFrame = mTfListener.resolve(mRobotFrame);
    mMapFrame = mTfListener.resolve(mMapFrame);

    // Create action servers 定义一个服务器
    mMoveActionServer = new MoveActionServer(mMoveActionTopic, boost::bind(&RobotNavigator::receiveMoveGoal, this, _1), false);
    mMoveActionServer->start();  // 服务器开始运行

    ROS_INFO("Navigator initialize 5.");
    mLocalHasNewMap = false;
    mHasNewMap = false;
    mIsStopped = false;
    mIsPaused = false;
    mIsNavPaused = false;
    mStatus = NAV_ST_IDLE;
    mCellInflationRadius = 0;
    mLocalPlanMissCounter = 0;
    mDebug_show_height = 0.0;
}

RobotNavigator::~RobotNavigator()
{
    delete[] mCurrentPlan;
    delete[] mLocalCurrentPlan;
    delete[] mLocalLastPlan;
    delete mMoveActionServer;
}

//从地图服务器中获得地图，并计算膨胀半径
bool RobotNavigator::getMap()
{	
    if(mHasNewMap)
        return true;

    if(!mGetMapClient.isValid())
    {
        ROS_ERROR("GetMap-Client is invalid!");
        return false;
    }

    nav_msgs::GetMap srv;
    // 向ros的mapserver请求地图，这里只需要全局地图
    if(!mGetMapClient.call(srv))
    {
        ROS_INFO("Could not get a map.");
        return false;
    }
    mCurrentMap.update(srv.response.map);  //mapserver返回的是占用栅格地图。现在是0.05分辨率。

    if(mCurrentPlan) delete[] mCurrentPlan;
    mCurrentPlan = new double[mCurrentMap.getSize()];  // 分配空间,栅格地图的大小(像素数量)

    if(mCellInflationRadius == 0)
    {
        ROS_INFO("Navigator is now initialized.");
        mCellInflationRadius = mInflationRadius / mCurrentMap.getResolution();//变成了格子的占用数(膨胀半径)
        mCellRobotRadius = mRobotRadius / mCurrentMap.getResolution();//变成了格子的占用数(机器人的半径)
        mInflationTool.computeCaches(mCellInflationRadius);//计算mInflationTool膨胀代价，障碍物距离越远值越小，栅格地图（不是costmap，是概率占用地图），0是free
        mCurrentMap.setLethalCost(mCostLethal);//刷新阈值，用于判断某个地图点是否是free
    }

    mHasNewMap = true;
    return true;
}

//基于机器人当前位置，局部costmap，全局mCurrentMap，
//在局部costmap 范围 中搜索mCurrentMap中的最小值，如果这个值没有被占用设为目标最小，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
//如果被占用了，则搜索costmap 范围以外的一个mCurrentMap中的最小值，设为目标，基于costmap的障碍物和mCurrentMap的障碍物叠加重新计算DJ代价，计算新的局部路径
//输入 mCurrentMap中的位置 double型
//输出  bool mLocalHasNewMap; GridMap mLocalCurrentMap; double* mLocalCurrentPlan;
//送入的参数所包含的区域有 机器人当前位置， costmap中从当前位置出发，最远的全局plan点（cost最小的点）
bool RobotNavigator::getLocalMap(double originX, double originY, unsigned int widthInCostMap, unsigned int heightInCostMap)
{
    if(mLocalHasNewMap) return true;//时间没到 不更新

    mCostmap = mLocalMap->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

    // 局部地图在全局地图中的长宽
    unsigned int widthInCurrentMap = widthInCostMap * mCostmap->getResolution() /  mCurrentMap.getResolution();
    unsigned int heightInCurrentMap = heightInCostMap * mCostmap->getResolution() /  mCurrentMap.getResolution();

    // 局部地图原点在全局地图中的位置。要加0.5，否则会有一格的偏移
    unsigned int idX = (originX - mCurrentMap.getOriginX()) / mCurrentMap.getResolution() + 0.5;
    unsigned int idY = (originY - mCurrentMap.getOriginY()) / mCurrentMap.getResolution() + 0.5;

    //这里有问题 width和height是基于costmap的分辨率计算的 10m距离 全局地图分辨率不同 对应的宽和高也不同
    if(idX+widthInCurrentMap > mCurrentMap.getWidth() || idY+heightInCurrentMap > mCurrentMap.getHeight())
    {
        ROS_ERROR("getLocalMap fail!idX=%d idY=%d w=%d h=%d", idX, idY, widthInCurrentMap, heightInCurrentMap);
        return false;
    }

    // 基于全局栅格地图再创建一个小的局部栅格地图
    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(widthInCurrentMap*heightInCurrentMap);
    tempMap.info.height = heightInCurrentMap;
    tempMap.info.width = widthInCurrentMap;
    tempMap.info.resolution = mCurrentMap.getResolution();
    tempMap.info.origin.position.x = originX;
    tempMap.info.origin.position.y = originY;
    tempMap.info.origin.position.z = mDebug_show_height;

    int id=0;
    char mapValue_temp;
    for(int j=0; j<heightInCurrentMap; j++)
    {
        for(int i=0;i<widthInCurrentMap; i++)
        {
            mapValue_temp = mCurrentMap.getData(idX + i,idY + j);
            if(mapValue_temp > 0 && mapValue_temp < mCostObstacle)
                mapValue_temp = 0;
            tempMap.data[id++] = mapValue_temp;
        }
    }

    mLocalCurrentMap.update(tempMap);  //更新局部的栅格地图

    double wx, wy;
    int idLocalX, idLocalY;

    for(unsigned int mx=0; mx<mCostmap->getSizeInCellsX(); mx++)
    {
        for(unsigned int my=0; my<mCostmap->getSizeInCellsY(); my++)
        {
            if(mCostmap->getCost(mx,my) == costmap_2d::LETHAL_OBSTACLE )  // 致命障碍
            {
                mCostmap->mapToWorld(mx, my, wx, wy);//这个句话是将costmap中的某个点的id坐标转化到meter坐标。全局map坐标系下
                idLocalX = (wx-mLocalCurrentMap.getOriginX())/mLocalCurrentMap.getResolution();
                idLocalY = (wy-mLocalCurrentMap.getOriginY())/mLocalCurrentMap.getResolution();
                mLocalCurrentMap.setData(idLocalX, idLocalY, mCostObstacle);  // 设为100
            }
        }
    }

    if(mLocalCurrentPlan) delete[] mLocalCurrentPlan;
    mLocalCurrentPlan = new double[mLocalCurrentMap.getSize()];

    mCostLethal = (1.0 - (mRobotRadius_a / mInflationRadius)) * (double)mCostObstacle;
    mLocalCurrentMap.setLethalCost(mCostLethal);  //刷新阈值，用于判断某个地图点是否是free
    //mLocalHasNewMap = true;
    return true;
}

bool RobotNavigator::GridMapWorldToLocalCellCheck(unsigned int worldInCellX, unsigned int worldInCellY)
{
    double worldInMeterX = ((worldInCellX+ 0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX();
    double worldInMeterY = ((worldInCellY+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY();

    if(worldInMeterX >= mLocalCurrentMap.getOriginX() + (mLocalCurrentMap.getWidth()-0.5)* mLocalCurrentMap.getResolution() || \
       worldInMeterX <= mLocalCurrentMap.getOriginX() || worldInMeterY <= mLocalCurrentMap.getOriginY() || \
       worldInMeterY >= mLocalCurrentMap.getOriginY() + (mLocalCurrentMap.getHeight()-0.5)* mLocalCurrentMap.getResolution())
    {
        //ROS_WARN("GridMapWorldToLocalCell WorldX=%d, WorldY=%d !LocalX=%d LocalY=%d worldInMeterX=%f worldInMeterY=%f", worldInCellX,  worldInCellY, localInCellX, localInCellY, worldInMeterX, worldInMeterY);
        //ROS_WARN("LocalOriginX=%f LocalOriginY=%f LocalWidth=%f LocalHeight=%f",mLocalCurrentMap.getOriginX(), mLocalCurrentMap.getOriginY(), mLocalCurrentMap.getWidth()* mLocalCurrentMap.getResolution(), mLocalCurrentMap.getHeight()* mLocalCurrentMap.getResolution());
        return false;
    }
    return true;
}

//全部转化到物理尺度meter下 cell+0.5后*分辨率。从meter变成cell不用转化
bool RobotNavigator::GridMapWorldToLocalCell(unsigned int worldInCellX, unsigned int worldInCellY, unsigned int &localInCellX, unsigned int &localInCellY)
{
    double worldInMeterX = ((worldInCellX+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX();
    double worldInMeterY = ((worldInCellY+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY();

    if(worldInMeterX >= mLocalCurrentMap.getOriginX() + (mLocalCurrentMap.getWidth()-0.5)* mLocalCurrentMap.getResolution() || \
       worldInMeterX <= mLocalCurrentMap.getOriginX() || \
       worldInMeterY <= mLocalCurrentMap.getOriginY() || \
       worldInMeterY >= mLocalCurrentMap.getOriginY() + (mLocalCurrentMap.getHeight()-0.5)* mLocalCurrentMap.getResolution())
    {
        //ROS_WARN("GridMapWorldToLocalCell WorldX=%d, WorldY=%d !LocalX=%d LocalY=%d worldInMeterX=%f worldInMeterY=%f", worldInCellX,  worldInCellY, localInCellX, localInCellY, worldInMeterX, worldInMeterY);
        //ROS_WARN("LocalOriginX=%f LocalOriginY=%f LocalWidth=%f LocalHeight=%f",mLocalCurrentMap.getOriginX(), mLocalCurrentMap.getOriginY(), mLocalCurrentMap.getWidth()* mLocalCurrentMap.getResolution(), mLocalCurrentMap.getHeight()* mLocalCurrentMap.getResolution());
        return false;
    }

    localInCellX = (worldInMeterX - mLocalCurrentMap.getOriginX()) / mLocalCurrentMap.getResolution();
    localInCellY = (worldInMeterY - mLocalCurrentMap.getOriginY()) / mLocalCurrentMap.getResolution();
    //ROS_WARN("!!!goalX=%d, goalY=%d !goalLocalX=%d goalLocalY=%d", worldInCellX,  worldInCellY, localInCellX, localInCellY);
    return true;
}

bool RobotNavigator::GridMapLocalToWorldCell(unsigned int localInCellX, unsigned int localInCellY, unsigned int &worldInCellX, unsigned int &worldInCellY)
{
    worldInCellX = ((localInCellX+0.5)*mLocalCurrentMap.getResolution() + mLocalCurrentMap.getOriginX() - mCurrentMap.getOriginX())/mLocalCurrentMap.getResolution();
    worldInCellY = ((localInCellY+0.5)*mLocalCurrentMap.getResolution() + mLocalCurrentMap.getOriginY() - mCurrentMap.getOriginY())/mLocalCurrentMap.getResolution();
    return true;
}

bool RobotNavigator::GridMapLocalToWorldMeter(double localInMeterX, double localInMeterY, double &worldInMeterX, double &worldInMeterY)
{
    //因为loaclmap的meter的0在图像中心 不在右下角
    worldInMeterX = localInMeterX;// + (mLocalCurrentMap.getOriginX() - mCurrentMap.getOriginX());
    worldInMeterY = localInMeterY;// + (mLocalCurrentMap.getOriginY() - mCurrentMap.getOriginY());
    return true;
}

bool RobotNavigator::GridMapLocalToWorldId(unsigned int localId, unsigned int &worldId)
{
    unsigned int xInWorldCell, yInWorldCell;
    unsigned int xInLocalCell, yInLocalCell;

    mLocalCurrentMap.getCoordinates(xInLocalCell, yInLocalCell, localId);
    if(GridMapLocalToWorldCell(xInLocalCell, yInLocalCell, xInWorldCell, yInWorldCell))
    {
        mCurrentMap.getIndex(xInWorldCell, yInWorldCell, worldId);
        return true;
    }
    else
        return false;
}

bool RobotNavigator::GridMapWorldToLocalId(unsigned int worldId, unsigned int &localId)
{
    unsigned int xInWorldCell, yInWorldCell;
    unsigned int xInLocalCell, yInLocalCell;

    mCurrentMap.getCoordinates(xInWorldCell, yInWorldCell, worldId);
    if(GridMapWorldToLocalCell(xInWorldCell, yInWorldCell, xInLocalCell, yInLocalCell))
    {
        mLocalCurrentMap.getIndex(xInLocalCell, yInLocalCell, localId);
        return true;
    }
    else
    {
        return false;
    }
}

typedef std::multimap<double, unsigned int> Queue;  // 默认是从大到小排序
typedef std::pair<double, unsigned int> Entry;

//清除车体位置内的障碍物，更新膨胀后的全局地图。
bool RobotNavigator::preparePlan()
{
    // Get the current map
    if(!getMap()) // return false;
    {
        if(mCellInflationRadius == 0) return false;
        ROS_WARN("Could not get a new map, trying to go with the old one...");
    }

    // Where am I?
    if(!setCurrentPosition()) return false;

    // Clear robot footprint in map
    //把机器人在局部地图中的区域设置为free
    unsigned int x = 0, y = 0;
    if(mCurrentMap.getCoordinates(x, y, mStartPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
                mCurrentMap.setData(x+i, y+j, 0);

    mInflationTool.inflateMap(&mCurrentMap);  // 膨胀地图。

    // 发布膨胀后的地图
    mGridMapPublisher.publish(mCurrentMap.getMap());

    return true;
}

//假如goal点是被占用的，就需要调节goal的位置，直到找到一个free的点，
//这个会有点问题。。目标点变了。。
bool RobotNavigator::correctGoalPose()
{
    ROS_INFO("Correct Goal Pose!!!");
    // Reset the plan
    int mapSize = mCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mCurrentPlan[i] = -1;
    }

    // Initialize the queue with the goal point
    Queue queue;
    Entry goal(0.0, mGoalPoint);
    queue.insert(goal);//这里都不检查可行性？？
    mCurrentPlan[mGoalPoint] = 0;

    Queue::iterator next;
    double linear = mCurrentMap.getResolution();

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        double distance = next->first;
        unsigned int index = next->second;
        queue.erase(next);

        if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;

        // Add all adjacent cells
        std::vector<unsigned int> neighbors = mCurrentMap.getNeighbors(index);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            if(mCurrentMap.isFree(neighbors[i]))
            {
                mGoalPoint = neighbors[i];
                return true;
            }else
            {
                double newDistance = distance + linear;
                if(mCurrentPlan[neighbors[i]] == -1)
                {
                    queue.insert(Entry(newDistance, neighbors[i]));
                    mCurrentPlan[neighbors[i]] = newDistance;
                }
            }
        }
    }
    return false;
}


//局部规划使用1.5倍大小的局部地图，其中叠加1倍大小的局部障碍物，换言之 靠边上的0.5倍的区域不存在局部障碍物，只存在全局障碍物 用于导航规划。
bool RobotNavigator::createLocalMap()
{
    //必须要加这两句话 否则会返回空的costmap！
    mCostmap = mLocalMap->getCostmap();
    //ROS_INFO("Creat local cost map!!!");
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));

    //注意 这个是costmap中的cell 他和mapserver发布的分辨率会不同
    // 实际创建的局部地图比costmap大1.5倍
    unsigned int localMapWidthInCell  = mCostmap->getSizeInMetersX() / mCostmap->getResolution() * 1.2;
    unsigned int localMapHeightInCell = mCostmap->getSizeInMetersY() / mCostmap->getResolution() * 1.2;

    // 得到局部地图左上角的坐标
    double localMapStartInMeterX = mCurrentPositionX - (localMapWidthInCell) * mCostmap->getResolution() / 2.0;
    double localMapStartInMeterY = mCurrentPositionY - (localMapHeightInCell) * mCostmap->getResolution() / 2.0;

    // 基于全局地图和代价地图创建一个局部地图
    if(!getLocalMap(localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell))
    {
        ROS_WARN("Could not get a new local map, trying to go with the old one...");
        return false;//这个应该是不会发生的
    }
    //ROS_INFO("mCurrentPositionX=%f, mCurrentPositionY=%f, MeterX=%f , MeterY=%f , WidthInCell=%d, HeightInCell=%d",mCurrentPositionX,mCurrentPositionY,localMapStartInMeterX, localMapStartInMeterY, localMapWidthInCell, localMapHeightInCell);

    mLocalInflationTool.computeCaches(mInflationRadius/mLocalCurrentMap.getResolution());  // 设置了局部地图的膨胀半径
    mLocalInflationTool.inflateMap(&mLocalCurrentMap);  //膨胀地图。

    //因为人跟着，这个估计还是要清除的
    //实际上因为全局定位会有偏差，导致在实验室门口狭窄的地方，机器人被定位到了全局map的障碍物点上，导致无法找到局部goal。
    // Clear robot footprint in map

    /*unsigned int x = 0, y = 0;
    if(mLocalCurrentMap.getCoordinates(x, y, mLocalStartPoint))
        for(int i = -mCellRobotRadius; i < (int)mCellRobotRadius; i++)
            for(int j = -mCellRobotRadius; j < (int)mCellRobotRadius; j++)
                mLocalCurrentMap.setData(x+i, y+j, 0);  //把机器人在局部地图中的区域设置为free
                */
    // 发布局部的栅格地图（costmap）
    mLocalGridMapPublisher.publish(mLocalCurrentMap.getMap());

    return true;
}

//在创建全局地图时会清除机器人区域，然后做全局plan计算
//当机器人要获得localplan时要根据全局plan找到localGoal
//但是由于全局plan只在初始化时会清除机器人区域，之后不会在地图中清除对应的区域，因此在做局部规划时，将当前机器人放进去算会导致在计算局部goal时卡死，周围没有空闲区域。
//因为全局定位难免有偏差
bool RobotNavigator::createLocalGoal()
{
    //要查找局部地图中的goal点，基于已知的全局mCurrentPlan
    unsigned int index = mStartPoint;
    std::vector<std::pair<double, double> > points;

    //因为mCurrentPlan是一开始规划时就确定的，有可能当前位置开到了mCurrentPlan中值为-1的无效地方导致无法查找到局部goal
    //因为地图是统一膨胀的，再加上地图并不标的那么精确，车子在靠近路边缘时会进入mCurrentPlan中值为-1的无效地方。
    //这里查找车子以及周围1m内的空闲点，同时存在mCurrentPlan值的点（不是-1），找一个最近点作为车子的起始点进行规划
    if(mCurrentPlan[index] == -1)//如果初始位置不空闲，就需要修改初始位置
    {
        unsigned int start_x, start_y;
        mCurrentMap.getCoordinates(start_x, start_y, index);
        int RSize = 1.0/mCurrentMap.getResolution();
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index, RSize);//获得周围8个领域
        int lenth = 100;
        for(int i=0; i<neighbors.size(); i++)
        {
            if(mCurrentPlan[neighbors[i]] != -1)
            {
                unsigned int x = 0, y = 0;
                mCurrentMap.getCoordinates(x, y, neighbors[i]);
                int lenth_temp = abs((int)start_x - (int)x) + abs((int)start_y - (int)y);
                if(lenth_temp < lenth)  // 找最近的点
                {
                    lenth = lenth_temp;
                    index = neighbors[i];
                }
            }
        }
        if(lenth == 100)//在机器人为中心的一米内也没找到空闲点
        {
            ROS_ERROR("createLocalGoal Fail! Can not find a free start point ! mStartPoint = %d",mStartPoint);
            return false;
        }
        ROS_WARN("mStartPoint %d is not free, adjust to point %d", mStartPoint, index);
        mStartPoint = index;//在查找局部goal时就修改掉当前位置，后面做局部路径规划时就不用再次查找起始点了。
        setLocalCurrentPosition();
    }

    // 基于全局地图寻找到目标点的最短路径，一直到超出局部地图边界为止，把边界上的点作为局部路径规划的目标点
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mCurrentMap.getCoordinates(x,y,index)) {
            points.push_back(std::pair<double, double>(
                                 ((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(),
                                 ((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY()
                                 ));
        }

        //将map中的点转化到局部地图中
        //必须先检查这个边界条件！
        unsigned int worldInCellX, worldInCellY;
        mCurrentMap.getCoordinates(worldInCellX, worldInCellY, index);
        unsigned int localInCellX, localInCellY;
        if(!GridMapWorldToLocalCellCheck(worldInCellX, worldInCellY))
        {
            points.pop_back();//删除最后一个点，保证所有点都在局部地图中
            break;  //超出局部地图就停止搜索。但是最后一个点已经被保存了，刚好处于局部地图的外面
        }

        if(mCurrentPlan[index] == 0)
        {
            //ROS_INFO("Find value 0 index = %d",index);
            break;  //迭代到了goal
        }

        unsigned int next_index = index;
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index);
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[next_index])
                next_index = neighbors[i];
        }

        if(index == next_index)
        {
            ROS_INFO("index == next_index");
            ROS_INFO("mCurrentPlan[next_index]=%f", mCurrentPlan[next_index]);
            break;//当周围没有空闲区域时
        }
        index = next_index;
    }

    double goalWorldX, goalWorldY;
    if(points.size()>0)
    {
        // 局部路径规划目标点的世界坐标（m为单位）
        goalWorldX = points.back().first;
        goalWorldY = points.back().second;
        //ROS_INFO("goalWorld size=%d", points.size());
    }
    else
    {
        goalWorldX = points[0].first;
        goalWorldY = points[0].second;
        //这样写有可能内存溢出，后期要修改，局部找路径找不到的情况。
        ROS_ERROR("goalWorld size is 0");
        return false;
    }

    unsigned int goalLocalX, goalLocalY;

    // 换算到全局栅格地图坐标
    int goalX =  (double)(goalWorldX - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
    int goalY =  (double)(goalWorldY - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
    if(goalX < 0) goalX = 0;
    if(goalX >= (int)mCurrentMap.getWidth()) goalX = mCurrentMap.getWidth() - 1;
    if(goalY < 0) goalY = 0;
    if(goalY >= (int)mCurrentMap.getHeight()) goalY = mCurrentMap.getHeight() - 1;

    // 再转换到局部栅格地图坐标
    GridMapWorldToLocalCell(goalX, goalY, goalLocalX, goalLocalY);

    //ROS_INFO("reday to LocalPlan goalgoalX=%d, goalMgoalY=%d !goalLocalX=%d goalLocalY=%d", goalX, goalY, goalLocalX, goalLocalY);
    // 获得局部路径规划goal的id号，用于下一步规划
    if(mLocalCurrentMap.getIndex(goalLocalX, goalLocalY, mLocalGoalPoint))
    {
        return true;
    }
    else
    {
        ROS_ERROR("Couldn't convert mLocalGoalPoint goalLocalX=%d goalLocalY=%d mLocalGoalPoint=%d", goalLocalX, goalLocalY, mLocalGoalPoint);
        return false;
    }
}

bool RobotNavigator::createPlan()
{	
    ROS_DEBUG("Map-Value of goal point is %d, lethal threshold is %d.", mCurrentMap.getData(mGoalPoint), mCostLethal);

    unsigned int goal_x = 0, goal_y = 0;
    if(mCurrentMap.getCoordinates(goal_x,goal_y,mGoalPoint))
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mCurrentMap.getOriginX() + (((double)goal_x+0.5) * mCurrentMap.getResolution());
        marker.pose.position.y = mCurrentMap.getOriginY() + (((double)goal_y+0.5) * mCurrentMap.getResolution());
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = mCurrentMap.getResolution() * 1.0;
        marker.scale.y = mCurrentMap.getResolution() * 1.0;
        marker.scale.z = 3.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        mMarkerPublisher.publish(marker);  // 发布一个marker(目标点箭头)
    }else
    {
        ROS_ERROR("Couldn't ressolve goal point coordinates!");
    }

    Queue queue;

    // Reset the plan
    int mapSize = mCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mCurrentPlan[i] = -1;
    }

    if(mCurrentMap.isFree(mGoalPoint))
    {
        queue.insert(Entry(0.0, mGoalPoint));
        mCurrentPlan[mGoalPoint] = 0;  // 目标点设置为0(路径规划是从目标点开始搜索的，所以目标点代价为0)
    }
    else
    {
        // Initialize the queue with area around the goal point
        int reach = mCellRobotRadius + (1.0 / mCurrentMap.getResolution());
        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(mGoalPoint, reach);

        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            queue.insert(Entry(0.0, neighbors[i]));
            mCurrentPlan[neighbors[i]] = 0;
        }
        //如果goal被占用，则寻找周围车身半径+1m范围的区域内所有的空闲区域，都设置为0的代价。
    }

    Queue::iterator next;
    double distance;
    unsigned int x, y, index;

    unsigned int start_x=0, start_y=0;
    if(!mCurrentMap.getCoordinates(start_x, start_y, mStartPoint)) return false;

    double linear = mCurrentMap.getResolution();  // 线性距离
    double diagonal = std::sqrt(2.0) * linear;  // 对角线距离

    int count = 0;

    ROS_INFO("Do full search with Dijkstra-Algorithm");
    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        //count++;
        // Get the nearest cell from the queue
        next = queue.begin();  // 获得distance(代价)最小的格子
        distance = next->first;
        index = next->second;
        queue.erase(next);  // 将这个格子从“开启列表”清除

        //当地图中的点已经有代价，并且这个点代价比distance小（因为被其他相邻节点更新了），就不需要再更新这个节点。
        if(mCurrentPlan[index] >= 0 && mCurrentPlan[index] < distance) continue;

        //注释掉这个break是为了防止车子在导航中途因为避障碍，开向其他地方后无法再找到周围的迭代方向
        //这里是全局规划，反正只做一次，就慢慢的规划好了。
        if(index == mStartPoint)
        {
            ROS_ERROR("break...");
            //break;
        }

        std::vector<unsigned int> ind;
        ind = mCurrentMap.getNeighbors(index, true);  // 得到当前格的八个邻域
        for(unsigned int it = 0; it < ind.size(); it++)
        {
            unsigned int i = ind[it];

            if(mCurrentMap.isFree(i))  // 如果它是可通过(未被占用)的
            {
                double delta = (it < 4) ? linear : diagonal;  // 前4个是相邻关系，后4个是对角线关系，距离不同
                double newDistance = mCurrentPlan[index] + delta + \
                                     (10 * delta * (double)mCurrentMap.getData(i) / (double)mCostObstacle);//最后一项，在基本的DJ算法上，增加cost的代价。(double)mCurrentMap.getData(i)/(double)mCostObstacle是0-1的值，所以放大10倍

                //std::cout<< "newDistance=" << newDistance << std::endl;
                // 代价为空或者新算出的距离更小
                if(mCurrentPlan[i] == -1 || newDistance < mCurrentPlan[i])
                {
                    if(!mCurrentMap.getCoordinates(x, y, i)) continue;

                    count++;
                    // 增加启发函数，变为A*算法
                    int dx = std::abs((int)x-(int)start_x);
                    int dy = std::abs((int)y-(int)start_y);
                    double heuristic = linear*(dx + dy) + (diagonal - 2*linear) * std::min(dx, dy);

                    double priority = newDistance + heuristic;

                    queue.insert(Entry(newDistance, i));
                    //std::cout << "heuristic=" << heuristic << "    newDistance=" << newDistance << std::endl;
                    //queue.insert(Entry(newDistance, i));
                    mCurrentPlan[i] = newDistance;
                }
            }
        }
    }

    ROS_INFO("search count:%d -----------", count);

    if(mCurrentPlan[mStartPoint] < 0)
    {
        ROS_ERROR("In createPlan. No way between robot and goal! mStartPoint = %d", mStartPoint);
        return false;
    }

    //以上是将整张地图根据目标点进行cost计算，相当暴力和耗时的方法，没有任何优化
    return true;
}

bool RobotNavigator::createLocalPlan()
{
    unsigned int goal_x = 0, goal_y = 0;
    if(mLocalCurrentMap.getCoordinates(goal_x, goal_y, mLocalGoalPoint)) //传进来的就是局部地图坐标系下的目标点
    {
        unsigned int world_goal_x, world_goal_y;
        GridMapLocalToWorldCell(goal_x, goal_y, world_goal_x, world_goal_y);

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mCurrentMap.getOriginX() + (((double)world_goal_x) * mCurrentMap.getResolution());
        marker.pose.position.y = mCurrentMap.getOriginY() + (((double)world_goal_y) * mCurrentMap.getResolution());
        marker.pose.position.z = mDebug_show_height;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        //mLocalMarkerPublisher.publish(marker);
    }
    else
    {
        ROS_ERROR("Couldn't ressolve goal point coordinates!");
        return false;
    }

    Queue queue;

    // Reset the plan
    int mapSize = mLocalCurrentMap.getSize();
    for(int i = 0; i < mapSize; i++)
    {
        mLocalCurrentPlan[i] = -1;
    }
    //ROS_INFO("mLocalGoalPoint=%d, goal_x=%d, goal_y=%d", mLocalGoalPoint, goal_x, goal_y);

    if(mLocalCurrentMap.isFree(mLocalGoalPoint))
    {
        queue.insert(Entry(0.0, mLocalGoalPoint));
        mLocalCurrentPlan[mLocalGoalPoint] = 0;
    }
    else
    {
        //在局部地图中，这个条件一般情况不会成立
        //因为全局规划路线时就不会将被占用的点当作路径点
        //但是当靠近goal，goal处于局部costmap区域中（mLocalCurrentMap是1.5倍的costmap大小）就会出现这个问题
        //如果目标点，以及周围1m内都被障碍物占用了，这里的写法是有bug的。
        // Initialize the queue with area around the goal point
        int reach = mCellRobotRadius + (1.0 / mLocalCurrentMap.getResolution());
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(mLocalGoalPoint, reach);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            queue.insert(Entry(0.0, neighbors[i]));
            mLocalCurrentPlan[neighbors[i]] = 0;
        }
        ROS_WARN("local goal is not free! find free neighbors number %d, reach=%d", (int)neighbors.size(), reach);
        //如果goal被占用，则寻找周围车身半径+1m范围的区域内所有的空闲区域，都设置为0的代价。
    }
    //将上一次的最短路径转化到当前map坐标下。
    std::vector<unsigned int> lastLocalPlanInd;
    for(unsigned int i=0; i<mLocalPlanPointsInWorldCell.size(); i++)
    {
        unsigned int lastLocalPlanId;
        if(GridMapWorldToLocalId(mLocalPlanPointsInWorldCell[i], lastLocalPlanId))
        {
            lastLocalPlanInd.push_back(lastLocalPlanId);
        }
    }

    Queue::iterator next;
    double distance;
    unsigned int x, y, index;
    unsigned int start_x=0, start_y=0;
    if(!mCurrentMap.getCoordinates(start_x, start_y, mLocalStartPoint)) return false;

    double linear = mLocalCurrentMap.getResolution();
    double diagonal = std::sqrt(2.0) * linear;

    // Do full search with Dijkstra-Algorithm
    while(!queue.empty())
    {
        // Get the nearest cell from the queue
        next = queue.begin();
        distance = next->first;
        index = next->second;
        queue.erase(next);

        //当地图中的点已经有代价，并且这个点代价比distance小（因为被其他相邻节点更新了），就不需要再更新这个节点。
        //if(mLocalCurrentPlan[index] >= 0 && mLocalCurrentPlan[index] < distance) continue;

        //注释掉这个break是为了防止车子在导航中途因为避障碍，开向其他地方后无法再找到周围的最优迭代方向
        //个人觉得在局部地图中规划时，这个break可以加
        if(index == mLocalStartPoint) break;

        // Add all adjacent cells
        if(!mLocalCurrentMap.getCoordinates(x, y, index)) continue;

        std::vector<unsigned int> ind;
        ind = mLocalCurrentMap.getNeighbors(index, true);

        for(unsigned int it = 0; it < ind.size(); it++)
        {
            unsigned int i = ind[it];
            if(mLocalCurrentMap.isFree(i))
            {
                double delta = (it < 4) ? linear : diagonal;
                //增加上一帧的最优路径奖励
                auto itt = find(lastLocalPlanInd.begin(),lastLocalPlanInd.end(), i);
                if (itt != lastLocalPlanInd.end())
                {
                    //vec中存在value值
                    delta = delta;
                }
                else
                {
                    //vec中不存在value值
                    delta = 1.1*delta;
                }

                double newDistance = mLocalCurrentPlan[index] + delta + \
                                     (10 * delta * (double)mLocalCurrentMap.getData(i) / (double)mCostObstacle);//最后一项，在基本的DJ算法上，增加cost的代价。(double)mCurrentMap.getData(i) / (double)mCostObstacle是0-1的值，所以放大10倍

                if(mLocalCurrentPlan[i] == -1 || newDistance < mLocalCurrentPlan[i])
                {
                    if(!mLocalCurrentMap.getCoordinates(x, y, i)) continue;

                    // 增加启发函数
                    int dx = std::abs((int)x-(int)start_x);
                    int dy = std::abs((int)y-(int)start_y);
                    double heuristic = linear*(dx + dy) + (diagonal - 2*linear) * std::min(dx, dy);

                    double priority = newDistance + heuristic;

                    queue.insert(Entry(priority, i));
                    mLocalCurrentPlan[i] = newDistance;
                }
            }
        }
    }

    //DJ结束以后检查初始位置是否是可行的
    if(mLocalCurrentPlan[mLocalStartPoint] < 0)
    {
        ROS_WARN("In createLocalPlan. No way between robot and goal!");
        return false;
    }
    return true;
}

void RobotNavigator::publishPlan()
{
    unsigned int index = mStartPoint;  // 路径规划的开始点
    //std::vector<std::pair<double, double> > points;

    mglobalPlan_points.clear();

    while(true)
    {
        unsigned int x = 0, y = 0;

        if(mCurrentMap.getCoordinates(x,y,index))
        {
            mglobalPlan_points.push_back({
                                             ((x+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginX(),
                                             ((y+0.5) * mCurrentMap.getResolution()) + mCurrentMap.getOriginY(),
                                             0.0,
                                             index
                                         });
        }

        if(mCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;

        std::vector<unsigned int> neighbors = mCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mCurrentPlan[neighbors[i]] >= 0 && mCurrentPlan[neighbors[i]] < mCurrentPlan[next_index])
                next_index = neighbors[i];
        }

        if(index == next_index) break;//这个条件，暂时想不出来有什么可能会落入
        index = next_index;
    }

    //这里是全局地图坐标下的点
    sensor_msgs::PointCloud plan_msg;
    plan_msg.header.frame_id = mMapFrame.c_str();
    plan_msg.header.stamp = Time::now();

    sensor_msgs::ChannelFloat32 temp;
    geometry_msgs::Point32 temp2;
    temp.name = "intensity";

    for(unsigned int i = 0; i < mglobalPlan_points.size(); i++)
    {
        temp2.x = mglobalPlan_points[i].world_x;
        temp2.y = mglobalPlan_points[i].world_y;
        temp2.z = 0;
        plan_msg.points.push_back(temp2);
        temp.values.push_back(i);
    }
    plan_msg.channels.push_back(temp);
    mPlanPublisher.publish(plan_msg);
}
//最原始的方案是查找车身周围一定距离区域，找一个cost最小的点当作目标点 计算出方向送入下一级
//这里有个问题就是室内，这个距离0.75比较合适，室外1.5比较合适
//室内的合适时为了应对走廊狭窄的90度角，室外是为了不让车体左右抖动，实际上室内也存在左右抖动的问题。
//目标点的选取在operator再做优化，导航层只管发布可通行路径
void RobotNavigator::publishLocalPlan()
{
    unsigned int index = mLocalStartPoint;
    unsigned int indexInWorld;
    //std::vector< std::pair<double, double> > points;
    mLocalPlanPointsInWorldCell.clear();

    GridMapLocalToWorldId(index, indexInWorld);
    mLocalPlanPointsInWorldCell.push_back(indexInWorld);

    mlocalPlan_points.clear();
    // 从起始点到目标点之间寻找一条代价最小的路线
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
            mlocalPlan_points.push_back({
                                            ((x+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginX(),
                                            ((y+0.5) * mLocalCurrentMap.getResolution()) + mLocalCurrentMap.getOriginY(),
                                            0.0,
                                            index
                                        });

        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;

        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index])
                next_index = neighbors[i];
        }

        if(index == next_index) break;//这个条件，暂时想不出来有什么可能会落入
        index = next_index;

        GridMapLocalToWorldId(index, indexInWorld);
        mLocalPlanPointsInWorldCell.push_back(indexInWorld);
    }
    //这里是局部地图坐标下的点要转化到全局坐标进行发布

    sensor_msgs::PointCloud plan_msg;
    plan_msg.header.frame_id = mMapFrame.c_str();
    plan_msg.header.stamp = Time::now();

    sensor_msgs::ChannelFloat32 temp;
    geometry_msgs::Point32 temp2;
    temp.name = "intensity";

    double wx, wy;
    for(unsigned int i = 0; i < mlocalPlan_points.size(); i++)
    {
        GridMapLocalToWorldMeter(mlocalPlan_points[i].world_x, mlocalPlan_points[i].world_y, wx, wy);
        temp2.x = wx;
        temp2.y = wy;
        temp2.z = mDebug_show_height;
        plan_msg.points.push_back(temp2);
        temp.values.push_back(i);
    }

    plan_msg.channels.push_back(temp);
    mLocalPlanPublisher.publish(plan_msg);  // 发布局部规划的路径
}

//在先前的基础上 基于DJST计算出来的路径 从近处开始向远处查询，
//计算mLocalStartPoint到第i个路径点连成的直线，遍历这个直线上所有的栅格，如果大于某一个占用阈值，就认为失败否则就是可行点
bool RobotNavigator::RayTrackisClean(unsigned int endPointIndex)
{
    if(endPointIndex == mLocalStartPoint)
        return true;

    unsigned int startX = 0, startY = 0;
    unsigned int endX = 0, endY = 0;

    mLocalCurrentMap.getCoordinates(startX, startY, mLocalStartPoint);
    mLocalCurrentMap.getCoordinates(endX, endY, endPointIndex);

    int deltaX = abs(int(startX)-int(endX));
    int deltaY = abs(int(startY)-int(endY));
    //ROS_INFO("startX%d, startY%d endX%d, endY%d deltaX%d deltaY %d",startX, startY,endX, endY,deltaX,deltaY);
    //因为一开始排除了同一个点的情况 所以deltaX deltaY 不可能同时为0 ，所以除数不会为0
    if(deltaX > deltaY)
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaX;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaX;
        //ROS_INFO("stepX%f stepY%f", stepX, stepY);

        for(int i=1; i<= deltaX; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            //ROS_INFO("cellX %d  cellY %d, mCostMiddle%d cell%d startCell%d ", cellX, cellY, mCostMiddle, mLocalCurrentMap.getData(cellX, cellY), mLocalCurrentMap.getData(startX, startY));
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostMiddle)
            {
                return false;
            }
        }
        return true;
    }
    else
    {
        double stepX = (int(endX)-int(startX)) * mLocalCurrentMap.getResolution() / deltaY;
        double stepY = (int(endY)-int(startY)) * mLocalCurrentMap.getResolution() / deltaY;

        for(int i=1; i<= deltaY; i++)
        {
            double meterX = startX*mLocalCurrentMap.getResolution() + i*stepX;//为了统一格式。。虽然我知道这么写浪费计算资源
            double meterY = startY*mLocalCurrentMap.getResolution() + i*stepY;

            int cellX = meterX/mLocalCurrentMap.getResolution();
            int cellY = meterY/mLocalCurrentMap.getResolution();
            if(mLocalCurrentMap.getData(cellX, cellY) > mCostMiddle)
                return false;
        }
        return true;
    }
}

void RobotNavigator::publishDJGridMap()
{
    nav_msgs::OccupancyGrid tempMap;
    tempMap.data.resize(mLocalCurrentMap.getSize());
    tempMap.info.height = mLocalCurrentMap.getHeight();
    tempMap.info.width = mLocalCurrentMap.getWidth();
    tempMap.info.resolution = mLocalCurrentMap.getResolution();
    tempMap.info.origin.position.x = mLocalCurrentMap.getOriginX();
    tempMap.info.origin.position.y = mLocalCurrentMap.getOriginY();
    tempMap.info.origin.position.z = mDebug_show_height;//故意提高 看的清楚

    double max_value=0.0;
    for(int i=0; i<mLocalCurrentMap.getSize(); i++)
    {
        if(mLocalCurrentPlan[i] > max_value)
            max_value = mLocalCurrentPlan[i];
        //ROS_INFO("value = %f", mLocalCurrentPlan[i]);
    }
    //ROS_INFO("CurrentPlan max_value = %f", max_value);

    for(int id=0; id<mLocalCurrentMap.getSize(); id++)
    {
        tempMap.data[id] = (mLocalCurrentPlan[id]/max_value)*255;
    }
    mLocalDJGridMapPublisher.publish(tempMap);
}

//运动输出
//注意，这里是根据当前位置和代价地图，寻找周围节点的代价，找大梯度运动，并不是沿着路径规划走的。。。。无语了

/* 目标位置的服务程序，请求这个服务以后，在这个程序中会开始进行导航规划，直到达到目标点位置 */
void RobotNavigator::receiveMoveGoal(const csc_nav2d_navigator::MoveToPosition2DGoal::ConstPtr &goal)
{
    if(mStatus != NAV_ST_IDLE)
    {
        ROS_WARN("Navigator is busy!");
        mMoveActionServer->setAborted();
        return;
    }
    ROS_INFO("Received Goal: %.2f, %.2f (in frame '%s')", goal->target_pose.x, goal->target_pose.y, goal->header.frame_id.c_str());

    // Start navigating according to the generated plan
    Rate loopRate(FREQUENCY);
    unsigned int cycle = 0;
    bool reached = false;

    mGoalTheta = goal->target_pose.theta;  // 目标点的角度
    double targetDistance = (goal->target_distance > 0) ? goal->target_distance : mNavigationGoalDistance;  // 目标点的容忍距离
    double targetAngle = (goal->target_angle > 0) ? goal->target_angle : mNavigationGoalAngle;  // 目标点的容忍角度

    csc_nav2d_msgs::localPlanPoints planPoints_msg;  // 路径点的消息

    // 这里是做全局规划
    if(true)
    {
        WallTime startTime = WallTime::now();
        mStatus = NAV_ST_NAVIGATING;  // 导航中...

        // Create the plan for navigation
        mHasNewMap = false;
        // 从mapserver获取全局地图，并按照设定的规则对其进行膨胀操作
        if(!preparePlan())
        {
            ROS_ERROR("Prepare failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        //换算到grid坐标系
        int goalX =  (double)(goal->target_pose.x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
        int goalY =  (double)(goal->target_pose.y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
        if(goalX < 0) goalX = 0;
        if(goalX >= (int)mCurrentMap.getWidth()) goalX = mCurrentMap.getWidth() - 1;
        if(goalY < 0) goalY = 0;
        if(goalY >= (int)mCurrentMap.getHeight()) goalY = mCurrentMap.getHeight() - 1;

        bool success = false;
        if(mCurrentMap.getIndex(goalX, goalY, mGoalPoint))  //获得goal的id号，用于下一步规划
            success = createPlan();  // 进行全局路径规划，生成从起始点到目标点的最短路径

        if(!success)
        {
            if(correctGoalPose())//如果没有规划成功，就矫正目标点的位置，重新规划，实际不会出现这个问题，人手点地图时，选择可以规划到的空白区域！
                success = createPlan();
        }

        if(!success)
        {
            ROS_ERROR("Planning failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        // 之后再是基于这张costmap进行路径规划
        publishPlan();

        WallTime endTime = WallTime::now();
        WallDuration d = endTime - startTime;
        ROS_INFO("Path planning took %.09f seconds, distance is %.2f m.", d.toSec(), mCurrentPlan[mStartPoint]);
    }

    // 局部路径规划(10Hz)
    while(true)
    {
        planPoints_msg.navigation_state = 0;  // 0表示正常路径规划

        // 获得在全局地图中的位置，更新了mStartPoint值，即可用于计算此时到目标点的距离
        if(!setCurrentPosition())
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        // 构建局部地图，以机器人为中心1.1倍golbalCostMap的大小，中间叠加了带障碍物信息的局部costmap
        if(!createLocalMap())
        {
            ROS_ERROR("Prepare LocalMap failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        // 计算局部路径规划的起始点（当前在全局地图中的位置）在局部地图中的位置
        if(!setLocalCurrentPosition())
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        //基于全局路径找到局部地图里最边缘上的局部goal，
        //因为只叠加了1倍区域的costmap，外部0.5倍的环带区域是一定存在全局路径的。
        //为了防止局部地图中存在绕行的全局路线，在搜索时是从机器人当前位置搜索。
        if(!createLocalGoal())
        {
            ROS_ERROR("Prepare LocalGoal failed!");
            mMoveActionServer->setAborted();
            stop();
            return;
        }

        WallTime startTime = WallTime::now();
        // 执行局部路径规划
        if(!createLocalPlan())
        {
            ROS_WARN("Prepare LocalPlan failed!");
            //publishDJGridMap();
            mLocalPlanMissCounter = 5;
            mIsNavPaused = true;

            csc_nav2d_operator::cmd stopMsg;
            stopMsg.Turn = 0;
            stopMsg.Velocity = 0;
            mCommandPublisher.publish(stopMsg);
        }
        else
        {
            if(mLocalPlanMissCounter > 0)
            {
                mLocalPlanMissCounter--;
                ROS_INFO("mLocalPlanMissCounter=%d", mLocalPlanMissCounter);
            }
            else
            {
                mIsNavPaused = false;
            }
            //publishDJGridMap();  // 规划路径时的代价图，只为调试，不发布
            publishLocalPlan();  // 这一步必须执行。里面有保存当前帧的最短路径，用于优化下一次路径规划
        }

        WallDuration d = WallTime::now() - startTime;
        //ROS_INFO("Path planning took %.2f ms", d.toSec()*1000);

        // Check if we are asked to preempt
        if(!ok() || mMoveActionServer->isPreemptRequested() || mIsStopped)
        {
            ROS_INFO("Navigation has been preempted externally.");
            mMoveActionServer->setPreempted();  //强制中断服务
            mIsStopped = false;
            stop();
            return;
        }

        // Where are we now
        mHasNewMap = false;
        if(!setCurrentPosition())  //获得在全局地图中的位置
        {
            ROS_ERROR("Navigation failed, could not get current position.");
            mMoveActionServer->setAborted();
            stop();
        }

        // Are we already close enough?
        if(!reached && mCurrentPlan[mStartPoint] <= mNavigationGoalDistance && mCurrentPlan[mStartPoint] >= 0
           && fabs(mGoalTheta - mCurrentDirection) < mNavigationGoalAngle)
        {
            ROS_INFO("Reached target.");
            reached = true;
        }

        // reach以后开始调节角度
        if(reached)
        {
            ROS_INFO("auto run complete!");

            // Are we also headed correctly?
            double deltaTheta = mCurrentDirection - goal->target_pose.theta;
            while(deltaTheta < -PI) deltaTheta += 2*PI;
            while(deltaTheta >  PI) deltaTheta -= 2*PI;

            double diff = (deltaTheta > 0) ? deltaTheta : -deltaTheta;
            //ROS_INFO_THROTTLE(1.0,"Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
            if(diff <= targetAngle)
            {
                ROS_INFO("Final Heading: %.2f / Desired: %.2f / Difference: %.2f / Tolerance: %.2f", mCurrentDirection, goal->target_pose.theta, diff, targetAngle);
                break;
            }

            csc_nav2d_operator::cmd msg;
            if(deltaTheta > 0)
            {
                msg.Turn = 1;
                msg.Velocity = deltaTheta;
            }else
            {
                msg.Turn = -1;
                msg.Velocity = -deltaTheta;
            }
            if(msg.Velocity > 1) msg.Velocity = 1;
            msg.Mode = 1;//mode为1时是靠近goal周围才会被置位的，一般导航情况下，mode是0
            //operator中，对于mode为1的，是直接赋值，对于0的会基于局部地图，修正方向
            //当turn为1或者-1时，vel的值就是原地自转的速度，在operator中有做判断
            mCommandPublisher.publish(msg);
        }
        else
        {
            generateLocalCommand();
        }

        // Publish feedback via ActionServer
        if(cycle%10 == 0)
        {
            csc_nav2d_navigator::MoveToPosition2DFeedback fb;
            fb.distance = mCurrentPlan[mStartPoint];
            mMoveActionServer->publishFeedback(fb);
        }

        // Sleep remaining time
        cycle++;
        spinOnce();
        loopRate.sleep();

        if(cycle%10 == 0)
            ROS_INFO("Local plan took %.3f s", loopRate.cycleTime().toSec());
        //if(loopRate.cycleTime() > ros::Duration(1.0 / FREQUENCY))
        //    ROS_WARN("Missed desired rate of %.2fHz! Loop actually took %.4f seconds!",FREQUENCY, loopRate.cycleTime().toSec());
    }

    // Set ActionServer suceeded
    ROS_INFO("Goal reached.");
    csc_nav2d_navigator::MoveToPosition2DResult r;
    r.final_pose.x = mCurrentPositionX;
    r.final_pose.y = mCurrentPositionY;
    r.final_pose.theta = mCurrentDirection;
    r.final_distance = mCurrentPlan[mStartPoint];
    mMoveActionServer->setSucceeded(r);
    stop();
}

bool RobotNavigator::receiveStop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    mIsStopped = true;
    res.success = true;
    res.message = "Navigator received stop signal.";
    return true;
}

bool RobotNavigator::receivePause(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if(mIsPaused)
    {
        mIsPaused = false;
        res.success = false;
        res.message = "Navigator continues.";
    }
    else
    {
        mIsPaused = true;
        res.success = true;
        res.message = "Navigator pauses.";

        csc_nav2d_operator::cmd stopMsg;
        stopMsg.Turn = 0;
        stopMsg.Velocity = 0;
        mCommandPublisher.publish(stopMsg);
    }
    return true;
}

bool RobotNavigator::isLocalized()
{
    return mTfListener.waitForTransform(mMapFrame, mRobotFrame, Time::now(), Duration(0.1));
}

/* 获取机器人当前全局位置 */
bool RobotNavigator::setCurrentPosition()
{
    // 获取机器人到地图坐标系的tf
    StampedTransform transform;
    try
    {
        mTfListener.lookupTransform(mMapFrame, mRobotFrame, Time(0), transform);
    }
    catch(TransformException ex)
    {
        ROS_ERROR("Could not get robot position: %s", ex.what());
        return false;
    }

    double world_x = transform.getOrigin().x();
    double world_y = transform.getOrigin().y();
    double world_theta = tf::getYaw(transform.getRotation());

    // 计算出机器人在栅格地图中的像素坐标(相对地图的原点)
    unsigned int current_x = (world_x - mCurrentMap.getOriginX()) / mCurrentMap.getResolution();
    unsigned int current_y = (world_y - mCurrentMap.getOriginY()) / mCurrentMap.getResolution();
    unsigned int i;

    if(!mCurrentMap.getIndex(current_x, current_y, i))
    {
        if(mHasNewMap || !getMap() || !mCurrentMap.getIndex(current_x, current_y, i))
        {
            ROS_ERROR("Is the robot out of the map?");
            return false;
        }
    }
    mStartPoint = i;
    mCurrentDirection = world_theta;
    mCurrentPositionX = world_x;
    mCurrentPositionY = world_y;

    return true;
}

/* 计算局部路径规划的起始点在局部地图中的位置 */
bool RobotNavigator::setLocalCurrentPosition()
{
    unsigned int current_x;
    unsigned int current_y;
    unsigned int current_local_x;
    unsigned int current_local_y;
    unsigned int i;

    mCurrentMap.getCoordinates(current_x, current_y, mStartPoint);

    // 由全局地图中的位置坐标转换到局部地图中的位置坐标
    GridMapWorldToLocalCell(current_x, current_y, current_local_x, current_local_y);

    if(!mLocalCurrentMap.getIndex(current_local_x, current_local_y, i))
    {
        ROS_ERROR("Is the robot out of the LocalMap?wx=%d wy=%d lx=%d ly=%d",current_x, current_y,current_local_x, current_local_y);
        return false;
    }
    mLocalStartPoint = i;
    return true;
}

void RobotNavigator::stop()
{
	csc_nav2d_operator::cmd stopMsg;
	stopMsg.Turn = 0;
	stopMsg.Velocity = 0;
	mCommandPublisher.publish(stopMsg);

    mStatus = NAV_ST_IDLE;
    mIsPaused = false;
    mIsStopped = false;
    mIsPaused = false;
}

//运动输出
//注意，这里是根据当前位置和代价地图，寻找周围节点的代价，找大梯度运动，并不是沿着路径规划走的。。。。无语了
bool RobotNavigator::generateLocalCommand()
{
    // Do nothing when paused
    if(mIsPaused || mIsNavPaused)
    {
        ROS_INFO_THROTTLE(1.0, "Navigator is paused and will not move now.");
        return true;
    }

    if(mStatus != NAV_ST_NAVIGATING )
    {
        ROS_WARN_THROTTLE(1.0, "Navigator has status %d when generateCommand() was called!", mStatus);
        return false;
    }

    // Generate direction command from plan
    unsigned int current_x = 0, current_y = 0;
    if(!mLocalCurrentMap.getCoordinates(current_x, current_y, mLocalStartPoint)) // || !mCurrentMap.isFree(mStartPoint))
    {
        ROS_ERROR("Plan execution failed, robot not in map!");
        return false;
    }

    unsigned int targetInSize = mLocalStartPoint;
    int steps = 0.75 / mLocalCurrentMap.getResolution();//看0.75m范围内的目标点，室内0.75比较好，狭窄环境能通过
    for(int i = 0; i < steps; i++)
    {
        unsigned int bestPoint = targetInSize;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(targetInSize);
        for(unsigned int i = 0; i < neighbors.size(); i++)
        {
            if(mLocalCurrentPlan[neighbors[i]] >= (unsigned int)0 && \
               mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[bestPoint])
                bestPoint = neighbors[i];
        }
        targetInSize = bestPoint;
    }

    //raytrack优化局部目标
    unsigned int index = mLocalStartPoint;

    std::vector<unsigned int> pointsId;
    while(true)
    {
        unsigned int x = 0, y = 0;
        if(mLocalCurrentMap.getCoordinates(x,y,index))
        {
            pointsId.push_back(index);
        }
        if(mLocalCurrentPlan[index] == 0) break;//迭代到了goal

        unsigned int next_index = index;
        std::vector<unsigned int> neighbors = mLocalCurrentMap.getFreeNeighbors(index);//获得周围8个领域
        for(unsigned int i = 0; i < neighbors.size(); i++)//查找8个领域里距离最小的值，也就是最大的梯度方向
        {
            if(mLocalCurrentPlan[neighbors[i]] >= 0 && mLocalCurrentPlan[neighbors[i]] < mLocalCurrentPlan[next_index])
                next_index = neighbors[i];
        }
        if(index == next_index) break;
        index = next_index;
    }

    unsigned int target = mLocalStartPoint;
    if(pointsId.size() > 1)//raytrack有效时
    {
        int i;
        for(i=1; i<pointsId.size(); i++)
        {
            if(RayTrackisClean(pointsId[i]))
            {
                target = pointsId[i];
            }
            else
                break;
        }
        //之后判断 是否在0.75距离以内，0.75是室内运动的一个比较好的前瞻值
        //另外由于室内狭窄 有时候points[1]的值就有可能比mCostMiddle大
        if(mLocalCurrentPlan[target] > mLocalCurrentPlan[targetInSize])
        {
            target = targetInSize;//raytrack 追踪的节点没有0.75范围内找到的更加靠近goal
            ROS_INFO("raytrackGoal == localTargetGoal!");
        }
    }
    else
    {
        target = targetInSize;
    }

    // Head towards (x,y)
    unsigned int x = 0, y = 0;
    if(!mLocalCurrentMap.getCoordinates(x, y, target))
    {
        ROS_ERROR("Plan execution failed, target pose not in map!");
        return false;
    }
    double map_angle = atan2((double)y - current_y, (double)x - current_x);

    double angle = map_angle - mCurrentDirection;
    if(angle < -PI) angle += 2*PI;
    if(angle > PI) angle -= 2*PI;

    // Create the command message
    csc_nav2d_operator::cmd msg;
    //msg.Turn = -4.0 * angle / PI;//当目标角度和当前角度偏差大于pi/4 就执行原地自转
    msg.Turn = -(180.0/50) * angle / PI;//当目标角度和当前角度偏差大于pi/4 就执行原地自转
    if(msg.Turn < -1) msg.Turn = -1;//原地转向的
    if(msg.Turn >  1) msg.Turn = 1;

    if(mLocalCurrentPlan[mLocalStartPoint] > mNavigationHomingDistance)
        msg.Mode = 2;//0，使用局部避障，因为现在全局规划包含了局部障碍物的局部路径 因此直接送入控制。
    else
        msg.Mode = 1;

    if(mLocalCurrentPlan[mLocalStartPoint] > 1.0 || mLocalCurrentPlan[mLocalStartPoint] < 0)
    {
        msg.Velocity = 0.8;//1.0
        if(msg.Turn == -1 || msg.Turn==1)
        {
            msg.Velocity = msg.Velocity/2;//无缝切换理论上是msg.Velocity，实际上发现在室外摩擦力较高的地方，这个自转速度太高，车子震动明显。
            //当原地自转时，operator会将msg.Velocity设置为control_z的速度。又由于在motor_control不是线性叠加的給到轮胎的速度，speed = z/2+x，因此这里直接给msg.Velocity
            //可以在切换时，某个轮胎速度不产生
            ROS_INFO("msg.Turn=%f !!",msg.Turn);
        }
    }else//在距离目标1m左右开始变慢
    {
        msg.Velocity = 0.8;//1.0
        if(msg.Turn == -1 || msg.Turn==1)
        {
            msg.Velocity = msg.Velocity/2;
            ROS_INFO("msg.Turn=%f !!",msg.Turn);
        }

        /*
    msg.Velocity = 0.5 + (mLocalCurrentPlan[mLocalStartPoint] / 2.0);
    if(msg.Turn == -1 || msg.Turn==1)
    {
      msg.Velocity = 0.4;
    }
    */
    }
    mCommandPublisher.publish(msg);

    //调试使用 发布局部方向点的marker
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = mLocalCurrentMap.getOriginX() + (((double)x+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.y = mLocalCurrentMap.getOriginY() + (((double)y+0.5) * mLocalCurrentMap.getResolution());
    marker.pose.position.z = mDebug_show_height;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = mCurrentMap.getResolution() * 1.0;
    marker.scale.y = mCurrentMap.getResolution() * 1.0;

    if(target == targetInSize)
        marker.scale.z = 3.0;
    else
        marker.scale.z = 6.0;//使用raytrack 用更长的mark表示
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    mLocalDirectionMarkerPublisher.publish(marker);

    return true;
}
