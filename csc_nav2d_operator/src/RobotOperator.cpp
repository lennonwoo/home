#include <nav_msgs/GridCells.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <csc_nav2d_operator/RobotOperator.h>

#define PI 3.14159265

RobotOperator::RobotOperator()
{
    // Create the local costmap
    //注意这个名字不是随便取的！必须和launch文件中对应的命名空间一致
    mLocalMap = new costmap_2d::Costmap2DROS("local_costmap", mTfListener);//这里相当于开启了一个costmap的node
    //mLocalMap = new costmap_2d::Costmap2DROS("local_map", mTfListener);//离线仿真用
    mRasterSize = mLocalMap->getCostmap()->getResolution();

    std::cout << "getOriginX, Y " << mLocalMap->getCostmap()->getOriginX() << "  "<< mLocalMap->getCostmap()->getOriginY()<<std::endl
              << "getSizeInCellsX, Y " <<  mLocalMap->getCostmap()->getSizeInCellsX() <<  "  "<< mLocalMap->getCostmap()->getSizeInCellsY()<<std::endl
              << "getSizeInMetersX, Y " <<  mLocalMap->getCostmap()->getSizeInMetersX()<<  "  "<<  mLocalMap->getCostmap()->getSizeInMetersY() << std::endl;
    std::cout << "getGlobalFrameID getBaseFrameID " << mLocalMap->getGlobalFrameID() << "  " << mLocalMap->getBaseFrameID() << std::endl;

    // Publish / subscribe to ROS topics
    ros::NodeHandle robotNode;
    robotNode.param("robot_frame", mRobotFrame, std::string("robot"));
    robotNode.param("odometry_frame", mOdometryFrame, std::string("odometry_base"));
    mCommandSubscriber = robotNode.subscribe(COMMAND_TOPIC, 1, &RobotOperator::receiveCommand, this);
    mControlPublisher = robotNode.advertise<geometry_msgs::Twist>(CONTROL_TOPIC, 1);
    mControlStopPublisher = robotNode.advertise<std_msgs::Float32>("/stop",1);
    mCostPublisher = robotNode.advertise<geometry_msgs::Vector3>("costs", 1);

    // Get parameters from the parameter server
    ros::NodeHandle operatorNode("~/");
    operatorNode.param("publish_route", mPublishRoute, false);
    if(mPublishRoute)
    {
        ROS_INFO("Will publish desired direction on '%s' and control direction on '%s'.", ROUTE_TOPIC, PLAN_TOPIC);
        mTrajectoryPublisher = operatorNode.advertise<sensor_msgs::PointCloud>(ROUTE_TOPIC, 5);
        mPlanPublisher = operatorNode.advertise<sensor_msgs::PointCloud>(PLAN_TOPIC, 5);
    }
    operatorNode.param("max_free_space", mMaxFreeSpace, 5.0);//5米
    operatorNode.param("safety_decay", mSafetyDecay, 0.95);
    operatorNode.param("distance_weight", mDistanceWeight, 1);
    operatorNode.param("safety_weight", mSafetyWeight, 1);
    operatorNode.param("conformance_weight", mConformanceWeight, 1);
    operatorNode.param("continue_weight", mContinueWeight, 1);
    operatorNode.param("max_velocity", mMaxVelocity, 1.0);

    // Apply tf_prefix to all used frame-id's
    mOdometryFrame = "map";
    mRobotFrame = mTfListener.resolve(mRobotFrame);
    mOdometryFrame = mTfListener.resolve(mOdometryFrame);

    // Initialize the lookup table for the driving directions
    ROS_INFO("Initializing LUT...");
    initTrajTable();
    ROS_INFO("...done!");

    // Set internal parameters
    mDesiredDirection = 0;
    mDesiredVelocity = 0;
    mCurrentDirection = 0;
    mCurrentVelocity = 0;
    mDriveMode = 0;
    mRecoverySteps = 0;
}

RobotOperator::~RobotOperator()
{
	for(int i = 0; i < LUT_RESOLUTION; i++)//为什么不是(LUT_RESOLUTION * 4) + 2 ？
	{
		delete mTrajTable[i];
	}
}

//这里是构建常数表，用来预测轨迹，按当前给的转向角度进行预测
//建立了向前左，前右，后左，后右四张表，每张表对180度进行了100个划分，下次可以直接查表来获得运动轨迹。
void RobotOperator::initTrajTable()
{
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		mTrajTable[i] = NULL;
	}
	for(int i = 1; i < LUT_RESOLUTION; i++)//99个
	{
		double tw = -PI * i / LUT_RESOLUTION;
		double tx = cos(tw) + 1;
		double ty = -sin(tw);
		double tr = ((tx*tx)+(ty*ty))/(ty+ty);
		std::vector<geometry_msgs::Point32> points;
		double alpha = 0;
		while(alpha < PI)
		{
			double x = tr * sin(alpha);
			double y = tr * (1.0 - cos(alpha));
			geometry_msgs::Point32 p;
			p.x = x;
			p.y = y;
			p.z = 0;
			points.push_back(p);
			alpha += mRasterSize / tr;//mRasterSize是地图的分辨率，这里假设是0.05m
		}
		// Add the PointCloud to the LUT
		// Circle in forward-left direction
		sensor_msgs::PointCloud* flcloud = new sensor_msgs::PointCloud();
		flcloud->header.stamp = ros::Time(0);
		flcloud->header.frame_id = mRobotFrame;
		flcloud->points.resize(points.size());
		
		// Circle in forward-right direction
		sensor_msgs::PointCloud* frcloud = new sensor_msgs::PointCloud();
		frcloud->header.stamp = ros::Time(0);
		frcloud->header.frame_id = mRobotFrame;
		frcloud->points.resize(points.size());
		
		// Circle in backward-left direction
		sensor_msgs::PointCloud* blcloud = new sensor_msgs::PointCloud();
		blcloud->header.stamp = ros::Time(0);
		blcloud->header.frame_id = mRobotFrame;
		blcloud->points.resize(points.size());
		
		// Circle in backward-right direction
		sensor_msgs::PointCloud* brcloud = new sensor_msgs::PointCloud();
		brcloud->header.stamp = ros::Time(0);
		brcloud->header.frame_id = mRobotFrame;
		brcloud->points.resize(points.size());
		
		for(unsigned int j = 0; j < points.size(); j++)
		{
			flcloud->points[j] = points[j];
			frcloud->points[j] = points[j];
			blcloud->points[j] = points[j];
			brcloud->points[j] = points[j];
			
			frcloud->points[j].y *= -1;
			blcloud->points[j].x *= -1;
			brcloud->points[j].x *= -1;
			brcloud->points[j].y *= -1;
		}
		mTrajTable[LUT_RESOLUTION - i] = flcloud;
		mTrajTable[LUT_RESOLUTION + i] = frcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) - i] = blcloud;
		mTrajTable[(3 * LUT_RESOLUTION + 1) + i] = brcloud;//这里会访问到400=301+99
	}
	//i的范围是1-99
	//至此mTrajTable的0 100 200 201 301 401
	
	// Add First and Last LUT-element
	geometry_msgs::Point32 p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	
	sensor_msgs::PointCloud* turn = new sensor_msgs::PointCloud();
	turn->header.stamp = ros::Time(0);
	turn->header.frame_id = mRobotFrame;
	turn->points.resize(1);
	turn->points[0] = p;
	
	int straight_len = 5.0 / mRasterSize;
	
	sensor_msgs::PointCloud* fscloud = new sensor_msgs::PointCloud();
	fscloud->header.stamp = ros::Time(0);
	fscloud->header.frame_id = mRobotFrame;
	fscloud->points.resize(straight_len);
	
	sensor_msgs::PointCloud* bscloud = new sensor_msgs::PointCloud();
	bscloud->header.stamp = ros::Time(0);
	bscloud->header.frame_id = mRobotFrame;
	bscloud->points.resize(straight_len);
	
	for(int i = 0; i < straight_len; i++)
	{
		fscloud->points[i] = p;
		bscloud->points[i] = p;
		bscloud->points[i].x *= -1;
		p.x += mRasterSize;
	}
	
	mTrajTable[LUT_RESOLUTION] = fscloud;//100
	mTrajTable[LUT_RESOLUTION*3 + 1] = bscloud;//301
	
	mTrajTable[0] = turn;//0
	mTrajTable[LUT_RESOLUTION*2] = turn;//200
	mTrajTable[LUT_RESOLUTION*2 + 1] = turn;//201
	mTrajTable[LUT_RESOLUTION*4 + 1] = turn;//401
	
	for(int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
	{
		if(!mTrajTable[i])
		{
			ROS_ERROR("Table entry %d has not been initialized!", i);
		}
	}
}

/* 由remote节点或者navigator节点发出的命令回调函数 */
void RobotOperator::receiveCommand(const csc_nav2d_operator::cmd::ConstPtr& msg)
{
	if(msg->Turn < -1 || msg->Turn > 1)
	{
		// The given direction is invalid.
		// Something is going wrong, so better stop the robot:
		mDesiredDirection = 0;
		mDesiredVelocity = 0;
		mCurrentDirection = 0;
		mCurrentVelocity = 0;
		ROS_ERROR("Invalid turn direction on topic '%s'!", COMMAND_TOPIC);
		return;
	}
	mDesiredDirection = msg->Turn;
	mDesiredVelocity = msg->Velocity * mMaxVelocity;
	mDriveMode = msg->Mode;
}

/* 在main函数的while循环中运行，10Hz */
void RobotOperator::executeCommand()
{
	// 1. Get a copy of the costmap to work on.

	mCostmap = mLocalMap->getCostmap();
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(mCostmap->getMutex()));
	double bestDirection, d;
	
	// 2. Set velocity and direction depending on drive mode
	switch(mDriveMode)
	{
	case 0:
		bestDirection = findBestDirection();//自动导航的bug，出在这里
		//ROS_INFO("bestDirection=%f mDesiredVelocity=%f ", bestDirection, mDesiredVelocity);
		//ROS_INFO("mCurrentDirection=%f mDesiredDirection=%f ", mCurrentDirection, mDesiredDirection);
		d = bestDirection - mCurrentDirection;
		if(d < -0.2) d = -0.2;
		if(d > 0.2) d = 0.2;
		mCurrentDirection += d;
		mCurrentVelocity = mDesiredVelocity;
		break;
	case 1:
		mCurrentDirection = mDesiredDirection;
		mCurrentVelocity = mDesiredVelocity;
		break;
	case 2://xjh_add
		if(fabs(mDesiredDirection) == 1)//原地自转属于紧急情况，一般不会需要的，因此直接跳过慢慢的累加
			mCurrentDirection = mDesiredDirection;
		else
		{
			d = mDesiredDirection - mCurrentDirection;
			if(d < -0.1) d = -0.1;
			if(d > 0.1) d = 0.1;
			mCurrentDirection += d;
		}
		//不用对速度进行滤波 因为在后面会根据方向动态的调节速度，只要方向不突变，速度就不会突变！
		mCurrentVelocity = mDesiredVelocity;
		break;

	default:
		ROS_ERROR("Invalid drive mode!");
		mCurrentVelocity = 0.0;
	}
	
	// Create some Debug-Info
	//evaluateAction(mCurrentDirection, mCurrentVelocity, true);
	
	sensor_msgs::PointCloud* originalCloud = getPointCloud(mCurrentDirection, mDesiredVelocity);
	sensor_msgs::PointCloud transformedCloud;

	try
	{
		mTfListener.transformPointCloud(mOdometryFrame,*originalCloud,transformedCloud);
	}
	catch(tf::TransformException ex)
	{
		ROS_ERROR("%s", ex.what());
		return;
	}
	
	//上一步获得了最好的轨迹，然后基于该轨迹，重新修改速度
	//这里和之前计算轨迹一样，存在一个问题，实际车体是角速度和线速度的叠加，如果修改了速度，轨迹也会变化
	//但是既然前面用LUT的方法都无视速度的影响，这里也不会考虑这个问题
	//实际使用是可以的
	// Determine maximum linear velocity
	int freeCells = calculateFreeSpace(&transformedCloud);  // 机器人路径上的自由栅格数
	double freeSpace = mRasterSize * freeCells;
	double safeVelocity = (freeSpace / mMaxFreeSpace) + 0.05;  // 自由栅格数越小，机器人的安全速度越低

	if(freeCells == transformedCloud.points.size() && safeVelocity < 0.5)
		safeVelocity = 0.5;

	if(freeSpace < 0.3 && freeCells < transformedCloud.points.size())
		safeVelocity = 0;

	if(safeVelocity > mMaxVelocity)
		safeVelocity = mMaxVelocity;

	// Check whether the robot is stuck
	if(mRecoverySteps > 0) mRecoverySteps--;
	//当车子运动性能不佳，遇到障碍物希望转弯又无法转过去时，会渐渐靠近障碍物，这个条件就会进入，导致车子“卡死”
	//0.1改0.2
	if(safeVelocity < 0.1)
	{
		if(mDriveMode == 0 || mDriveMode == 2)
		{
			mRecoverySteps = 30; // Recover for 3 seconds
			ROS_WARN_THROTTLE(1, "Robot is stuck! Trying to recover...");
		}
		else
		{
			//mCurrentVelocity = 0;
			ROS_WARN_THROTTLE(1, "Robot cannot move further in this direction!");
		}
	}

	// Publish route via ROS (mainly for debugging)
	if(mPublishRoute)
	{
		{
			sensor_msgs::PointCloud plan_msg;
			plan_msg.header.frame_id = "map";
			plan_msg.header.stamp = ros::Time::now();

			sensor_msgs::ChannelFloat32 temp;
			geometry_msgs::Point32 temp2;
			temp.name = "intensity";

			for(int i = 0; i < freeCells; i++)
			{
				temp2.x = transformedCloud.points[i].x;
				temp2.y = transformedCloud.points[i].y;
				temp2.z = transformedCloud.points[i].z;
				plan_msg.points.push_back(temp2);
				temp.values.push_back(0);
			}
			plan_msg.channels.push_back(temp);
			mTrajectoryPublisher.publish(plan_msg);  // 发布当前的运动路径
		}

		// Publish plan via ROS (mainly for debugging)
		sensor_msgs::PointCloud* originalPlanCloud = getPointCloud(mDesiredDirection, mDesiredVelocity);
		sensor_msgs::PointCloud transformedPlanCloud;

		try
		{
			mTfListener.transformPointCloud(mOdometryFrame,*originalPlanCloud,transformedPlanCloud);
		}
		catch(tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}
		{
			sensor_msgs::PointCloud plan_msg;
			plan_msg.header.frame_id = "map";
			plan_msg.header.stamp = ros::Time::now();

			sensor_msgs::ChannelFloat32 temp;
			geometry_msgs::Point32 temp2;
			temp.name = "intensity";

			int freeSpacePlan = calculateFreeSpace(&transformedPlanCloud);
			for(int i = 0; i < freeSpacePlan; i++)
			{
				temp2.x = transformedPlanCloud.points[i].x;
				temp2.y = transformedPlanCloud.points[i].y;
				temp2.z = transformedPlanCloud.points[i].z;
				plan_msg.points.push_back(temp2);
				temp.values.push_back(i);
			}
			plan_msg.channels.push_back(temp);
			mPlanPublisher.publish(plan_msg);  //遥控 或者navigator給的期望路径 蓝色
		}
	}
	// Publish result via Twist-Message
	geometry_msgs::Twist controlMsg;

	double velocity = mCurrentVelocity;
	if(mCurrentDirection == 0)
	{
		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}
		else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = 0;
	}
	else if(mCurrentDirection == -1 || mCurrentDirection == 1)//当方向给到最大时，只进行自转
	{
		controlMsg.linear.x = 0;
		controlMsg.angular.z = -1.0 * mCurrentDirection * velocity;
	}
	else
	{
		double x = sin(mCurrentDirection * PI);
		double y = (cos(mCurrentDirection * PI) + 1);
		double r = ((x*x) + (y*y)) / (2*x);
		double abs_r = (r > 0) ? r : -r;

		velocity /= (1 + (1.0/abs_r));

		if(velocity > safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safeVelocity);
			velocity = safeVelocity;
		}
		else if(velocity < -safeVelocity)
		{
			ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safeVelocity);
			velocity = -safeVelocity;
		}
		
		controlMsg.linear.x = velocity;
		controlMsg.angular.z = -1.0 / r * controlMsg.linear.x;
	}

    std_msgs::Float32 stop;
    stop.data = 0;
    mControlStopPublisher.publish(stop);  //not stop

    mControlPublisher.publish(controlMsg);
}

/* 统计机器人当前路径上有多少个自由的栅格(不与障碍物发生碰撞) */
int RobotOperator::calculateFreeSpace(sensor_msgs::PointCloud* cloud)
{	
	unsigned int mx, my;
	int length = cloud->points.size();
	int freeSpace = 0;
	for(int i = 0; i < length; i++)
	{
		if(mCostmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my))
		{
			if(mCostmap->getCost(mx,my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
			{
				freeSpace++;//物理尺度不
			}else
			{
				break;
			}
		}else
		{
			break;
		}
	}

	return freeSpace;
}

double RobotOperator::evaluateAction(double direction, double velocity, bool debug,double &debug_Conformance,double &debug_Continue, double &debug_Distance,double  &debug_Safety)
{
    sensor_msgs::PointCloud* originalCloud = getPointCloud(direction, velocity);
    sensor_msgs::PointCloud transformedCloud;//因为激光雷达的costmap是基于tf获得的，所以是全局map坐标系，因此LUT表也要转化成map坐标系下
    try
    {
        mTfListener.transformPointCloud(mOdometryFrame, *originalCloud,transformedCloud);
    }
    catch(tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return 1;
    }

    double valueDistance = 0.0;    // How far can the robot move in this direction?
    double valueSafety = 0.0;      // How safe is it to move in that direction?
    double valueConformance = 0.0; // How conform is it with the desired direction?

    double freeSpace = 0.0;
    double decay = 1.0;
    unsigned char cell_cost;
    double safety;

    // Calculate safety value
    int length = transformedCloud.points.size();
    bool gettingBetter = true;
    //这个距离的写法有一个重大的bug！！！
    //每一个LUT路径的size是不同的，尤其是想直走时，会有非常多的点，python模拟过了，
    //这就导致在后面计算valueDistance的比利时不具有任何参考意义！
    for(int i = 0; i < length; i++)
    {
        unsigned int mx, my;
        if(mCostmap->worldToMap(transformedCloud.points[i].x, transformedCloud.points[i].y, mx, my))
        {
            cell_cost = mCostmap->getCost(mx,my);
            if(cell_cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                // Trajectory hit an obstacle
                break;
            }
        }
        freeSpace += mRasterSize;//0.05分辨率，这个参数具有物理意义

        safety = costmap_2d::INSCRIBED_INFLATED_OBSTACLE - (cell_cost * decay);
        if(gettingBetter)
        {
            if(safety >= valueSafety) valueSafety = safety;
            else gettingBetter = false;
        }else
        {
            if(safety < valueSafety) valueSafety = safety;
        }
        decay *= mSafetyDecay;
    }

    double action_value = 0.0;
    double normFactor = 0.0;
    valueSafety /= costmap_2d::INSCRIBED_INFLATED_OBSTACLE;

    // Calculate distance value
    if(freeSpace >= mMaxFreeSpace)
    {
        freeSpace = mMaxFreeSpace;
    }
    valueDistance = freeSpace / std::min(mMaxFreeSpace, length*mRasterSize);
    normFactor = mDistanceWeight + mSafetyWeight;

    if(mRecoverySteps == 0)
    {
        // Calculate continuety value
        double valueContinue = mCurrentDirection - direction;
        if(valueContinue < 0) valueContinue *= -1;
        valueContinue = 1.0 / (1.0 + exp(pow(valueContinue-0.5,15)));

        // Calculate conformance value
        double desired_sq = (mDesiredDirection > 0) ? mDesiredDirection * mDesiredDirection : mDesiredDirection * -mDesiredDirection;
        double evaluated_sq = (direction > 0) ? direction * direction : direction * -direction;
        valueConformance = cos(PI / 2.0 * (desired_sq - evaluated_sq)); // cos(-PI/2 ... +PI/2) --> [0 .. 1 .. 0]
        //期望值和当前值越方差越小 分数越高
        //valueConformance，参数给的不好会一直在震荡。。。比如开机时期望方向什么都不給。
        //按默认的参数，程序启动时会看到这个小圈从0变到了向正前方，就这这里的原因。
        action_value += valueConformance * mConformanceWeight;
        action_value += valueContinue * mContinueWeight;
        normFactor += mConformanceWeight + mContinueWeight;

        debug_Conformance = valueConformance* mConformanceWeight;
        debug_Continue = valueContinue* mContinueWeight;
        debug_Distance = valueDistance* mDistanceWeight;
        debug_Safety = valueSafety* mSafetyWeight;
    }

    action_value += valueDistance * mDistanceWeight;
    action_value += valueSafety * mSafetyWeight;
    action_value /=  normFactor;

    if(debug)
    {
        geometry_msgs::Vector3 cost_msg;
        cost_msg.x = valueDistance;
        cost_msg.y = valueSafety;
        //		cost_msg.y = valueContinue;
        cost_msg.z = valueConformance;
        mCostPublisher.publish(cost_msg);
    }

    return action_value;
}

double diff(double v1, double v2)
{
	if(v1 > v2)
		return v1 - v2;
	else
		return v2 - v1;
}

double RobotOperator::findBestDirection()
{
	double best_dir = -1.0;
	double best_value = 0.0;
	double step = 0.01;
	double dir = -1.0;
	double debug_Conformance;
	double debug_Continue;
	double debug_Distance;
	double debug_Safety;
	
	while(dir <= 1.0)
	{
		double a,b,c,d;
		double value = evaluateAction(dir, mDesiredVelocity,false, a,b,c,d);
		if(value > best_value)
		{
			best_dir = dir;
			best_value = value;
			debug_Conformance=a;
			debug_Continue=b;
			debug_Distance=c;
			debug_Safety=d;
		}
		dir += step;
	}
	//ROS_INFO("best_value=%f, Conformance=%f, Continue=%f, Distance=%f, Safety=%f",best_value,debug_Conformance, debug_Continue, debug_Distance, debug_Safety);
	return best_dir;
}
//从固定列表中获得一组轨迹，速度用来区分是向前还是向后，角度用来决定转弯半径。
//这里存在一个问题，这种评估其实只使用了方向，实际上应该是角速度和线速度的矢量叠加！
//但由于机器人在时刻的调整，而这种方式是负反馈，因此实际的效果，看起来就像是pd调节不好的小车在过弯
//再次强调，LUT列表的轨迹并不是基于速度和方向的仿真轨迹！
sensor_msgs::PointCloud* RobotOperator::getPointCloud(double direction, double velocity)
{
	if(direction < -1) direction = -1;
	if(direction > 1) direction = 1;
	int offset = (velocity >= 0) ? LUT_RESOLUTION : 3*LUT_RESOLUTION + 1;
	int table_index = (direction * LUT_RESOLUTION) + offset;
	return mTrajTable[table_index];
}
