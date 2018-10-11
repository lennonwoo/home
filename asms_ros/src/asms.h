#ifndef ASMS_ROS_ASMS_H_
#define ASMS_ROS_ASMS_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <boost/thread/pthread/shared_mutex.hpp>
#include <sensor_msgs/PointCloud2.h>

#include "colotracker.h"
#include "asms_ros/BoundingBox.h"

class ASMS {
public:
  explicit ASMS(const ros::NodeHandle &nh);
  ~ASMS();

private:
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

  void initParameters();
  void init();

  void enableTrackCallback(const std_msgs::Bool& enableTrack);
  void trackerBBoxCallback(const asms_ros::BoundingBox& box);
  void cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg,
                      const sensor_msgs::PointCloud2ConstPtr& pc2Msg);
  bool isTracking();
  void publishBBox(const BBox* bbox);

  ros::NodeHandle nodeHandle_;
  ColorTracker *tracker_;
  asms_ros::BoundingBox pubBBox;
  BBox *bbox;

  int update_interval_;

  message_filters::Synchronizer<MySyncPolicy> *sync_;
  message_filters::Subscriber<sensor_msgs::Image> *imageSub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *pc2Sub_;

  ros::Subscriber enableTrackSub_;
  ros::Subscriber bboxSub_;
  ros::Publisher bboxPub_;

  cv::Mat camImageCopy_;
  sensor_msgs::PointCloud2 pc2Msg_;
  boost::shared_mutex mutexIsTracking_;

  bool isTracking_;
};

#endif // ASMS_ROS_ASMS_H_
