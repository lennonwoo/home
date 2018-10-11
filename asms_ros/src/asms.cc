#include "asms.h"

#include <time.h>

#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

double what_time_is_it_now()
{
  struct timeval time;
  if (gettimeofday(&time,NULL)){
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

ASMS::ASMS(const ros::NodeHandle &nh) {
  nodeHandle_ = nh;
  isTracking_ = false;

  initParameters();
  init();
}

ASMS::~ASMS() {
  if (tracker_ != nullptr) {
    delete tracker_;
    tracker_ = nullptr;
    delete imageSub_;
    imageSub_ = nullptr;
    delete pc2Sub_;
    pc2Sub_ = nullptr;
    delete sync_;
    sync_ = nullptr;
  }
}

void ASMS::init() {
  tracker_ = new ColorTracker();

  std::string enableTrackSubTopicName;
  int enableTrackSubQueueSize;
  nodeHandle_.param("subscribers/enable_track/topic",
                    enableTrackSubTopicName,
                    std::string("/asms/enable_track"));
  nodeHandle_.param("subscribers/enable_track/queue_size",
                    enableTrackSubQueueSize, 1);
  enableTrackSub_ = nodeHandle_.subscribe(enableTrackSubTopicName,
    enableTrackSubQueueSize, &ASMS::enableTrackCallback, this);

  std::string cameraTopicName;
  std::string pc2TopicName;
  int cameraQueueSize;
  int pc2QueueSize;
  nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName,
                    std::string("/camera/color/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("subscribers/depth_reading/topic", pc2TopicName,
                    std::string("/camera/depth_registered/points"));
  imageSub_ = new message_filters::Subscriber<sensor_msgs::Image>(nodeHandle_, cameraTopicName, cameraQueueSize);
  pc2Sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nodeHandle_, pc2TopicName, pc2QueueSize);
  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *imageSub_, *pc2Sub_);
  sync_->registerCallback(boost::bind(&ASMS::cameraCallback, this, _1, _2));

  std::string bboxSubTopicName;
  int bboxSubQueueSize;
  nodeHandle_.param("subscribers/bbox/topic", bboxSubTopicName, std::string("/asms/bbox_sub"));
  nodeHandle_.param("subscribers/bbox/queue_size", bboxSubQueueSize, 1);
  bboxSub_ = nodeHandle_.subscribe(bboxSubTopicName, bboxSubQueueSize, &ASMS::trackerBBoxCallback, this);

  std::string bboxPubTopicName;
  int bboxPubQueueSize;
  nodeHandle_.param("publishers/bbox/topic", bboxPubTopicName, std::string("/asms/bbox_pub"));
  nodeHandle_.param("publishers/bbox/queue_size", bboxSubQueueSize, 1);
  bboxPub_ = nodeHandle_.advertise<asms_ros::BoundingBox>(bboxPubTopicName, bboxPubQueueSize, false);
}

void ASMS::initParameters() {
  nodeHandle_.param("asms/update_interval", update_interval_, 5);
}

void ASMS::enableTrackCallback(const std_msgs::Bool& enableTrack) {
  boost::unique_lock<boost::shared_mutex> lockCallback(mutexIsTracking_);
  isTracking_ = enableTrack.data;
  ROS_INFO("Change tracking status: %d", isTracking_);
}

bool ASMS::isTracking() {
  boost::unique_lock<boost::shared_mutex> lockCallback(mutexIsTracking_);
  return isTracking_;
}

void ASMS::trackerBBoxCallback(const asms_ros::BoundingBox& box) {
  int x1 = box.xmin;
  int y1 = box.ymin;
  int x2 = box.xmax;
  int y2 = box.ymax;
  cv_bridge::CvImagePtr cam_image;
  cam_image = cv_bridge::toCvCopy(box.image, sensor_msgs::image_encodings::BGR8);

  if (cam_image) {
    //boost::unique_lock<boost::shared_mutex> lockCallback(mutexCallback_);
    //camImageCopy_ = cam_image->image.clone();
    tracker_->init(cam_image->image, x1, y1, x2, y2);
    {
      boost::unique_lock<boost::shared_mutex> lockCallback(mutexIsTracking_);
      isTracking_ = true;
    }
  }
}

void ASMS::cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg,
                          const sensor_msgs::PointCloud2ConstPtr& pc2Msg) {
  if (!isTracking()) return;

  cv_bridge::CvImagePtr cam_image;
  cam_image = cv_bridge::toCvCopy(imageMsg, sensor_msgs::image_encodings::BGR8);

  if (cam_image) {
    {
      double start_time = what_time_is_it_now();
      bbox = tracker_->track(cam_image->image);

      pubBBox.header = std_msgs::Header();
      pubBBox.header.stamp = ros::Time::now();
      pubBBox.probability = bbox->accuracy;
      pubBBox.xmin = static_cast<int>(bbox->x);
      pubBBox.ymin = static_cast<int>(bbox->y);
      pubBBox.xmax = static_cast<int>(bbox->width) + pubBBox.xmin;
      pubBBox.ymax = static_cast<int>(bbox->height) + pubBBox.ymin;
      ROS_INFO("Current accuracy_: %f", bbox->accuracy);

      pubBBox.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", cam_image->image).toImageMsg();
      pubBBox.pc2 = *pc2Msg.get();

      bboxPub_.publish(pubBBox);
      ROS_INFO("The all time used: %f", what_time_is_it_now() - start_time);
    }
  }
}

void ASMS::publishBBox(const BBox* bbox) {
  pubBBox.header = std_msgs::Header();
  pubBBox.header.stamp = ros::Time::now();
  pubBBox.probability = bbox->accuracy;
  pubBBox.xmin = static_cast<int>(bbox->x);
  pubBBox.ymin = static_cast<int>(bbox->y);
  pubBBox.xmax = static_cast<int>(bbox->width) + pubBBox.xmin;
  pubBBox.ymax = static_cast<int>(bbox->height) + pubBBox.ymin;

  pubBBox.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", camImageCopy_).toImageMsg();
  pubBBox.pc2 = pc2Msg_;
  bboxPub_.publish(pubBBox);
}
