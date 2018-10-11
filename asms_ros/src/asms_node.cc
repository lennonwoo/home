#include "asms.h"

#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "asms_ros");
    ros::NodeHandle nh("~");
    ASMS asms(nh);

    ros::spin();
    return 0;
}
