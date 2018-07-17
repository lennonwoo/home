#include <ros/ros.h>

#include "xf_ros/asr.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "xf_ros");
    ros::NodeHandle nodeHandle("~");
    xf_ros::ASR asr(nodeHandle);
    asr.run_asr();

    ros::spin();
    return 0;
}
