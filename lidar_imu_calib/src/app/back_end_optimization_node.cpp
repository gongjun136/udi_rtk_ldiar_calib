//
// Created by gj on 2022/7/28.
//

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_imu_calib/global_defination/global_defination.h"
#include "lidar_imu_calib/back_end_optimization/back_end_optimization_flow.hpp"


using namespace lidar_imu_calib;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_colorlogtostderr=true;
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_optimization_node");
    ros::NodeHandle nh;

//    std::string cloud_topic;
//    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<BackEndOptimizationFlow> back_end_optimization_flow_ptr = std::make_shared<BackEndOptimizationFlow>(nh);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_optimization_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}