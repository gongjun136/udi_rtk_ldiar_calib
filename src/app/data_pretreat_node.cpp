/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#include <ros/ros.h>
#include "glog/logging.h"

#include "lidar_imu_calib/global_defination/global_defination.h"
#include "lidar_imu_calib/data_pretreat/data_pretreat_flow.hpp"

using namespace lidar_imu_calib;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_colorlogtostderr=true;
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}