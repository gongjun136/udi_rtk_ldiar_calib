//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_ODOMETRY_SUBSCRIBER_HPP
#define CATKIN_WS_UDI_CALIB_ODOMETRY_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "lidar_imu_calib/sensor_data/pose_data.hpp"

namespace lidar_imu_calib {
    class OdometrySubscriber {
    public:
        OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        OdometrySubscriber() = default;
        void ParseData(std::deque<PoseData>& deque_pose_data);

    private:
        void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<PoseData> new_pose_data_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_ODOMETRY_SUBSCRIBER_HPP
