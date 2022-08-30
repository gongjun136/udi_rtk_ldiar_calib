//
// Created by udi on 2022/7/25.
//

#ifndef CATKIN_WS_UDI_CALIB_VELOCITY_SUBSCRIBER_HPP
#define CATKIN_WS_UDI_CALIB_VELOCITY_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"

#include "lidar_imu_calib/sensor_data/velocity_data.hpp"

namespace lidar_imu_calib {
    class VelocitySubscriber {
    public:
        VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        VelocitySubscriber() = default;
        void ParseData(std::deque<VelocityData>& deque_velocity_data);

    private:
        void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<VelocityData> new_velocity_data_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_VELOCITY_SUBSCRIBER_HPP
