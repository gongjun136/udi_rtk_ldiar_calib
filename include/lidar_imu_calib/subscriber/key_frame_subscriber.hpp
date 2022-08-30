//
// Created by gj on 2022/7/30.
//

#ifndef CATKIN_WS_UDI_CALIB_KEY_FRAME_SUBSCRIBER_HPP
#define CATKIN_WS_UDI_CALIB_KEY_FRAME_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_imu_calib/sensor_data/key_frame.hpp"

namespace lidar_imu_calib {
    class KeyFrameSubscriber {
    public:
        KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        KeyFrameSubscriber() = default;
        void ParseData(std::deque<KeyFrame>& key_frame_buff);

    private:
        void msg_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& key_frame_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<KeyFrame> new_key_frame_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_KEY_FRAME_SUBSCRIBER_HPP
