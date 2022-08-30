//
// Created by gj on 2022/7/29.
//

#ifndef CATKIN_WS_UDI_CALIB_KEY_FRAME_PUBLISHER_HPP
#define CATKIN_WS_UDI_CALIB_KEY_FRAME_PUBLISHER_HPP
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "lidar_imu_calib/sensor_data/key_frame.hpp"

namespace lidar_imu_calib {
    class KeyFramePublisher {
    public:
        KeyFramePublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        KeyFramePublisher() = default;

        void Publish(KeyFrame& key_frame);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_ = "";
    };
}
#endif //CATKIN_WS_UDI_CALIB_KEY_FRAME_PUBLISHER_HPP
