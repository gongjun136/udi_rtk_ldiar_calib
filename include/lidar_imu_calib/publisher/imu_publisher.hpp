//
// Created by udi on 2022/7/25.
//

#ifndef CATKIN_WS_UDI_CALIB_IMU_PUBLISHER_HPP
#define CATKIN_WS_UDI_CALIB_IMU_PUBLISHER_HPP
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_imu_calib/sensor_data/imu_data.hpp"

namespace lidar_imu_calib {
    class IMUPublisher {
    public:
        IMUPublisher(
                ros::NodeHandle& nh,
                std::string topic_name,
                std::string frame_id,
                size_t buff_size
        );
        IMUPublisher() = default;

        void Publish(const IMUData &imu_data, double time);
        void Publish(const IMUData &imu_data);

        bool HasSubscribers(void);

    private:
        void PublishData(const IMUData &imu_data, ros::Time time);

        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;

        sensor_msgs::Imu imu_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_IMU_PUBLISHER_HPP
