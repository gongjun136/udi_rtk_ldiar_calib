//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_POS_VEL_PUBLISHER_HPP
#define CATKIN_WS_UDI_CALIB_POS_VEL_PUBLISHER_HPP
#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "lidar_imu_calib/sensor_data/pos_vel_data.hpp"

#include "lidar_imu_calib/PosVel.h"

namespace lidar_imu_calib {

    class PosVelPublisher {
    public:
        PosVelPublisher(
                ros::NodeHandle& nh,
                std::string topic_name,
                std::string base_frame_id,
                std::string child_frame_id,
                int buff_size
        );
        PosVelPublisher() = default;

        void Publish(const PosVelData &pos_vel_data, const double &time);
        void Publish(const PosVelData &pos_vel_data);

        bool HasSubscribers();

    private:
        void PublishData(
                const PosVelData &pos_vel_data,
                ros::Time time
        );

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        PosVel pos_vel_msg_;
    };

}
#endif //CATKIN_WS_UDI_CALIB_POS_VEL_PUBLISHER_HPP
