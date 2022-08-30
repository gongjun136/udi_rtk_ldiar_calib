/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#ifndef CATKIN_WS_LIDAR_IMU_CALIB_IMU_SUBSCRIBER_HPP
#define CATKIN_WS_LIDAR_IMU_CALIB_IMU_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"

#include "lidar_imu_calib/sensor_data/imu_data.hpp"

namespace lidar_imu_calib {
    class IMUSubscriber {
    public:
        IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        IMUSubscriber() = default;
        void ParseData(std::deque<IMUData>& deque_imu_data);
        void Convert2IMUData(const sensor_msgs::ImuConstPtr& imu_msg_ptr,IMUData* imu_data);

    private:
        void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<IMUData> new_imu_data_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_WS_LIDAR_IMU_CALIB_IMU_SUBSCRIBER_HPP
