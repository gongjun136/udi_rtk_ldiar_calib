/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#include "lidar_imu_calib/subscriber/imu_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_imu_calib{
    IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
            :nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
    }

    void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {


        // convert ROS IMU to GeographicLib compatible GNSS message:
        IMUData imu_data;
        Convert2IMUData(imu_msg_ptr, &imu_data);
        // add new message to buffer:
        buff_mutex_.lock();
        new_imu_data_.push_back(imu_data);
        buff_mutex_.unlock();
    }

    void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {
        buff_mutex_.lock();

        // pipe all available measurements to output buffer:
        if (new_imu_data_.size() > 0) {
            imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
            new_imu_data_.clear();
        }

        buff_mutex_.unlock();
    }

    void IMUSubscriber::Convert2IMUData(const sensor_msgs::ImuConstPtr &imu_msg_ptr, IMUData* imu_data_ptr) {
        // convert ROS IMU to GeographicLib compatible GNSS message:
        imu_data_ptr->time = imu_msg_ptr->header.stamp.toSec();

        imu_data_ptr->linear_acceleration.x = imu_msg_ptr->linear_acceleration.x;
        imu_data_ptr->linear_acceleration.y = imu_msg_ptr->linear_acceleration.y;
        imu_data_ptr->linear_acceleration.z = imu_msg_ptr->linear_acceleration.z;

        imu_data_ptr->angular_velocity.x = imu_msg_ptr->angular_velocity.x;
        imu_data_ptr->angular_velocity.y = imu_msg_ptr->angular_velocity.y;
        imu_data_ptr->angular_velocity.z = imu_msg_ptr->angular_velocity.z;

        imu_data_ptr->orientation.x = imu_msg_ptr->orientation.x;
        imu_data_ptr->orientation.y = imu_msg_ptr->orientation.y;
        imu_data_ptr->orientation.z = imu_msg_ptr->orientation.z;
        imu_data_ptr->orientation.w = imu_msg_ptr->orientation.w;
    }
}