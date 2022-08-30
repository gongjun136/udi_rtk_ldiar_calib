/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#ifndef CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_SUBSCRIBER_HPP
#define CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

namespace lidar_imu_calib{
    class CloudSubscriber {
    public:
        CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        CloudSubscriber() = default;
        void ParseData(std::deque<CloudData>& deque_cloud_data);
        //将ros点云转化为自定义点云类型
        void Convert2CloudData(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr, CloudData* cloud_data);

    private:
        void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

    public:
        std::deque<CloudData> new_cloud_data_;

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::mutex buff_mutex_;
    };
}


#endif //CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_SUBSCRIBER_HPP
