/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月19日 
*/

#ifndef CATKIN_WS_UDI_CALIB_GNSS_SUBSCRIBER_HPP
#define CATKIN_WS_UDI_CALIB_GNSS_SUBSCRIBER_HPP
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"

#include "lidar_imu_calib/sensor_data/gnss_data.hpp"

namespace lidar_imu_calib {
    class GNSSSubscriber {
    public:
        GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
        GNSSSubscriber() = default;
        void ParseData(std::deque<GNSSData>& deque_gnss_data);

        void Convert2GNSSData(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr,GNSSData* gnss_data);

    private:
        void msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber subscriber_;
        std::deque<GNSSData> new_gnss_data_;

        std::mutex buff_mutex_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_GNSS_SUBSCRIBER_HPP
