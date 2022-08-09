/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#include "lidar_imu_calib/subscriber/cloud_subscriber.hpp"
#include "glog/logging.h"
#include <iomanip>

namespace lidar_imu_calib{
    CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
            :nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    }

    void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
        CloudData* cloud_data_ptr=new CloudData();
        Convert2CloudData(cloud_msg_ptr,cloud_data_ptr);
        // add new message to buffer:
        buff_mutex_.lock();
        new_cloud_data_.push_back(*cloud_data_ptr);
        buff_mutex_.unlock();
    }

    void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
        buff_mutex_.lock();

        // pipe all available measurements to output buffer:
        if (new_cloud_data_.size() > 0) {
            cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
            new_cloud_data_.clear();
        }

        buff_mutex_.unlock();
    }
    void CloudSubscriber::Convert2CloudData(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr,CloudData* cloud_data_ptr){

        // convert ROS PointCloud2 to pcl::PointCloud<pcl::PointXYZ>:
        cloud_data_ptr->time = cloud_msg_ptr->header.stamp.toSec();
//        std::cout<<"ring:"<<cloud_msg_ptr->ring;
        pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data_ptr->cloud_ptr));

    }
}