/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月13日 
*/

#ifndef CATKIN_WS_UDI_CALIB_ODOMETRY_PUBLISHER_HPP
#define CATKIN_WS_UDI_CALIB_ODOMETRY_PUBLISHER_HPP
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace lidar_imu_calib {
    class OdometryPublisher {
    public:
        OdometryPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string base_frame_id,
                          std::string child_frame_id,
                          int buff_size);
        OdometryPublisher() = default;

        void Publish(const Eigen::Matrix4f& transform_matrix, double time);
        void Publish(const Eigen::Matrix4f& transform_matrix);

        bool HasSubscribers();

    private:
        void PublishData(const Eigen::Matrix4f& transform_matrix, ros::Time time);

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        nav_msgs::Odometry odometry_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_ODOMETRY_PUBLISHER_HPP
