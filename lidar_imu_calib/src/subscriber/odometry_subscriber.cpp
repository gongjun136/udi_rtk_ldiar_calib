//
// Created by gj on 2022/7/26.
//

#include "lidar_imu_calib/subscriber/odometry_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_imu_calib{
    OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
            :nh_(nh) {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::msg_callback, this);
    }

    void OdometrySubscriber::msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
        buff_mutex_.lock();
        PoseData pose_data;
        pose_data.time = odom_msg_ptr->header.stamp.toSec();

        // set the position:
        pose_data.pose(0,3) = odom_msg_ptr->pose.pose.position.x;
        pose_data.pose(1,3) = odom_msg_ptr->pose.pose.position.y;
        pose_data.pose(2,3) = odom_msg_ptr->pose.pose.position.z;

        // set the orientation:
        Eigen::Quaternionf q;
        q.x() = odom_msg_ptr->pose.pose.orientation.x;
        q.y() = odom_msg_ptr->pose.pose.orientation.y;
        q.z() = odom_msg_ptr->pose.pose.orientation.z;
        q.w() = odom_msg_ptr->pose.pose.orientation.w;
        pose_data.pose.block<3,3>(0,0) = q.matrix();

        // set the linear velocity:
        pose_data.vel.x() = odom_msg_ptr->twist.twist.linear.x;
        pose_data.vel.y() = odom_msg_ptr->twist.twist.linear.y;
        pose_data.vel.z() = odom_msg_ptr->twist.twist.linear.z;

        new_pose_data_.push_back(pose_data);

        buff_mutex_.unlock();
    }

    void OdometrySubscriber::ParseData(std::deque<PoseData>& pose_data_buff) {
        buff_mutex_.lock();
        if (new_pose_data_.size() > 0) {
            pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
            new_pose_data_.clear();
        }
        buff_mutex_.unlock();
    }
}