//
// Created by gj on 2022/7/28.
//

#ifndef CATKIN_WS_UDI_CALIB_TF_BROADCASTER_HPP
#define CATKIN_WS_UDI_CALIB_TF_BROADCASTER_HPP
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace lidar_imu_calib {
    class TFBroadCaster {
    public:
        TFBroadCaster(std::string frame_id, std::string child_frame_id);
        TFBroadCaster() = default;
        void SendTransform(Eigen::Matrix4f pose, double time);
    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_TF_BROADCASTER_HPP
