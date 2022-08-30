//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_POSE_DATA_HPP
#define CATKIN_WS_UDI_CALIB_POSE_DATA_HPP
#include <Eigen/Dense>

namespace lidar_imu_calib {

    class PoseData {
    public:
        double time = 0.0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        Eigen::Vector3f vel = Eigen::Vector3f::Zero();

    public:
        Eigen::Quaternionf GetQuaternion();
    };

}
#endif //CATKIN_WS_UDI_CALIB_POSE_DATA_HPP
