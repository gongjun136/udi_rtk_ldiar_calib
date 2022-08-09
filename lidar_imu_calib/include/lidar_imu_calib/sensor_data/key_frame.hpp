//
// Created by gj on 2022/7/27.
//

#ifndef CATKIN_WS_UDI_CALIB_KEY_FRAME_HPP
#define CATKIN_WS_UDI_CALIB_KEY_FRAME_HPP

#include <Eigen/Dense>

namespace lidar_imu_calib {
    class KeyFrame {
    public:
        double time = 0.0;
        unsigned int index = 0;
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    public:
        Eigen::Quaternionf GetQuaternion() const;
        Eigen::Vector3f GetTranslation() const;
    };
}

#endif //CATKIN_WS_UDI_CALIB_KEY_FRAME_HPP
