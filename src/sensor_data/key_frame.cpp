//
// Created by gj on 2022/7/27.
//

#include "lidar_imu_calib/sensor_data/key_frame.hpp"

namespace lidar_imu_calib {

    Eigen::Quaternionf KeyFrame::GetQuaternion() const {
        Eigen::Quaternionf q;
        q = pose.block<3,3>(0,0);

        return q;
    }

    Eigen::Vector3f KeyFrame::GetTranslation() const {
        Eigen::Vector3f t = pose.block<3,1>(0,3);

        return t;
    }

}