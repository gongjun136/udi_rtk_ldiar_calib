//
// Created by gj on 2022/7/26.
//

#include "lidar_imu_calib/sensor_data/pose_data.hpp"

namespace lidar_imu_calib {

    Eigen::Quaternionf PoseData::GetQuaternion() {
        Eigen::Quaternionf q;
        q = pose.block<3,3>(0,0);
        return q;
    }

}