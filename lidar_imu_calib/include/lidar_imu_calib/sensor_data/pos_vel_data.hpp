//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_POS_VEL_DATA_HPP
#define CATKIN_WS_UDI_CALIB_POS_VEL_DATA_HPP

#include <string>

#include <Eigen/Dense>

namespace lidar_imu_calib {

    class PosVelData {
    public:
        double time = 0.0;

        Eigen::Vector3f pos = Eigen::Vector3f::Zero();
        Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    };

}
#endif //CATKIN_WS_UDI_CALIB_POS_VEL_DATA_HPP
