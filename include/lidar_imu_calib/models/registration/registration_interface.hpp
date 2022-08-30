/** 
* @Description: 配准父类
* @Author: gj
* @Date: 2022年07月11日 
*/

#ifndef CATKIN_WS_UDI_CALIB_REGISTRATION_INTERFACE_HPP
#define CATKIN_WS_UDI_CALIB_REGISTRATION_INTERFACE_HPP
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

namespace lidar_imu_calib {
    class RegistrationInterface {
    public:
        virtual ~RegistrationInterface() = default;

        virtual bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) = 0;
        virtual bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                               const Eigen::Matrix4f& predict_pose,
                               CloudData::CLOUD_PTR& result_cloud_ptr,
                               Eigen::Matrix4f& result_pose) = 0;
        virtual float GetFitnessScore() = 0;
    };
}

#endif //CATKIN_WS_UDI_CALIB_REGISTRATION_INTERFACE_HPP
