/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月16日 
*/

#ifndef CATKIN_WS_UDI_CALIB_DISTORTION_ADJUST_HPP
#define CATKIN_WS_UDI_CALIB_DISTORTION_ADJUST_HPP
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"

#include "lidar_imu_calib/sensor_data/velocity_data.hpp"
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

namespace lidar_imu_calib {
    class DistortionAdjust {
    public:
        void SetMotionInfo(float scan_period, VelocityData velocity_data);
        bool AdjustCloud(CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& output_cloud_ptr);

    private:
        inline Eigen::Matrix3f UpdateMatrix(float real_time);

    private:
        float scan_period_;
        Eigen::Vector3f velocity_;
        Eigen::Vector3f angular_rate_;
    };
}
#endif //CATKIN_WS_UDI_CALIB_DISTORTION_ADJUST_HPP
