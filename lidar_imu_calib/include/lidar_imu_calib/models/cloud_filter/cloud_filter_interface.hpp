/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月12日 
*/

#ifndef CATKIN_WS_UDI_CALIB_CLOUD_FILTER_INTERFACE_HPP
#define CATKIN_WS_UDI_CALIB_CLOUD_FILTER_INTERFACE_HPP

#include <yaml-cpp/yaml.h>
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

namespace lidar_imu_calib {

    class CloudFilterInterface {
    public:
        virtual ~CloudFilterInterface() = default;

        virtual bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) = 0;
    };

}
#endif //CATKIN_WS_UDI_CALIB_CLOUD_FILTER_INTERFACE_HPP
