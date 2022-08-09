/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月12日 
*/

#ifndef CATKIN_WS_UDI_CALIB_NO_FILTER_HPP
#define CATKIN_WS_UDI_CALIB_NO_FILTER_HPP

#include "lidar_imu_calib/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_imu_calib {
    class NoFilter: public CloudFilterInterface {
    public:
        NoFilter();

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;
    };
}
#endif //CATKIN_WS_UDI_CALIB_NO_FILTER_HPP
