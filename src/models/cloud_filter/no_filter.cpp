/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月12日 
*/

#include "lidar_imu_calib/models/cloud_filter/no_filter.hpp"
#include "glog/logging.h"

namespace lidar_imu_calib {
    NoFilter::NoFilter() {
    }

    bool NoFilter::Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) {
        filtered_cloud_ptr.reset(new CloudData::CLOUD(*input_cloud_ptr));
        return true;
    }
}