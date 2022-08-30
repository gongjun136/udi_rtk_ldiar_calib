/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月12日 
*/

#ifndef CATKIN_WS_UDI_CALIB_VOXEL_FILTER_HPP
#define CATKIN_WS_UDI_CALIB_VOXEL_FILTER_HPP
#include <pcl/filters/voxel_grid.h>
#include "lidar_imu_calib/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_imu_calib {

    class VoxelFilter: public CloudFilterInterface {
    public:
        VoxelFilter(const YAML::Node& node);
        VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

    private:
        bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    private:
        pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
    };

}
#endif //CATKIN_WS_UDI_CALIB_VOXEL_FILTER_HPP
