//
// Created by gj on 2022/8/1.
//

#ifndef CATKIN_WS_UDI_CALIB_BOX_FILTER_HPP
#define CATKIN_WS_UDI_CALIB_BOX_FILTER_HPP

#include <pcl/filters/crop_box.h>
#include "lidar_imu_calib/models/cloud_filter/cloud_filter_interface.hpp"

namespace lidar_imu_calib {
    class BoxFilter: public CloudFilterInterface {
    public:
        BoxFilter(YAML::Node node);
        BoxFilter() = default;

        bool Filter(const CloudData::CLOUD_PTR& input_cloud_ptr, CloudData::CLOUD_PTR& filtered_cloud_ptr) override;

        void SetSize(std::vector<float> size);
        void SetOrigin(std::vector<float> origin);
        std::vector<float> GetEdge();

    private:
        void CalculateEdge();

    private:
        pcl::CropBox<CloudData::POINT> pcl_box_filter_;

        std::vector<float> origin_;
        std::vector<float> size_;
        std::vector<float> edge_;
    };
}

#endif //CATKIN_WS_UDI_CALIB_BOX_FILTER_HPP
