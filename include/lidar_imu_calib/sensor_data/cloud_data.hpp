/** 
* @Description: 自定义点云
* @Author: gj
* @Date: 2022年07月07日 
*/

#ifndef CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_DATA_HPP
#define CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_DATA_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_imu_calib {
    class CloudData {
    public:
        using POINT = pcl::PointXYZ;
        using CLOUD = pcl::PointCloud<POINT>;
        using CLOUD_PTR = CLOUD::Ptr;

    public:
        CloudData()
                :cloud_ptr(new CLOUD()) {
        }

    public:
        double time = 0.0;
        CLOUD_PTR cloud_ptr;
    };
}

#endif //CATKIN_WS_LIDAR_IMU_CALIB_CLOUD_DATA_HPP
