/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月11日 
*/

#ifndef CATKIN_WS_UDI_CALIB_LIDAR_ODOMETRY_HPP
#define CATKIN_WS_UDI_CALIB_LIDAR_ODOMETRY_HPP
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

namespace lidar_imu_calib{
    class LiDAROdometry {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<LiDAROdometry> Ptr;

        //odom数据类型
        struct OdomData {
            double timestamp;
            Eigen::Matrix4f pose; // cur scan to first scan
        };

        LiDAROdometry() = default;

//        static pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndtInit(
//                double ndt_resolution);
//
        void feedScan(double timestamp,
                      CloudData::CLOUD_PTR& cur_scan,
                      Eigen::Matrix4d pose_predict = Eigen::Matrix4d::Identity(),
                      const bool update_map = true);
//
//        void clearOdomData();

//        void setTargetMap(VPointCloud::Ptr map_cloud_in);
//
//        void saveTargetMap(const std::string& path) const {
//            std::cout << "Save NDT target map to " << path
//                      << "; size: " << map_cloud_->size() << std::endl;
//            pcl::io::savePCDFileASCII(path, *map_cloud_);
//        }
//
//        const VPointCloud::Ptr getTargetMap(){
//            return map_cloud_;
//        }
//
//        const pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr& getNDTPtr() const {
//            return ndt_omp_;
//        }

//        const std::vector<OdomData, Eigen::aligned_allocator<OdomData>> &get_odom_data()  {
//            return odom_data_;
//        }
        std::vector<OdomData, Eigen::aligned_allocator<OdomData>> odom_data_;

    private:

//        void registration(const VPointCloud::Ptr& cur_scan,
//                          const Eigen::Matrix4d& pose_predict,
//                          Eigen::Matrix4d& pose_out,
//                          VPointCloud::Ptr scan_in_target);
//
//        void updateKeyScan(const VPointCloud::Ptr& cur_scan, const OdomData& odom_data);
//
//        bool checkKeyScan(const OdomData& odomdata);
//
//        // Normalize angle to be between [-180, 180]
//        static inline double normalize_angle(double ang_degree) {
//            if(ang_degree > 180)
//                ang_degree -= 360;
//
//            if(ang_degree < -180)
//                ang_degree += 360;
//            return ang_degree;
//        }


    private:

//        VPointCloud::Ptr map_cloud_;
//
//        pclomp::NormalDistributionsTransform<VPoint, VPoint>::Ptr ndt_omp_;
//
//        std::vector<size_t> key_frame_index_;

    };
}

#endif //CATKIN_WS_UDI_CALIB_LIDAR_ODOMETRY_HPP
