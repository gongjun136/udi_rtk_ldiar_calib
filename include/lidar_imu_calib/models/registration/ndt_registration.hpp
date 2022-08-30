/** 
* @Description: 配准子类：多线程ndt
* @Author: gj
* @Date: 2022年07月11日 
*/

#ifndef CATKIN_WS_UDI_CALIB_NDT_REGISTRATION_HPP
#define CATKIN_WS_UDI_CALIB_NDT_REGISTRATION_HPP
//#include <pcl/registration/ndt.h>
//#include "lidar_imu_calib/models/registration/registration_interface.hpp"
////#include "pclomp/ndt_omp.h"
//
//namespace lidar_imu_calib {
//    class NDTRegistration: public RegistrationInterface {
//    public:
//        NDTRegistration(const YAML::Node& node);
//
//        NDTRegistration(float res, float step_size, float trans_eps, int max_iter, int num_thr,
//                        std::string neighbor_search_method);
//
//        bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
//        /**
//         * @brief 配准函数
//         * @param input_source
//         * @param predict_pose
//         * @param result_cloud_ptr
//         * @param result_pose source点云到target点云的变换
//         * @return
//         */
//        bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
//                       const Eigen::Matrix4f& predict_pose,
//                       CloudData::CLOUD_PTR& result_cloud_ptr,
//                       Eigen::Matrix4f& result_pose) override;
//        float GetFitnessScore() override;
//
//    private:
//        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter, int num_thr,
//                                  std::string neighbor_search_method);
//
//    private:
//        pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
//    };
//
//}

#include <pcl/registration/ndt.h>
#include "lidar_imu_calib/models/registration/registration_interface.hpp"

namespace lidar_imu_calib {
    class NDTRegistration: public RegistrationInterface {
    public:
        NDTRegistration(const YAML::Node& node);
        NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

        bool SetInputTarget(const CloudData::CLOUD_PTR& input_target) override;
        bool ScanMatch(const CloudData::CLOUD_PTR& input_source,
                       const Eigen::Matrix4f& predict_pose,
                       CloudData::CLOUD_PTR& result_cloud_ptr,
                       Eigen::Matrix4f& result_pose) override;
        float GetFitnessScore() override;

    private:
        bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

    private:
        pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
    };
}

#endif //CATKIN_WS_UDI_CALIB_NDT_REGISTRATION_HPP
