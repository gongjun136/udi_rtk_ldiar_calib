/** 
* @Description: 配准子类：多线程ndt
* @Author: gj
* @Date: 2022年07月11日 
*/

//#include "lidar_imu_calib/models/registration/ndt_registration.hpp"
//#include "glog/logging.h"
//
//namespace lidar_imu_calib {
//
//    NDTRegistration::NDTRegistration(const YAML::Node& node)
//            :ndt_ptr_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
//
//        float res = node["res"].as<float>();
//        float step_size = node["step_size"].as<float>();
//        float trans_eps = node["trans_eps"].as<float>();
//        int max_iter = node["max_iter"].as<int>();
//        int num_thr = node["num_thr"].as<int>();
//        std::string neighbor_search_method = node["neighbor_search_method"].as<std::string>();
//
//
//        SetRegistrationParam(res, step_size, trans_eps, max_iter, num_thr, neighbor_search_method);
//    }
//
//    NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter,int num_thr,
//                                     std::string neighbor_search_method)
//            :ndt_ptr_(new pclomp::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {
//
//        SetRegistrationParam(res, step_size, trans_eps, max_iter, num_thr, neighbor_search_method);
//    }
//
//    bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter,int num_thr,std::string neighbor_search_method) {
//        ndt_ptr_->setResolution(res);
//        ndt_ptr_->setStepSize(step_size);
//        ndt_ptr_->setTransformationEpsilon(trans_eps);
//        ndt_ptr_->setMaximumIterations(max_iter);
////        ndt_ptr_->setNumThreads(num_thr);
//
////        if (neighbor_search_method == "pclomp::DIRECT7") {
////            std::cout << "  " << neighbor_search_method << std::endl;
////            ndt_ptr_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
////        }
//
//
//        std::cout << "NDT 参数:" << std::endl
//                  << "分辨率: " << res << ", "
//                  << "步长: " << step_size << ", "
//                  << "迭代阈值: " << trans_eps << ", "
//                  << "迭代次数: " << max_iter << ", "
//                  << "线程数:" << num_thr << ", "
//                  << std::endl << std::endl;
//
//        return true;
//    }
//
//    bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
////        for (int i = 0; i < input_target->size(); i++) {
////            std::cout << "target point" << input_target->points[i]<<std::endl;
////        }
//        ndt_ptr_->setInputTarget(input_target);
////        std::cout << "input target size:" << input_target->size() << std::endl;
//
//        return true;
//    }
//
//    bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
//                                    const Eigen::Matrix4f& predict_pose,
//                                    CloudData::CLOUD_PTR& result_cloud_ptr,
//                                    Eigen::Matrix4f& result_pose) {
////        std::cout << "input source size:"  << input_source->size() << std::endl;
////        for (int i = 0; i < input_source->size(); i++) {
////            std::cout << "source point" << input_source->points[i]<<std::endl;
////        }
//        ndt_ptr_->setInputSource(input_source);
////        std::cout << "start align." << std::endl;
//        ndt_ptr_->align(*result_cloud_ptr, predict_pose);
////        std::cout << "end align." << std::endl;
//        result_pose = ndt_ptr_->getFinalTransformation();
//
//        return true;
//    }
//
//    float NDTRegistration::GetFitnessScore() {
//        return ndt_ptr_->getFitnessScore();
//    }
//}

#include "lidar_imu_calib/models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace lidar_imu_calib {

    NDTRegistration::NDTRegistration(const YAML::Node& node)
            :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

        float res = node["res"].as<float>();
        float step_size = node["step_size"].as<float>();
        float trans_eps = node["trans_eps"].as<float>();
        int max_iter = node["max_iter"].as<int>();

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
            :ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>()) {

        SetRegistrationParam(res, step_size, trans_eps, max_iter);
    }

    bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
        ndt_ptr_->setResolution(res);
        ndt_ptr_->setStepSize(step_size);
        ndt_ptr_->setTransformationEpsilon(trans_eps);
        ndt_ptr_->setMaximumIterations(max_iter);

        std::cout << "NDT params:" << std::endl
                  << "res: " << res << ", "
                  << "step_size: " << step_size << ", "
                  << "trans_eps: " << trans_eps << ", "
                  << "max_iter: " << max_iter
                  << std::endl << std::endl;

        return true;
    }

    bool NDTRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
        ndt_ptr_->setInputTarget(input_target);

        return true;
    }

    bool NDTRegistration::ScanMatch(const CloudData::CLOUD_PTR& input_source,
                                    const Eigen::Matrix4f& predict_pose,
                                    CloudData::CLOUD_PTR& result_cloud_ptr,
                                    Eigen::Matrix4f& result_pose) {
        ndt_ptr_->setInputSource(input_source);
        ndt_ptr_->align(*result_cloud_ptr, predict_pose);
        result_pose = ndt_ptr_->getFinalTransformation();

        return true;
    }

    float NDTRegistration::GetFitnessScore() {
        return ndt_ptr_->getFitnessScore();
    }
}