//
// Created by gj on 2022/7/28.
//
#include "lidar_imu_calib/back_end_optimization/back_end_optimization.hpp"
#include "lidar_imu_calib/global_defination/global_defination.h"
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


namespace lidar_imu_calib{

    BackEndOptimization::BackEndOptimization() : global_map_ptr_(new CloudData::CLOUD),
                                                 current_scan_ptr_(new CloudData::CLOUD),
                                                 local_map_ptr_(new CloudData::CLOUD) {

        std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);

        InitParam(config_node);
        // 初始化局部地图滤波器
        InitFilter("global_map", global_map_filter_ptr_, config_node);

        //初始化关键帧滤波器
        InitFilter("frame", frame_filter_ptr_, config_node);

        InitLocalMapSegmenter(config_node);

        InitRegistration(registration_ptr_, config_node);

        distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();


    }

    bool BackEndOptimization::InitParam(const YAML::Node &config_node) {
        data_path_ = WORK_SPACE_PATH+config_node["data_path"].as<std::string>();
        lidar_frequency_ = config_node["measurements"]["lidar"]["frequency"].as<int>();

    }

    bool BackEndOptimization::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
        std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
        std::cout << "后端" << filter_user << "点云滤波方法: " << filter_mothod;

        if (filter_mothod == "voxel_filter") {
            filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
        } else {
            LOG(ERROR) << "配置文件中点云滤波方法不支持！！！" << std::endl;
            return false;
        }

        return true;
    }

    bool BackEndOptimization::InitLocalMapSegmenter(const YAML::Node &config_node) {
        local_map_segmenter_ptr_ = std::make_shared<BoxFilter>(config_node);
        return true;
    }

    bool BackEndOptimization::InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node) {
        std::string registration_method = config_node["registration_method"].as<std::string>();
        std::cout << "后端Lidar配准方法：" << registration_method<<std::endl;

        if (registration_method == "NDT") {
            registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
        }else{
            LOG(ERROR) << "配置文件中配准方法不支持！！！" << std::endl;
        }
    }

    bool BackEndOptimization::Update(){
        return false;
    }
    bool BackEndOptimization::UnDistortion(VelocityData& current_velocity_data,CloudData::CLOUD_PTR current_cloud_data_ptr) {
        distortion_adjust_ptr_->SetMotionInfo(1/10, current_velocity_data);
        distortion_adjust_ptr_->AdjustCloud(current_cloud_data_ptr, current_cloud_data_ptr);
//        global_map_ptr_+=
    }

    bool BackEndOptimization::AddToGlobalMap(CloudData::CLOUD_PTR cloud_data_ptr, Eigen::Matrix4f& pose){
        // 对输入点云进行滤波
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(cloud_data_ptr, filtered_cloud_ptr);

        CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
        pcl::transformPointCloud(*filtered_cloud_ptr, *transformed_cloud_ptr, pose);
//        pcl::transformPointCloud(*cloud_data_ptr, *transformed_cloud_ptr, pose);
        *global_map_ptr_ += *transformed_cloud_ptr;
//        std::cout << "global size:" << global_map_ptr_->size() << std::endl;

//        CloudData::CLOUD_PTR filtered_global_map_ptr(new CloudData::CLOUD());
//        global_map_filter_ptr_->Filter(global_map_ptr_, filtered_global_map_ptr);

    }

    bool BackEndOptimization::SaveGlobalMap(){
        pcl::io::savePCDFileBinary("/home/gj/global_map.pcd", *global_map_ptr_);
        return true;
    }

    bool BackEndOptimization::SetInitPoseAndLocalMap(Eigen::Matrix4f &init_pose) {
        init_pose_=init_pose;

//        registration_ptr_->SetInputTarget(global_map_ptr_);

        ResetLocalMap(
                init_pose_(0,3),
                init_pose_(1,3),
                init_pose_(2,3)
        );
        return true;
    }

    bool BackEndOptimization::UpdateLaserRelativePose(const CloudData::CLOUD_PTR cloud_data_ptr,
                                                      const Eigen::Matrix4f &cloud_predit_pose,
                                                      Eigen::Matrix4f &cloud_pose,
                                                      Eigen::Matrix4f& current_lidar_relative_pose) {

        static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
        static Eigen::Matrix4f last_pose = init_pose_;
        static Eigen::Matrix4f predict_pose = init_pose_;

//        std::cout << "cloud size:" << cloud_data_ptr->size() << std::endl;

        // remove invalid measurements:
        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*cloud_data_ptr, *cloud_data_ptr, indices);

        // downsample:
        CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
        frame_filter_ptr_->Filter(cloud_data_ptr, filtered_cloud_ptr);

//        std::cout << "cloud filter done" << std::endl;

        predict_pose = cloud_predit_pose;

        // matching:
        CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
        registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cloud_pose);
        pcl::transformPointCloud(*cloud_data_ptr, *current_scan_ptr_, cloud_pose);

//        std::cout << "cloud done" << std::endl;

        // update predicted pose:
        step_pose = last_pose.inverse() * cloud_pose;
        predict_pose = cloud_pose * step_pose;
        last_pose = cloud_pose;

        // shall the local map be updated:
        std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();
        for (int i = 0; i < 3; i++) {
            if (
                    fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
                    fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
                    ) {
                continue;
            }

            ResetLocalMap(
                    cloud_pose(0, 3),
                    cloud_pose(1, 3),
                    cloud_pose(2, 3)
            );
            break;
        }


        return true;
    }


    bool BackEndOptimization::ResetLocalMap(
            float x,
            float y,
            float z
    ) {
        std::cout << "reset localmap" << std::endl;
        std::vector<float> origin = {x, y, z};

        // segment local map from global map:
        local_map_segmenter_ptr_->SetOrigin(origin);
        local_map_segmenter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

        registration_ptr_->SetInputTarget(local_map_ptr_);

        has_new_local_map_ = true;

        std::vector<float> edge = local_map_segmenter_ptr_->GetEdge();

        LOG(INFO) << "New local map:"
                  << edge.at(0) << ","
                  << edge.at(1) << ","
                  << edge.at(2) << ","
                  << edge.at(3) << ","
                  << edge.at(4) << ","
                  << edge.at(5) << std::endl << std::endl;

        return true;
    }

    bool BackEndOptimization::UpdateGNSSRelativePose(){

        return true;
    }

    bool BackEndOptimization::Solve() {

        return true;
    }


    CloudData::CLOUD_PTR & BackEndOptimization::GetGlobalMap() {
        // downsample global map for visualization:
        return global_map_ptr_;
    }


    bool BackEndOptimization::GetAllData() {

    }

    bool BackEndOptimization::LoadKeyFrame() {



    }



}
