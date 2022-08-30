//
// Created by gj on 2022/7/28.
//

#ifndef CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_HPP
#define CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_HPP
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <deque>
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"
#include "lidar_imu_calib/sensor_data/key_frame.hpp"
#include "lidar_imu_calib/models/scan_adjust/distortion_adjust.hpp"
#include "lidar_imu_calib/sensor_data/velocity_data.hpp"
#include "lidar_imu_calib/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_imu_calib/models/cloud_filter/voxel_filter.hpp"
#include "lidar_imu_calib/models/cloud_filter/box_filter.hpp"

#include "lidar_imu_calib/models/registration/registration_interface.hpp"
#include "lidar_imu_calib/models/registration/ndt_registration.hpp"


namespace lidar_imu_calib{

    class BackEndOptimization{
    public:
        struct KeyFrameData{
            KeyFrame key_frame;
            CloudData::CLOUD_PTR cloud_ptr;
            struct angular_velocity {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
            };

            struct linear_velocity {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
            };
        };
    public:
        BackEndOptimization();




        bool Update();

        bool LoadKeyFrame();

        bool UnDistortion(VelocityData& current_velocity_data,CloudData::CLOUD_PTR current_cloud_data_ptr);

        bool AddToGlobalMap(CloudData::CLOUD_PTR cloud_data_ptr, Eigen::Matrix4f& pose);

        bool GetAllData();

        bool SaveGlobalMap();

        bool UpdateLaserRelativePose(const CloudData::CLOUD_PTR cloud_data_ptr,
                                     const Eigen::Matrix4f &cloud_predit_pose,
                                     Eigen::Matrix4f &cloud_pose,
                                     Eigen::Matrix4f& current_lidar_relative_pose);

        bool UpdateGNSSRelativePose();

        bool Solve();

        CloudData::CLOUD_PTR & GetGlobalMap();

        bool HasInited() const { return has_inited_; }

        bool SetInitPoseAndLocalMap(Eigen::Matrix4f &init_pose);

    private:
        bool InitParam(const YAML::Node &config_node);

        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr,
                        const YAML::Node &config_node);

        bool InitLocalMapSegmenter(const YAML::Node &config_node);

        bool InitRegistration(std::shared_ptr<RegistrationInterface> &registration_ptr, const YAML::Node &config_node);

        bool ResetLocalMap(float x, float y, float z);

    private:

        std::string data_path_ = "";
        int lidar_frequency_;

        std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

        std::deque<KeyFrameData> all_key_frame_data_;

        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR current_scan_ptr_;

        std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<BoxFilter> local_map_segmenter_ptr_;


        std::shared_ptr<RegistrationInterface> registration_ptr_;



        bool has_inited_=false;
        Eigen::Matrix4f init_pose_;

        bool has_new_local_map_=false;
    };
}

#endif //CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_HPP
