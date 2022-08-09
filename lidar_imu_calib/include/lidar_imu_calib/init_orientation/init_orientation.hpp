//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_HPP
#define CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_HPP
#include "Eigen/Core"
#include "yaml-cpp/yaml.h"
#include "glog/logging.h"
#include "lidar_imu_calib/sensor_data/cloud_data.hpp"

#include "lidar_imu_calib/models/registration/registration_interface.hpp"
#include "lidar_imu_calib/models/registration/ndt_registration.hpp"
#include "lidar_imu_calib/models/cloud_filter/cloud_filter_interface.hpp"
#include "lidar_imu_calib/models/cloud_filter/voxel_filter.hpp"
#include "lidar_imu_calib/sensor_data/key_frame.hpp"
#include "lidar_imu_calib/sensor_data/pose_data.hpp"



namespace lidar_imu_calib{
    class InitOrientation{
    public:
        // 帧的结构，包含位姿和对应的点云数据
        struct Frame {
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            CloudData cloud_data;
        };
    public:
        InitOrientation();

        bool SetInitPose(PoseData &init_data);

        bool Update(const CloudData& cloud_data,Eigen::Matrix4f & cloud_pose, Eigen::Matrix4f& cloud_relative_pose);

        bool HasNewKeyFrame(){ return has_new_key_frame_; };

        bool GetBeginEndTime(double& start_time, double& end_time);

        int& GetKeyFrameNum(){ return key_frame_num_; }

        std::deque<KeyFrame>& GetAllKeyFrame(){return all_key_frames_;};

        KeyFrame& GetCurrentKeyFrame(){ return current_key_frame_; };

        CloudData& GetCurrentKeyScan() {
//            key_scan.time = current_key_scan_.time;
//            key_scan.cloud_ptr.reset(
//                    new CloudData::CLOUD(*current_key_scan_.cloud_ptr)
//            );
            return current_key_scan_;
        }

//        KeyFrame& GetCurrentKeyGNSS() {
//            return current_key_gnss_;
//        }

    private:
        bool InitParam(const YAML::Node &config_node);

        bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);

        bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface> &filter_ptr,
                        const YAML::Node &config_node);
        // 初始化数据路径
        bool InitDataPath(const YAML::Node& config_node);

        bool UpdateWithNewFrame(const Frame& new_key_frame);
    private:
        Eigen::Matrix4f init_pose_;

        std::string data_path_ = "";

        std::shared_ptr<RegistrationInterface> registration_ptr_;


        // 滤波器指针
        std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
        std::shared_ptr<CloudFilterInterface> display_filter_ptr_;

        // 保存局部和全局关键帧
        std::deque<Frame> local_map_frames_;
        std::deque<KeyFrame> all_key_frames_;
//        std::deque<Frame> global_map_frames_;

        // 数据指针
        CloudData::CLOUD_PTR local_map_ptr_;
        CloudData::CLOUD_PTR global_map_ptr_;
        CloudData::CLOUD_PTR result_cloud_ptr_;

        // 当前帧
        Frame current_frame_;
        KeyFrame current_key_frame_;
        CloudData current_key_scan_;

        //　是否有关键帧
        bool has_new_key_frame_ = false;
        bool has_new_local_map_ = false;
        bool has_new_global_map_ = false;

        // 提取关键帧的参数
        float key_frame_distance_ = 0.5;
        float key_frame_diff_angle_ = 5.0;
        int local_frame_num_ = 20;

        //截取数据集后的有效时间段:ros系统时间
        double init_time_=-1;
        double start_time_=-1;
        double end_time_=-1;


        int key_frame_num_;
    };
}


#endif //CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_HPP
