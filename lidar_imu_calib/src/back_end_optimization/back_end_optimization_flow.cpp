//
// Created by gj on 2022/7/28.
//
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include "lidar_imu_calib/global_defination/global_defination.h"
#include "lidar_imu_calib/back_end_optimization/back_end_optimization_flow.hpp"



namespace lidar_imu_calib{
    BackEndOptimizationFlow::BackEndOptimizationFlow(ros::NodeHandle& nh) {
        //初始化订阅者：
        std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
        const YAML::Node config_node = YAML::LoadFile(config_file_path);
        InitSubscribers(nh, config_node);

//        back_end_optimization_ptr_ = std::make_shared<BackEndOptimization>();

        // a. global point cloud map:
//        global_map_pub_ptr_ =
//                std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100);
//        back_end_laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "back_end/laser_odom", "map", "back_end_lidar", 100);

    }

    bool BackEndOptimizationFlow::Run() {
        static bool has_calibration_result=false;
        static bool has_all_key_frame_data=false;
        if (!has_calibration_result) {
            if (!InitCalibration()) {
                return false;
            }
            if (!has_all_key_frame_data) {

                if (!ReadData())
                    return false;
                while(HasData()) {
                    if (!ValidData())
                        continue;
                }
                std::cout << "all key frame data size:" << all_key_frame_data_.size() << std::endl;
                has_all_key_frame_data=true;
            }

//            while (true) {//一直优化，知道收敛为止
//
//                if (HandEyeCalibration()) {
//                    has_calibration_result=true;
//                    std::cout << "calib finish!!!";
//                    return true;
//                }
//            }
//            // 方案１：
//            if (HandEyeCalibration()) {
//                has_calibration_result=true;
//                std::cout << "calib finish!!!";
//                return true;
//            }
            //方案２：
            if (Estimate()) {
                has_calibration_result=true;
            }

//            return false;
        }
        return true;

    }
    bool BackEndOptimizationFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node) {
        // 初始化订阅者
//        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node["measurements"]["lidar"]["topic_name"].as<std::string>(),
//                                                           config_node["measurements"]["lidar"]["queue_size"].as<int>());
//        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 1000000);
//        gnss_sub_ptr_ =
//                std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);
        gnss2lidar_tf_sub_ptr_ = std::make_shared<TFListener>(nh, "gnss", "lidar");
        // 同步后速度信息:
//        sync_velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/synced_vel", 1000000);

//        sync_imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/synced_imu", 1000000);

        key_scan_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/key_scan", 100000);
        key_frame_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_frame", 100000);
        key_gnss_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(nh, "/key_gnss", 100000);





    }
    bool BackEndOptimizationFlow::InitCalibration() {
        static bool calibration_received = false;
        if (!calibration_received) {
            if (gnss2lidar_tf_sub_ptr_->LookupData(gnss_to_lidar_)) {
                std::cout << "后端接收到初始旋转：" << std::endl << gnss_to_lidar_ << std::endl;
                calibration_received = true;
            }
        }

        return calibration_received;
    }

    bool BackEndOptimizationFlow::ReadData(){
        key_scan_sub_ptr_->ParseData(key_scan_buff_);
        key_frame_sub_ptr_->ParseData(key_frame_buff_);
        key_gnss_sub_ptr_->ParseData(key_gnss_buff_);
//        sync_imu_sub_ptr_->ParseData(sync_imu_buff_);
//        sync_velocity_sub_ptr_->ParseData(sync_velocity_buff_);
        std::cout << "scan buff:" << key_scan_buff_.size() << std::endl;
        std::cout << "frame buff:" << key_frame_buff_.size() << std::endl;
        std::cout << "gnss buff:" << key_gnss_buff_.size() << std::endl;
//        std::cout << "imu buff:" << sync_imu_buff_.size() << std::endl;
    }


    bool BackEndOptimizationFlow::HasData() {
        if (key_scan_buff_.size() == 0
            || key_frame_buff_.size() == 0
            || key_gnss_buff_.size() == 0
            //            ||　sync_imu_buff_.size() == 0
//            || sync_velocity_buff_.size() == 0
                ) {

            return false;
        }
//        std::cout << "has data." << std::endl;
        return true;
    }

    bool BackEndOptimizationFlow::ValidData() {
        current_key_scan_ = key_scan_buff_.front();
        current_key_frame_ = key_frame_buff_.front();
        current_key_gnss_ = key_gnss_buff_.front();
//        current_sync_imu_ = sync_imu_buff_.front();
//        current_sync_velocity_ = sync_velocity_buff_.front();

        double diff_gnss_time = current_key_frame_.time - current_key_gnss_.time;
//        double diff_imu_time = current_key_frame_.time - current_sync_imu_.time;
//        double diff_vel_time=current_key_frame_.time-current_sync_velocity_.time;

//        std::cout << "diff gnss time:" << diff_gnss_time << std::endl;
//        std::cout << "diff imu time:" << diff_imu_time << std::endl;
//        std::cout << "diff vel time:" << diff_vel_time << std::endl;

        if (diff_gnss_time < 0
            //        diff_imu_time < 0||
//            diff_vel_time<0
                ) {
            key_frame_buff_.pop_front();
            return false;
        }

        if (diff_gnss_time > 0) {
            key_gnss_buff_.pop_front();
            return false;
        }

//        if (diff_imu_time > 0) {
//            sync_imu_buff_.pop_front();
//            return false;
//        }

//        if (diff_vel_time > 0) {
////            sync_velocity_buff_.pop_front();
//            return false;
//        }

        key_scan_buff_.pop_front();
        key_frame_buff_.pop_front();
        key_gnss_buff_.pop_front();
//        sync_imu_buff_.pop_front();
//        sync_velocity_buff_.pop_front();

        KeyFrameData key_frame_data;
        key_frame_data.time=current_key_frame_.time;
        key_frame_data.index=current_key_frame_.index;
        key_frame_data.frame_pose=current_key_frame_.pose;
        key_frame_data.gnss_pose=current_key_gnss_.pose;
        key_frame_data.cloud_ptr = current_key_scan_.cloud_ptr;

//        key_frame_data.angular_velocity.x = current_sync_velocity_.angular_velocity.x;
//        key_frame_data.angular_velocity.y = current_sync_velocity_.angular_velocity.y;
//        key_frame_data.angular_velocity.z = current_sync_velocity_.angular_velocity.z;
//
//        key_frame_data.linear_velocity.x = current_sync_velocity_.linear_velocity.x;
//        key_frame_data.linear_velocity.y = current_sync_velocity_.linear_velocity.y;
//        key_frame_data.linear_velocity.z = current_sync_velocity_.linear_velocity.z;

        all_key_frame_data_.push_back(key_frame_data);

//        std::cout << "get vaild data" << std::endl;

        return true;
    }

//    //TODO:因为没有初始平移，我们只能补偿旋转
//    bool BackEndOptimizationFlow::AdjustData(bool need_undistortion){
//        // 畸变补偿
//        for (int i = 0; i < all_key_frame_data_.size(); ++i) {
//            VelocityData current_velocity;
//            current_velocity.linear_velocity.x = all_key_frame_data_.at(i).linear_velocity.x;
//            current_velocity.linear_velocity.y = all_key_frame_data_.at(i).linear_velocity.y;
//            current_velocity.linear_velocity.z = all_key_frame_data_.at(i).linear_velocity.z;
//
//            current_velocity.angular_velocity.x = all_key_frame_data_.at(i).angular_velocity.x;
//            current_velocity.angular_velocity.y = all_key_frame_data_.at(i).angular_velocity.y;
//            current_velocity.angular_velocity.z = all_key_frame_data_.at(i).angular_velocity.z;
//
//            current_velocity.TransformCoordinate(gnss_to_lidar_);
//
////            std::cout<<"before:"<<*all_key_frame_data_.at(i).cloud_ptr<<std::endl;
//
//            CloudData::CLOUD_PTR currnent_cloud_ptr(new CloudData::CLOUD(*all_key_frame_data_.at(i).cloud_ptr));//深拷贝当前点云，互不影响
////            std::cout << "cloud size:" << currnent_cloud_ptr->size() << std::endl;
////            std::cout<<"after:"<<*currnent_cloud_ptr<<std::endl;
//
//            if (need_undistortion) {
//                back_end_optimization_ptr_->UnDistortion(current_velocity, currnent_cloud_ptr);
//            }
//
//            back_end_optimization_ptr_->AddToGlobalMap(currnent_cloud_ptr, all_key_frame_data_.at(i).frame_pose);
//        }
//
//        has_new_global_map_=true;
////        back_end_optimization_ptr_->SaveGlobalMap();
//        std::cout << "finish global map" << std::endl;
//        return true;
//    }
//    bool BackEndOptimizationFlow::HandEyeCalibration() {
//        PublishGlobalMap();
//        static bool has_optimized = false;
//        if (has_optimized) {
//            AdjustData(false);
//            has_optimized = true;
//        }else{
//            AdjustData(true);
//        }
////        std::cout << "start calib" << std::endl;
//        for (int j = 0; j < all_key_frame_data_.size(); ++j) {
//
//            static bool has_pose_inited = false;
//            if (!has_pose_inited) {
//                has_pose_inited = true;
//                back_end_optimization_ptr_->SetInitPoseAndLocalMap(all_key_frame_data_.at(0).frame_pose);
//                continue;
//            }
//
//            back_end_optimization_ptr_->UpdateLaserRelativePose(all_key_frame_data_.at(j).cloud_ptr,
//                                                                all_key_frame_data_.at(j).frame_pose, laser_odometry_,
//                                                                current_lidar_relative_pose_);
////            std::cout << "lidar odom pub　before" << std::endl;
//            back_end_laser_odom_pub_ptr_->Publish(laser_odometry_, all_key_frame_data_.at(j).time);
////            std::cout << "lidar odom pubed" << std::endl;
//
//            back_end_optimization_ptr_->UpdateGNSSRelativePose();
//        }
//
//        return back_end_optimization_ptr_->Solve();
//    }

//    bool BackEndOptimizationFlow::PublishGlobalMap() {
//        if (has_new_global_map_ && global_map_pub_ptr_->HasSubscribers()) {
//
////            CloudData::CLOUD_PTR global_map_ptr(new CloudData::CLOUD());
////            back_end_optimization_ptr_->GetGlobalMap(global_map_ptr);
//            global_map_pub_ptr_->Publish(back_end_optimization_ptr_->GetGlobalMap());
////            std::cout << "pub global map finish." << std::endl;
//            has_new_global_map_=false;
//            return true;
//        }
//
//        return false;
//    }

//    bool BackEndOptimizationFlow::HasInited(void) {
//        return filtering_ptr_->HasInited();
//    }

//    bool BackEndOptimizationFlow::Init() {
//
//    }
    bool BackEndOptimizationFlow::Estimate(){
        static Eigen::Affine3d firstRTKInverse = Eigen::Affine3d::Identity(), firstLidarInverse = Eigen::Affine3d::Identity();
        eigenVector tvecsRtk, rvecsRtk, tvecsLidarl, rvecsLidarl;

        auto t1_it = all_key_frame_data_.begin();
        auto t2_it = all_key_frame_data_.begin();

        bool firstTransform = true;

        for (std::size_t i = 0; i < all_key_frame_data_.size(); ++i, ++t1_it, ++t2_it)
        {
            auto& eigenRTK = *t1_it;
            auto& eigenLidar = *t2_it;
            if (firstTransform)
            {
                firstRTKInverse.matrix() = eigenRTK.gnss_pose.inverse().cast<double>();

                firstLidarInverse.matrix() = eigenLidar.frame_pose.inverse().cast<double>();
                // ROS_INFO("Adding first transformation.");
                firstTransform = false;
            }
            else
            {
                Eigen::Affine3d rtkPoseinFirstTipBase;
                rtkPoseinFirstTipBase.matrix() = firstRTKInverse.matrix() * eigenRTK.gnss_pose.cast<double>();

                Eigen::Affine3d lidarInFirstLidarlBase ;
                lidarInFirstLidarlBase.matrix() = firstLidarInverse.matrix() * eigenLidar.frame_pose.cast<double>();

                rvecsRtk.push_back(eigenRotToEigenVector3dAngleAxis(rtkPoseinFirstTipBase.rotation()));
                tvecsRtk.push_back(rtkPoseinFirstTipBase.translation());

                rvecsLidarl.push_back(eigenRotToEigenVector3dAngleAxis(lidarInFirstLidarlBase.rotation()));
                tvecsLidarl.push_back(lidarInFirstLidarlBase.translation());
                // 前后帧交替
                firstRTKInverse = eigenRTK.gnss_pose.cast<double>().inverse();
                firstLidarInverse = eigenLidar.frame_pose.cast<double>().inverse();

            }
        }

        cicv::HandEyeCalibration calib;
        Eigen::Matrix4d result = gnss_to_lidar_.cast<double>();
//        result.block<3, 1>(0, 3) << 0.8, -0.3, 0.8;
        result.block<3, 1>(0, 3) << 0, 0, 0;
        calib.setVerbose(true);
        calib.estimateHandEyeScrew(rvecsRtk, tvecsRtk, rvecsLidarl, tvecsLidarl, result, false);
        std::cout << "Lidar Frame relative to RTK Frame\n" << result << std::endl;
        Eigen::Affine3d resultAffine(result);

        Eigen::Vector3d xyz = resultAffine.translation();
        std::cout << "Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
        Eigen::Quaterniond quaternionResult(resultAffine.rotation());
        std::stringstream ss;
        ss << quaternionResult.w() << ", " << quaternionResult.x() << ", " << quaternionResult.y() << ", "
           << quaternionResult.z() << std::endl;
        std::cout << "Rotation (w,x,y,z): " << ss.str() << std::endl;

        Eigen::Affine3d resultAffineInv = resultAffine.inverse();
        xyz = resultAffineInv.translation();
        std::cout << "Inverted Translation (x,y,z) : " << xyz(0) << ", " << xyz(1) << ", " << xyz(2) << std::endl;
        quaternionResult = Eigen::Quaterniond(resultAffineInv.rotation());
        std::cerr << "Inverted rotation (w,x,y,z): " << quaternionResult.w() << " " << quaternionResult.x() << " "
                  << quaternionResult.y() << " " << quaternionResult.z() << std::endl;
//        return resultAffine;
        return true;

    }

}
