//
// Created by gj on 2022/7/26.
//

#include "lidar_imu_calib/init_orientation/init_orientation_flow.hpp"
#include "lidar_imu_calib/global_defination/global_defination.h"
#include "yaml-cpp/yaml.h"
#include <ros/ros.h>
#include "lidar_imu_calib/tools/math.hpp"

namespace lidar_imu_calib{
    InitOrientationFlow::InitOrientationFlow(ros::NodeHandle &nh) {
        //初始化订阅者：
        std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
        const YAML::Node config_node = YAML::LoadFile(config_file_path);
        // 初始化订阅者
//        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node["measurements"]["lidar"]["topic_name"].as<std::string>(),
//                                                           config_node["measurements"]["lidar"]["queue_size"].as<int>());
        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/synced_cloud", 100000);
        gnss_sub_ptr_ =
                std::make_shared<OdometrySubscriber>(nh, "/synced_gnss", 100000);


        laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);

        tf_imu2lidar_pub_ptr_ = std::make_shared<TFBroadCaster>("gnss", "lidar");



        key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_frame", "/map", 100);

        key_scan_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/key_scan", "/lidar", 100);

        key_gnss_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "/key_gnss", "/map", 100);
        // 核心算法
        init_orientation_ptr_ = std::make_shared<InitOrientation>();
    }

    bool InitOrientationFlow::Run() {
        static bool has_init_orientation=false;
        if(!has_init_orientation){
            if (!ReadData()) {
                return false;
            }
            while (HasData()) {
                if (!ValidData()) {
                    return false;
                }

                if (GetInitOrientation()) {
                    has_init_orientation=true;
                }
                PublishData();

            }
            return false;
        }else{
            PublishInitOrientation();
        }

        return true;
    }

    bool InitOrientationFlow::ReadData(){
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        gnss_sub_ptr_->ParseData(gnss_data_buff_);

        return true;
    }
    bool InitOrientationFlow::HasData() {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (gnss_data_buff_.size() == 0)
            return false;
//        std::cout << "has data" << std::endl;

        return true;
    }
    bool InitOrientationFlow::ValidData(){
        current_cloud_data_ = cloud_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();

        double diff_gnss_time =
                current_cloud_data_.time - current_gnss_data_.time;

        if (diff_gnss_time < 0) {
            cloud_data_buff_.pop_front();
//            std::cout << "diff gnss time:" << diff_gnss_time << std::endl;
            return false;
        }

        if (diff_gnss_time > 0) {
            gnss_data_buff_.pop_front();
//            std::cout << "diff gnss time:" << diff_gnss_time << std::endl;
            return false;
        }


        cloud_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

//        std::cout << "get vaild data,data time:" << current_cloud_data_.time << std::endl;

        return true;
    }

    bool InitOrientationFlow::GetInitOrientation(){
        if (current_cloud_data_.time < end_time_ && current_cloud_data_.time > start_time_) {
//        std::cout << "laser odom:" << std::endl << laser_odometry_ << std::endl;
            laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.time);
        }
        if (UpdateLaserRelativePose()) {
            has_new_key_frame_=init_orientation_ptr_->HasNewKeyFrame();
            if (has_new_key_frame_) {
                if (UpdateGNSSRelativePose()) {
                    if (current_cloud_data_.time < end_time_ && current_cloud_data_.time > start_time_) {
                        return Solve();
                    }
                }
            }

        }


        return false;

    }
    bool InitOrientationFlow::UpdateGNSSRelativePose(){
        static bool is_first_gnss=true;
        static Eigen::Matrix4f last_gnss_pose;
        if(is_first_gnss){
            is_first_gnss=false;
            last_gnss_pose = current_gnss_data_.pose;
//            std::cout << "init pose:" << std::endl << current_gnss_data_.pose << std::endl;
            return false;
        }
        current_gnss_relative_pose_ = last_gnss_pose.inverse() * current_gnss_data_.pose;
        last_gnss_pose=current_gnss_data_.pose;

        return true;
    }

    bool InitOrientationFlow::UpdateLaserRelativePose() {
        has_new_key_frame_=false;
        static bool has_pose_inited = false;
        if (!has_pose_inited) {
            has_pose_inited = true;
            init_orientation_ptr_->SetInitPose(current_gnss_data_);
            //计算数据截取开始时间和结束时间
            init_time_ = current_gnss_data_.time;
            init_orientation_ptr_->GetBeginEndTime(start_time_,end_time_);
//            start_time_ += init_time_;
//            end_time_ += init_time_;
            std::cout <<std::setprecision(15)<< "数据初始化时间：" << init_time_ << std::endl;
            std::cout << "数据起始时间：" << start_time_ << std::endl;
            std::cout << "数据结束时间：" << end_time_ << std::endl;
            return init_orientation_ptr_->Update(current_cloud_data_, laser_odometry_,current_lidar_relative_pose_);
        }

//        laser_odometry_ = Eigen::Matrix4f::Identity();
        return init_orientation_ptr_->Update(current_cloud_data_, laser_odometry_, current_lidar_relative_pose_);
    }

    bool InitOrientationFlow::Solve(){

        Eigen::Quaterniond delta_qij_lidar(current_lidar_relative_pose_.cast<double>().block<3, 3>(0, 0));
        Eigen::Quaterniond delta_qij_gnss(current_gnss_relative_pose_.cast<double>().block<3,3>(0,0));

//        std::cout << "lidar:" << std::endl << current_lidar_relative_pose_ << std::endl;
//        std::cout << "gnss:" << std::endl << current_gnss_relative_pose_ << std::endl;

        // 计算imu相对旋转和lidar里程计相对旋转的角度变化差值
        Eigen::AngleAxisd R_vector1(current_lidar_relative_pose_.cast<double>().block<3, 3>(0, 0));
        Eigen::AngleAxisd R_vector2(current_gnss_relative_pose_.cast<double>().block<3,3>(0,0));
        //delta_angle是角度偏差，huber是权重，角度偏差越大，权重就越小，启发式方式来用于抑制异常值
        double delta_angle = 180 / M_PI * std::fabs(R_vector1.angle() - R_vector2.angle());
        double huber = delta_angle > 1.0 ? 1.0/delta_angle : 1.0;

        Eigen::Matrix4d lq_mat = LeftQuatMatrix(delta_qij_lidar);
        Eigen::Matrix4d rq_mat = RightQuatMatrix(delta_qij_gnss);
        A_vec_.push_back(huber * (lq_mat - rq_mat));

        //超定方程，方程数必须大于15
        size_t valid_size = A_vec_.size();

        if (valid_size < init_orientation_ptr_->GetKeyFrameNum()) {
            return false;
        }
//
        Eigen::MatrixXd A(valid_size * 4, 4);
        for (size_t i = 0; i < valid_size; ++i)
            A.block<4, 4>(i * 4, 0) = A_vec_.at(i);
        //SVD求解超定方程
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Vector4d x = svd.matrixV().col(3);
//        std::cout << "V：" <<std::endl<< svd.matrixV() << std::endl;
////        std::cout << "V：" <<std::endl<< svd. << std::endl;
//        std::cout << "初始旋转解：" <<std::endl<< x << std::endl;
        Eigen::Quaterniond q_gnss2lidar(x);

        Eigen::Vector4d cov = svd.singularValues();
        std::cout << "奇异值向量：" <<std::endl<< cov << std::endl;

        //　奇异值正比于方程数，此处奇异值阈值调参得到，用于限制足够的方程数
//        if (cov(2) > 0.25) {
//        init_orientation_ = q_gnss2lidar.inverse().matrix();
        init_orientation_ = q_gnss2lidar.matrix();


//        has_init_orientation_=true;

//        std::cout << "初始化旋转外参成功：" << std::endl << (init_orientation_.eulerAngles(2, 1, 0) * 180 / M_PI).transpose()
//                  << std::endl;
        std::cout << "初始化旋转外参成功：" << std::endl << (init_orientation_.eulerAngles(2,1,0)* 180 / M_PI).transpose()
                  << std::endl;
//        std::cout << "四元数形式：" << std::endl << Eigen::Quaterniond(init_orientation_).coeffs().transpose() << std::endl;
            //真值：
        Eigen::Quaterniond q0;
        q0.x()=-0.00741208800464;
        q0.y()=0.00101508648626;
        q0.z()=0.000385188259497;
        q0.w()=0.999971937106;
        std::cout << "真值为：" << std::endl
                  << (q0.toRotationMatrix().eulerAngles(2,1,0) * 180 / M_PI).transpose() << std::endl;

            return true;
//        } else {
//            return false;
//        }
    }

    bool InitOrientationFlow::PublishInitOrientation(){
        //发布初始外参
        Eigen::Matrix4f gnss2lidar = Eigen::Matrix4f::Identity();
        gnss2lidar.block<3, 3>(0, 0) = init_orientation_.cast<float>();
        tf_imu2lidar_pub_ptr_->SendTransform(gnss2lidar, current_cloud_data_.time);

    }

    bool InitOrientationFlow::PublishData(){
//        if (current_cloud_data_.time < end_time_ && current_cloud_data_.time > start_time_
        if (init_orientation_ptr_->HasNewKeyFrame()&&current_cloud_data_.time < end_time_ && current_cloud_data_.time > start_time_) {
            key_frame_pub_ptr_->Publish(init_orientation_ptr_->GetCurrentKeyFrame());
            key_scan_pub_ptr_->Publish(current_cloud_data_.cloud_ptr,
                                       current_cloud_data_.time);

            KeyFrame current_key_gnss_;
            current_key_gnss_.time = current_gnss_data_.time;
            current_key_gnss_.pose=current_gnss_data_.pose;
            current_key_gnss_.index = init_orientation_ptr_->GetCurrentKeyFrame().index;
            key_gnss_pub_ptr_->Publish(current_key_gnss_);
        }
        return true;

    }

}