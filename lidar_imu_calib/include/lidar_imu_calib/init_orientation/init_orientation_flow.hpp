//
// Created by gj on 2022/7/26.
//

#ifndef CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_FLOW_HPP
#define CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_FLOW_HPP

#include "lidar_imu_calib/subscriber/cloud_subscriber.hpp"
#include "lidar_imu_calib/subscriber/odometry_subscriber.hpp"

#include "lidar_imu_calib/publisher/odometry_publisher.hpp"
#include "lidar_imu_calib/publisher/key_frame_publisher.hpp"
#include "lidar_imu_calib/publisher/cloud_publisher.hpp"
#include "lidar_imu_calib/publisher/tf_broadcaster.hpp"
#include "glog/logging.h"
#include "Eigen/Core"

#include "lidar_imu_calib/init_orientation/init_orientation.hpp"

namespace lidar_imu_calib{
    class InitOrientationFlow{
    public:
        /**
         * @brief 构造函数，初始化订阅者和发布者，以及核心算法指针
         * @param nh
         */
        InitOrientationFlow(ros::NodeHandle &nh);
        /**
         * @brief 主函数
         * @return
         */
        bool Run();

    private:
        /**
         * @brief 读取数据
         * @return
         */
        bool ReadData();
        /**
         * @brief 判断是否有数据
         * @return
         */
        bool HasData();
        /**
         * @brief 判断数据是否有效
         * @return
         */
        bool ValidData();

        /**
         * @brief　svd计算初始旋转
         * @return
         */
        bool GetInitOrientation();
        /**
         * @brief 更新gnss相对位姿
         * @return
         */
        bool UpdateGNSSRelativePose();
        /**
         * @brief 更新雷达相对位姿
         * @return
         */
        bool UpdateLaserRelativePose();
        /**
         * @brief svd计算初始旋转外参
         * @return
         */
        bool Solve();
        /**
         * @brief 发布数据，关键帧对应的点云、位姿、gnss位姿
         * @return
         */
        bool PublishData();
        /**
         * @brief 发布初始旋转外参
         * @return
         */
        bool PublishInitOrientation();
    private:
        // 订阅者
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;

        // 数据缓存区
        std::deque<CloudData> cloud_data_buff_;
        std::deque<PoseData> gnss_data_buff_;
        // 发布者
        std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
        std::shared_ptr<CloudPublisher> key_scan_pub_ptr_;
        std::shared_ptr<TFBroadCaster> tf_imu2lidar_pub_ptr_;
        std::shared_ptr<KeyFramePublisher> key_gnss_pub_ptr_;
        // 核心算法指针
        std::shared_ptr<InitOrientation> init_orientation_ptr_;
        // 当前数据
        CloudData current_cloud_data_;
        PoseData current_gnss_data_;
        Eigen::Matrix4f laser_odometry_;
        Eigen::Matrix4f current_gnss_relative_pose_;
        Eigen::Matrix4f current_lidar_relative_pose_;
        // 标识符，是否有新的关键帧
        bool has_new_key_frame_=false;

        // 初始旋转外参
        Eigen::Matrix3d init_orientation_;
        // Ax=b的A矩阵
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> A_vec_;
        // 初始化时间
        double init_time_=-1;
        //截取数据集后的有效时间段:ros系统时间
        double start_time_=-1;
        double end_time_=-1;

    };
}

#endif //CATKIN_WS_UDI_CALIB_INIT_ORIENTATION_FLOW_HPP
