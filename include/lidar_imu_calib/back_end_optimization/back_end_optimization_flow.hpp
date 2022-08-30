//
// Created by gj on 2022/7/28.
//

#ifndef CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_FLOW_HPP
#define CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_FLOW_HPP

#include "lidar_imu_calib/subscriber/cloud_subscriber.hpp"
#include "lidar_imu_calib/subscriber/odometry_subscriber.hpp"
#include "lidar_imu_calib/subscriber/tf_listener.hpp"
#include "lidar_imu_calib/subscriber/velocity_subscriber.hpp"
#include "lidar_imu_calib/subscriber/imu_subscriber.hpp"
#include "lidar_imu_calib/subscriber/key_frame_subscriber.hpp"
#include "lidar_imu_calib/publisher/cloud_publisher.hpp"
#include "lidar_imu_calib/publisher/odometry_publisher.hpp"
#include "lidar_imu_calib/models/hand_eye_calibration/hand_eye_calibration.hpp"

#include "lidar_imu_calib/back_end_optimization/back_end_optimization.hpp"
namespace lidar_imu_calib{
    class BackEndOptimizationFlow {
    public:
        typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> eigenVector;

        /**
         * @brief
         * @tparam Input
         * @param eigenQuat
         * @return
         */
        template <typename Input>
        Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat)
        {
            Eigen::AngleAxisd ax3d(eigenQuat);
            return ax3d.angle() * ax3d.axis();
        }

        // 关键帧数据
        struct KeyFrameData{
            struct LinearVelocity {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
            };

            struct AngularVelocity {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
            };

            CloudData::CLOUD_PTR cloud_ptr;
            double time = 0.0;
            unsigned int index = 0;
            Eigen::Matrix4f frame_pose = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f gnss_pose = Eigen::Matrix4f::Identity();
            LinearVelocity linear_velocity;
            AngularVelocity angular_velocity;
        };
    public:
        /**
         * @brief 构造函数，初始化订阅者和发布者
         * @param nh
         */
        BackEndOptimizationFlow(ros::NodeHandle& nh);
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
         * @brief 接收初始旋转外参
         * @return
         */
        bool InitCalibration();
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
//        /**
//         * @brief
//         * @param need_undistortion
//         * @return
//         */
//        bool AdjustData(bool need_undistortion=false);

//        bool PublishGlobalMap();
//
//        bool HandEyeCalibration();
        /**
         * @brief 手眼标定，优化得到外参
         * @return
         */
        bool Estimate();
        /**
         * @brief 初始化订阅者
         * @param nh
         * @param config_node
         * @return
         */
        bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);

    private:
//        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
//        std::deque<CloudData> cloud_data_buff_;
//        GNSS
//        std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
//        std::deque<PoseData> gnss_data_buff_;
        // 订阅者
        std::shared_ptr<TFListener> gnss2lidar_tf_sub_ptr_;
//        std::shared_ptr<VelocitySubscriber> sync_velocity_sub_ptr_;//同步后的速度信息
//        std::shared_ptr<IMUSubscriber> sync_imu_sub_ptr_;
        std::shared_ptr<CloudSubscriber> key_scan_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
        std::shared_ptr<KeyFrameSubscriber> key_gnss_sub_ptr_;

//        std::shared_ptr<OdometryPublisher> back_end_laser_odom_pub_ptr_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

//        std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
        // 数据缓存区
        std::deque<CloudData> key_scan_buff_;
        std::deque<KeyFrame> key_frame_buff_;
        std::deque<KeyFrame> key_gnss_buff_;
//        std::deque<IMUData> sync_imu_buff_;
//        std::deque<VelocityData> sync_velocity_buff_;
        // 当前数据
        CloudData current_key_scan_;
        KeyFrame current_key_frame_;
        KeyFrame current_key_gnss_;
//        IMUData current_sync_imu_;
//        VelocityData current_sync_velocity_;

//        CloudData::CLOUD_PTR global_map_ptr_;

//        std::shared_ptr<BackEndOptimization> back_end_optimization_ptr_;

        // 外参
        Eigen::Matrix4f gnss_to_lidar_;
        // 所有关键帧数据
        std::deque<KeyFrameData> all_key_frame_data_;

//        bool has_new_global_map_=false;

//        Eigen::Matrix4f laser_odometry_;
//        Eigen::Matrix4f current_lidar_relative_pose_;

    };
}

#endif //CATKIN_WS_UDI_CALIB_BACK_END_OPTIMIZATION_FLOW_HPP
