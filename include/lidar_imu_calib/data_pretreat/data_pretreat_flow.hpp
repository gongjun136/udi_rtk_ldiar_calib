//
// Created by udi on 2022/7/25.
//

#ifndef CATKIN_WS_UDI_CALIB_DATA_PRETREAT_FLOW_HPP
#define CATKIN_WS_UDI_CALIB_DATA_PRETREAT_FLOW_HPP
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
// subscriber
#include "lidar_imu_calib/subscriber/cloud_subscriber.hpp"
#include "lidar_imu_calib/subscriber/imu_subscriber.hpp"
#include "lidar_imu_calib/subscriber/velocity_subscriber.hpp"
#include "lidar_imu_calib/subscriber/gnss_subscriber.hpp"
// publisher
// a. synced lidar measurement
#include "lidar_imu_calib/publisher/cloud_publisher.hpp"
// b. synced IMU measurement
#include "lidar_imu_calib/publisher/imu_publisher.hpp"
// c. synced GNSS-odo measurement:
#include "lidar_imu_calib/publisher/pos_vel_publisher.hpp"
// d. synced reference trajectory:
#include "lidar_imu_calib/publisher/odometry_publisher.hpp"


namespace lidar_imu_calib {
    class DataPretreatFlow {
    public:
        /**
         * @brief 构造函数，初始话订阅者和发布者
         * @param nh
         * @param cloud_topic
         */
        DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic);
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
         * @brief 初始化订阅者
         * @param nh
         * @param config_node
         * @return
         */
        bool InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node);
        /**
         * @brief 初始化gnss里程计原点
         * @return
         */
        bool InitGNSS();
        /**
         * @brief 判断是否有数据，并对齐时间戳
         * @return
         */
        bool HasData();
        /**
         * @brief 判断数据是否有效
         * @return
         */
        bool ValidData();
        /**
         * @brief　得到gnss里程计
         * @return
         */
        bool TransformData();
        /**
         * @brief 发布对齐时间戳后的所有数据
         * @return
         */
        bool PublishData();

    private:
        // 订阅者
        std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
        std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
        std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
        std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;

        // 发布者
        std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
        std::shared_ptr<IMUPublisher> imu_pub_ptr_;
//        ros::Publisher vel_pub_;
        std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;

        // 数据缓存区
        std::deque<CloudData> cloud_data_buff_;
        std::deque<IMUData> imu_data_buff_;
        std::deque<VelocityData> velocity_data_buff_;
        std::deque<GNSSData> gnss_data_buff_;

        //　当前数据
        CloudData current_cloud_data_;
        IMUData current_imu_data_;
        VelocityData current_velocity_data_;
        GNSSData current_gnss_data_;
        // 当前gnss里程计
        Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
    };
}
#endif //CATKIN_WS_UDI_CALIB_DATA_PRETREAT_FLOW_HPP
