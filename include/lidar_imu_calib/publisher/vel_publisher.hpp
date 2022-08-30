////
//// Created by gj on 2022/7/26.
////
//
//#ifndef CATKIN_WS_UDI_CALIB_VEL_PUBLISHER_HPP
//#define CATKIN_WS_UDI_CALIB_VEL_PUBLISHER_HPP
//#include <string>
//
//#include <Eigen/Dense>
//
//#include <ros/ros.h>
//
//#include "lidar_imu_calib/sensor_data/Velocity.hpp"
//
//#include "lidar_imu_calib/Vel.h"
//
//namespace lidar_imu_calib {
//
//    class VelPublisher {
//    public:
//        VelPublisher(
//                ros::NodeHandle& nh,
//                std::string topic_name,
//                std::string base_frame_id,
//                std::string child_frame_id,
//                int buff_size
//        );
//        VelPublisher() = default;
//
//        void Publish(const PosVelData &pos_vel_data, const double &time);
//        void Publish(const PosVelData &pos_vel_data);
//
//        bool HasSubscribers();
//
//    private:
//        void PublishData(
//                const PosVelData &pos_vel_data,
//                ros::Time time
//        );
//
//    private:
//        ros::NodeHandle nh_;
//        ros::Publisher publisher_;
//        PosVel pos_vel_msg_;
//    };
//
//}
//#endif //CATKIN_WS_UDI_CALIB_VEL_PUBLISHER_HPP
