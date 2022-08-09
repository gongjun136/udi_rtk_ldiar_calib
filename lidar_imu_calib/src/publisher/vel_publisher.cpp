////
//// Created by gj on 2022/7/26.
////
//
//#include "lidar_imu_calib/publisher/vel_publisher.hpp"
//
//namespace lidar_imu_calib {
//
//    VelPublisher::VelPublisher(
//            ros::NodeHandle& nh,
//            std::string topic_name,
//            std::string base_frame_id,
//            std::string child_frame_id,
//            int buff_size
//    )
//            :nh_(nh) {
//
//        publisher_ = nh_.advertise<PosVel>(topic_name, buff_size);
//        pos_vel_msg_.header.frame_id = base_frame_id;
//        pos_vel_msg_.child_frame_id = child_frame_id;
//    }
//
//    void VelPublisher::Publish(
//            const PosVelData &pos_vel_data,
//            const double &time
//    ) {
//        ros::Time ros_time(time);
//        PublishData(pos_vel_data, ros_time);
//    }
//
//    void VelPublisher::Publish(
//            const PosVelData &pos_vel_data
//    ) {
//        PublishData(pos_vel_data, ros::Time::now());
//    }
//
//    bool VelPublisher::HasSubscribers() {
//        return publisher_.getNumSubscribers() != 0;
//    }
//
//    void PosVelPublisher::PublishData(
//            const PosVelData &pos_vel_data,
//            ros::Time time
//    ) {
//        pos_vel_msg_.header.stamp = time;
//
//        // a. set position
//        pos_vel_msg_.position.x = pos_vel_data.pos.x();
//        pos_vel_msg_.position.y = pos_vel_data.pos.y();
//        pos_vel_msg_.position.z = pos_vel_data.pos.z();
//
//        // b. set velocity:
//        pos_vel_msg_.velocity.x = pos_vel_data.vel.x();
//        pos_vel_msg_.velocity.y = pos_vel_data.vel.y();
//        pos_vel_msg_.velocity.z = pos_vel_data.vel.z();
//
//        publisher_.publish(pos_vel_msg_);
//    }
//
//}