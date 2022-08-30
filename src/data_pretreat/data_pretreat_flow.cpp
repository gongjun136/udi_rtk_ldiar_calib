/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#include "lidar_imu_calib/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_imu_calib/global_defination/global_defination.h"

namespace lidar_imu_calib {
    DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string cloud_topic) {
        std::string config_file_path = WORK_SPACE_PATH + "/config/config.yaml";
//        std::string config_file_path = "/home/gj/catkin_ws_udi_calib/src/lidar_imu_calib/config/config.yaml";

        std::cout << config_file_path << std::endl;
        const YAML::Node config_node = YAML::LoadFile(config_file_path);
        InitSubscribers(nh, config_node["measurements"]);

//        lidar_frequency_ = config_node["measurements"]["lidar"]["frequency"].as<int>();
        // publishers:
        cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, "/velo_link", 100);
        imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", "/imu_link", 100);
//        pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", "/imu_link", 100);
        gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", "/velo_link", 100);

//        vel_pub_ptr_=std::make_shared<ros::Publisher>();
//        vel_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/synced_vel", 100);

        // motion compensation for lidar measurement:
//        distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
    }

    bool DataPretreatFlow::Run() {
        if (!ReadData())
            return false;

        if (!InitGNSS())
            return false;

        while (HasData()) {
            if (!ValidData())
                continue;

            TransformData();
            PublishData();
        }

        return true;
    }
    bool DataPretreatFlow::InitSubscribers(ros::NodeHandle& nh, const YAML::Node& config_node){

        cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, config_node["lidar"]["topic_name"].as<std::string>(),
                                                           config_node["lidar"]["queue_size"].as<int>());
        imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, config_node["imu"]["topic_name"].as<std::string>(),
                                                       config_node["imu"]["queue_size"].as<int>());
        gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, config_node["gnss"]["topic_name"].as<std::string>(),
                                                         config_node["gnss"]["queue_size"].as<int>());
//        velocity_sub_ptr_=std::make_shared<VelocitySubscriber>(nh, config_node["velocity"]["topic_name"].as<std::string>(),
//                                                               config_node["velocity"]["queue_size"].as<int>());
        return true;
    }

    bool DataPretreatFlow::ReadData() {
        static std::deque<IMUData> unsynced_imu_;
//        static std::deque<VelocityData> unsynced_velocity_;
        static std::deque<GNSSData> unsynced_gnss_;

        // fetch lidar measurements from buffer:
        cloud_sub_ptr_->ParseData(cloud_data_buff_);
        imu_sub_ptr_->ParseData(unsynced_imu_);
//        velocity_sub_ptr_->ParseData(unsynced_velocity_);
        gnss_sub_ptr_->ParseData(unsynced_gnss_);
        // 因为gnss频率低，所以需要保证size
        if (unsynced_gnss_.size() < 10) {
            return false;
        }
        if (unsynced_imu_.size() < 10) {
            return false;
        }

        if (cloud_data_buff_.size() == 0){
            return false;}

        // use timestamp of lidar measurement as reference:
        double cloud_time = cloud_data_buff_.front().time;
        // sync IMU, velocity and GNSS with lidar measurement:
        // find the two closest measurement around lidar measurement time
        // then use linear interpolation to generate synced measurement:
        bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
//        bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
        bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

        // only mark lidar as 'inited' when all the three sensors are synced:
        static bool sensor_inited = false;
        if (!sensor_inited) {
            if (!valid_imu ||
                //                !valid_velocity ||
                !valid_gnss) {
//                std::cout << "invalid data" << std::endl;
                cloud_data_buff_.pop_front();
                return false;
            }
            sensor_inited = true;
        }
//        std::cout << "get valid data" << std::endl;
        return true;
    }
    bool DataPretreatFlow::InitGNSS() {
        static bool gnss_inited = false;
        if (!gnss_inited) {
            GNSSData gnss_data = gnss_data_buff_.front();
            gnss_data.InitOriginPosition();
            gnss_inited = true;
//            init_time_=gnss_data.time;
            std::cout << "gnss 初始原点完成，原点坐标：" << std::endl << GNSSData::origin_altitude << " ,"
                      << GNSSData::origin_latitude << " ," << GNSSData::origin_longitude << std::endl;
        }

        return gnss_inited;
    }

    bool DataPretreatFlow::HasData() {
        if (cloud_data_buff_.size() == 0)
            return false;
        if (imu_data_buff_.size() == 0)
            return false;
//        if (velocity_data_buff_.size() == 0)
//            return false;
        if (gnss_data_buff_.size() == 0)
            return false;

        return true;
    }

    bool DataPretreatFlow::ValidData() {
        current_cloud_data_ = cloud_data_buff_.front();
        current_imu_data_ = imu_data_buff_.front();
//        current_velocity_data_ = velocity_data_buff_.front();
        current_gnss_data_ = gnss_data_buff_.front();

        double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
//        double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
        double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;

//        double max_diff_time = 0.5 / lidar_frequency_;
        if (diff_imu_time < 0 ||
            //            diff_velocity_time < 0 ||
            diff_gnss_time < 0) {
            cloud_data_buff_.pop_front();
            return false;
        }

        if (diff_imu_time > 0) {
            imu_data_buff_.pop_front();
            return false;
        }

//        if (diff_velocity_time > 0) {
//            velocity_data_buff_.pop_front();
//            return false;
//        }

        if (diff_gnss_time > 0) {
            gnss_data_buff_.pop_front();
            return false;
        }

        cloud_data_buff_.pop_front();
        imu_data_buff_.pop_front();
//        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
//        std::cout << "get sync data" << std::endl;
        return true;
    }


    bool DataPretreatFlow::TransformData() {
        // get GNSS & IMU pose prior:
        gnss_pose_ = Eigen::Matrix4f::Identity();
        // a. get position from GNSS
        current_gnss_data_.UpdateXYZ();
        gnss_pose_(0,3) = current_gnss_data_.local_E;
        gnss_pose_(1,3) = current_gnss_data_.local_N;
        gnss_pose_(2,3) = current_gnss_data_.local_U;
        // b. get orientation from IMU:
//        static bool is_first_gnss_pose= true;
//        static Eigen::Matrix3f init_gnss_pose;
//        if (is_first_gnss_pose) {
//            //init_gnss_ori = current_imu_data_.GetOrientationMatrix();
//            gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
//            init_gnss_pose = current_imu_data_.GetOrientationMatrix();
//            gnss_pose_.block<3, 3>(0, 0) = Eigen::Matrix3f::Identity();
//            is_first_gnss_pose=false;
//            return true;
//        }
//        gnss_pose_.block<3, 3>(0, 0) =
//                current_imu_data_.GetOrientationMatrix().inverse()*init_gnss_pose;
//        Eigen::AngleAxisf pre_T(M_PI, Eigen::Vector3f(0, 1, 0));

//        gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix() * pre_T;


//        Eigen::AngleAxisf yaw_angle(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));
//        Eigen::Vector3f point(gnss_pose_(0, 3), gnss_pose_(1, 3), gnss_pose_(2, 3));
//        point = yaw_angle * point;
//        gnss_pose_(0, 3) = point.x();
//        gnss_pose_(1, 3) = point.y();
//        gnss_pose_(2, 3) = point.z();

        gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();

//        std::cout << "gnss pose" << std::endl << gnss_pose_ << std::endl;
        // b. set synced pos vel
//        pos_vel_.pos.x() = current_gnss_data_.local_E;
//        pos_vel_.pos.y() = current_gnss_data_.local_N;
//        pos_vel_.pos.z() = current_gnss_data_.local_U;
//
//        pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
//        pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
//        pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

//        vel_->twist.linear.x=current_velocity_data_.linear_velocity.x;
//        vel_->twist.linear.y=current_velocity_data_.linear_velocity.y;
//        vel_->twist.linear.z=current_velocity_data_.linear_velocity.z;
//        vel_->twist.angular.x=current_imu_data_.angular_velocity.x;
//        vel_->twist.angular.y=current_imu_data_.angular_velocity.y;
//        vel_->twist.angular.z=current_imu_data_.angular_velocity.z;
        return true;
    }
    bool DataPretreatFlow::PublishData() {
        cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
        gnss_pub_ptr_->Publish(gnss_pose_, current_cloud_data_.time);
        imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);

//        geometry_msgs::TwistStamped vel;
//        vel.header.stamp=ros::Time(current_cloud_data_.time);
//        vel.header.frame_id = "/map";
//        vel.twist.linear.x=current_velocity_data_.linear_velocity.x;
//        vel.twist.linear.y=current_velocity_data_.linear_velocity.y;
//        vel.twist.linear.z=current_velocity_data_.linear_velocity.z;
//        vel.twist.angular.x=current_imu_data_.angular_velocity.x;
//        vel.twist.angular.y=current_imu_data_.angular_velocity.y;
//        vel.twist.angular.z=current_imu_data_.angular_velocity.z;
//        vel_pub_.publish(vel);

//        pos_vel_pub_ptr_->Publish(pos_vel_, current_cloud_data_.time);
        return true;
    }
}
