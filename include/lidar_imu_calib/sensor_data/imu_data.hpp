/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月07日 
*/

#ifndef CATKIN_WS_LIDAR_IMU_CALIB_IMU_DATA_HPP
#define CATKIN_WS_LIDAR_IMU_CALIB_IMU_DATA_HPP
#include <deque>
#include <cmath>
#include <Eigen/Dense>

namespace lidar_imu_calib {
    class IMUData {
    public:
        struct LinearAcceleration {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct AngularVelocity {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        class Orientation {
        public:
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double w = 0.0;

        public:
            void Normlize() {
                double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
                x /= norm;
                y /= norm;
                z /= norm;
                w /= norm;
            }
        };

        double time = 0.0;
        LinearAcceleration linear_acceleration;
        AngularVelocity angular_velocity;
        Orientation orientation;

    public:
        // 把四元数转换成旋转矩阵送出去
        Eigen::Matrix3f GetOrientationMatrix();
        //同步时间相邻的左右两个数据做线性插值
        static bool SyncData(std::deque<IMUData>& UnsyncedData, std::deque<IMUData>& SyncedData, double sync_time) ;

        static IMUData Interpolation(IMUData front_data, IMUData back_data, double sync_time);
    };
}
#endif //CATKIN_WS_LIDAR_IMU_CALIB_IMU_DATA_HPP
