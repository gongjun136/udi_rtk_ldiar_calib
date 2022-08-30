/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月16日 
*/

#ifndef CATKIN_WS_UDI_CALIB_VELOCITY_DATA_HPP
#define CATKIN_WS_UDI_CALIB_VELOCITY_DATA_HPP
#include <deque>
#include <Eigen/Dense>

namespace lidar_imu_calib {
    class VelocityData {
    public:
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

        double time = 0.0;
        LinearVelocity linear_velocity;
        AngularVelocity angular_velocity;

    public:
        static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);
        void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    };
}
#endif //CATKIN_WS_UDI_CALIB_VELOCITY_DATA_HPP
