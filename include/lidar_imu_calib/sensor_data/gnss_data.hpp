/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月19日 
*/

#ifndef CATKIN_WS_UDI_CALIB_GNSS_DATA_HPP
#define CATKIN_WS_UDI_CALIB_GNSS_DATA_HPP
#include <deque>

#include <GeographicLib/LocalCartesian.hpp>

namespace lidar_imu_calib {
    class GNSSData {
    public:
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        int status = 0;
        int service = 0;

        static double origin_longitude;
        static double origin_latitude;
        static double origin_altitude;

    private:
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;

    public:
        void InitOriginPosition();
        void UpdateXYZ();
        static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time);
    };
}
#endif //CATKIN_WS_UDI_CALIB_GNSS_DATA_HPP
