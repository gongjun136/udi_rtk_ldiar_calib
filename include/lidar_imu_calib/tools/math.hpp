/** 
* @Description: 
* @Author: gj
* @Date: 2022年07月12日 
*/

#ifndef CATKIN_WS_UDI_CALIB_MATH_HPP
#define CATKIN_WS_UDI_CALIB_MATH_HPP
namespace lidar_imu_calib{
    static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();

    //旋转矩阵转为欧拉角
    inline Eigen::Vector3f R2ypr(const Eigen::Matrix3f &R)
    {
        Eigen::Vector3f n = R.col(0);
        Eigen::Vector3f o = R.col(1);
        Eigen::Vector3f a = R.col(2);

        Eigen::Vector3f ypr;
        float y = atan2(n(1), n(0));
        float p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
        float r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
        ypr(0) = y;
        ypr(1) = p;
        ypr(2) = r;

        return ypr / M_PI * 180.0;
    }
    // 控制旋转角度在[-180,180]
    inline double normalize_angle(double ang_degree) {
        if(ang_degree > 180)
            ang_degree -= 360;

        if(ang_degree < -180)
            ang_degree += 360;
        return ang_degree;
    }
    /**
     * @brief 去反对称矩阵
     * @tparam Derived 矩阵数据类型
     * @param v3d
     * @return
     */
    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
        m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
                v3d.z(), typename Derived::Scalar(0), -v3d.x(),
                -v3d.y(), v3d.x(), typename Derived::Scalar(0);
        return m;
    }
    /**
     * @brief   构建左乘矩阵
     * @tparam Derived 矩阵数据类型
     * @param q
     * @return
     */
    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q) {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
        typename Derived::Scalar q4 = q.w();
        m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
        m.block(3, 0, 1, 3) << -vq.transpose();
        m.block(0, 3, 3, 1) << vq;
        m(3, 3) = q4;
        return m;
    }
    /**
     * @brief 构建右乘矩阵
     * @tparam Derived
     * @param p
     * @return
     */
    template<typename Derived>
    inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p) {
        Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
        Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
        typename Derived::Scalar p4 = p.w();
        m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
        m.block(3, 0, 1, 3) << -vp.transpose();
        m.block(0, 3, 3, 1) << vp;
        m(3, 3) = p4;
        return m;
    }


}
#endif //CATKIN_WS_UDI_CALIB_MATH_HPP
