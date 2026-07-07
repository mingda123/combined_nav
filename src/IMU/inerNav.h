#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <vector>

#include "fileLoader.h"

#include "COMMON/MyTime.h"
#include "COMMON/navState.h"
#include "COMMON/const_info.h"

// 静止检测默认阈值，仅用于零速判定等简化逻辑
const double STATIC_THRESHOLD_ACCEL = 0.5; // 单位 m/s^2
const double STATIC_THRESHOLD_GYRO = 0.5;  // 单位 rad/s

// 加速度计模型与补偿参数
class Accer
{
private:
    Eigen::Vector3d bias;   // 零偏，单位 m/s^2
    Eigen::Matrix3d Sa, Na; // 比例因子与交轴耦合矩阵

public:
    Accer()
    {
        bias = Eigen::Vector3d::Zero();
        Sa = Eigen::Matrix3d::Zero();
        Na = Eigen::Matrix3d::Zero();
    }

    void setBias(const Eigen::Vector3d &b) { bias = b; }
    void demarcate(const std::array<std::vector<IMUData>, 6> &data);
    void compensate(Eigen::Vector3d &acc_data);
    void showInfo() const;
};

// 陀螺仪模型与补偿参数
class Gyro
{
private:
    Eigen::Vector3d bias; // 零偏，单位 rad/s
    Eigen::Matrix3d Sg;   // 比例因子矩阵

public:
    Gyro()
    {
        bias = Eigen::Vector3d::Zero();
        Sg = Eigen::Matrix3d::Zero();
    }

    void setBias(const Eigen::Vector3d &b) { bias = b; }
    void demarcate(const std::array<std::vector<IMUData>, 6> &sta_data, const std::array<std::vector<IMUData>, 6> &dy_data);
    void compensate(Eigen::Vector3d &gyro_data);
    void showInfo() const;
};

// 惯性导航核心类
class InerNav
{
private:
    bool velUpdate(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur);
    bool blhUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur);
    bool attitudeUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur);

public:
    Accer acc;
    Gyro gyro;

public:
    InerNav() {}

    bool compensate(IMUData &raw_data, bool need_trans = false);
    void staticCoarseAlign(const double lat, const IMUData &imudata, PVA &pva);
    bool isZeroVel(const IMUData &imu_cur) const; // 判断零速
    void update(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur);
    void update_with_zupt(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur);
};
