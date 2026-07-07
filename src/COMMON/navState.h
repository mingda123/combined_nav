#pragma once

#include <cmath>

#include <Eigen/Dense>

#include "Attitude.h"
#include "const_info.h"

struct PVA
{
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();   // 纬度、经度、高程，单位为 rad、rad、m
    Eigen::Vector3d vel_n = Eigen::Vector3d::Zero(); // 导航系 NED 速度，单位 m/s
    Attitude att;
};

struct ImuBiasState
{
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero(); // 陀螺零偏，单位 rad/s
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();  // 加计零偏，单位 m/s^2
};

struct ImuScaleState
{
    Eigen::Vector3d gyro_scale = Eigen::Vector3d::Zero(); // 陀螺比例因子，无量纲
    Eigen::Vector3d acc_scale = Eigen::Vector3d::Zero();  // 加计比例因子，无量纲
};

struct ImuNoiseParam
{
    Eigen::Vector3d gyro_white_noise = Eigen::Vector3d::Zero();      // 陀螺白噪声
    Eigen::Vector3d acc_white_noise = Eigen::Vector3d::Zero();       // 加计白噪声
    Eigen::Vector3d gyro_bias_random_walk = Eigen::Vector3d::Zero(); // 陀螺零偏随机游走
    Eigen::Vector3d acc_bias_random_walk = Eigen::Vector3d::Zero();  // 加计零偏随机游走
};

struct IntegratedNavState
{
    PVA pva;
    ImuBiasState imu_bias;
    ImuScaleState imu_scale;
};

using Vector21d = Eigen::Matrix<double, 21, 1>;
using Matrix21d = Eigen::Matrix<double, 21, 21>;

enum ErrorState21Index
{
    ERR_POS_ID = 0,
    ERR_VEL_ID = 3,
    ERR_ATT_ID = 6,
    ERR_GYRO_BIAS_ID = 9,
    ERR_ACC_BIAS_ID = 12,
    ERR_GYRO_SCALE_ID = 15,
    ERR_ACC_SCALE_ID = 18
};

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &vec)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat(0, 1) = -vec[2];
    mat(0, 2) = vec[1];
    mat(1, 0) = vec[2];
    mat(1, 2) = -vec[0];
    mat(2, 0) = -vec[1];
    mat(2, 1) = vec[0];
    return mat;
}

inline Eigen::Matrix3d diagVector(const Eigen::Vector3d &vec)
{
    Eigen::Matrix3d mat = Eigen::Matrix3d::Zero();
    mat(0, 0) = vec[0];
    mat(1, 1) = vec[1];
    mat(2, 2) = vec[2];
    return mat;
}

inline Eigen::Matrix3d blhToNedScaleMatrix(const Eigen::Vector3d &blh)
{
    const double lat = blh[0];
    const double h = blh[2];
    const double rm = GRS80.Rm(lat) + h;
    const double rn = GRS80.Rn(lat) + h;

    Eigen::Matrix3d scale = Eigen::Matrix3d::Zero();
    scale(0, 0) = rm;
    scale(1, 1) = rn * std::cos(lat);
    scale(2, 2) = -1.0;
    return scale;
}

inline Eigen::Vector3d blhDeltaToNed(const Eigen::Vector3d &ref_blh, const Eigen::Vector3d &delta_blh)
{
    return blhToNedScaleMatrix(ref_blh) * delta_blh;
}

inline Eigen::Vector3d nedDeltaToBlh(const Eigen::Vector3d &ref_blh, const Eigen::Vector3d &delta_ned)
{
    return blhToNedScaleMatrix(ref_blh).inverse() * delta_ned;
}
