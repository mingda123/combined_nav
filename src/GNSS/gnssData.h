#pragma once

#include <Eigen/Dense>

#include "COMMON/MyTime.h"

struct GNSSData
{
    GPSTime gpstime;

    // 位置采用 BLH 表达：[纬度(rad), 经度(rad), 椭球高(m)]
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();

    // 位置标准差采用导航系 NED 表达：[北、东、下]，单位 m
    Eigen::Vector3d pos_std_ned = Eigen::Vector3d::Zero();

    // 速度采用导航系 NED 表达：[vn, ve, vd]，单位 m/s
    Eigen::Vector3d vel_n = Eigen::Vector3d::Zero();

    // 速度标准差采用导航系 NED 表达：[北、东、下]，单位 m/s
    Eigen::Vector3d vel_std_ned = Eigen::Vector3d::Zero();

    bool is_valid = false;
};
