#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include "GNSS/gnssData.h"
#include "IMU/fileLoader.h"
#include "IMU/inerNav.h"

// 组合模块中的初始对准配置。
struct AlignConfig
{
    std::string imu_file;
    std::string gnss_file;
    double imu_sample_rate = 200.0;
    bool imu_need_trans = false;

    double start_search_length = 600.0; // 起始搜索时长
    double end_search_length = 400.0;   // 末端搜索时长
    double min_static_duration = 180.0; // 静止窗口最短持续时间

    Eigen::Vector3d start_align_acc_bias = Eigen::Vector3d::Zero();  // 起始粗对准使用的加计零偏，单位 m/s^2
    Eigen::Vector3d start_align_gyro_bias = Eigen::Vector3d::Zero(); // 起始粗对准使用的陀螺零偏，单位 rad/s
    Eigen::Vector3d end_align_acc_bias = Eigen::Vector3d::Zero();    // 末端粗对准使用的加计零偏，单位 m/s^2
    Eigen::Vector3d end_align_gyro_bias = Eigen::Vector3d::Zero();   // 末端粗对准使用的陀螺零偏，单位 rad/s

    ImuNoiseParam imu_noise; // IMU 噪声参数，供组合滤波初始化使用
};

// 静止窗口统计结果。
struct StaticAlignWindow
{
    double start_sow = 0.0;
    double end_sow = 0.0;
    double duration = 0.0;
    double static_ratio = 0.0;
    int sample_count = 0;
    IMUData imu_mean;
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
    double acc_norm_std = 0.0;
    double gyro_norm_std = 0.0;
};

// 单端对准结果。
struct SingleAlignSolution
{
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    PVA leveling_compass; // 调平 + 直接罗经
    PVA analytic;         // 解析法静态粗对准
    PVA final_pva;        // 最终采用的初始姿态
    bool use_gnss_yaw_fallback = false;
    double gnss_yaw_rad = 0.0;
    std::string note;
};

// 对准结果。
struct AlignResult
{
    StaticAlignWindow start_window; // 起始静止窗口
    StaticAlignWindow end_window;   // 末端静止窗口
    bool has_end_window = false;

    SingleAlignSolution start_solution;
    SingleAlignSolution end_solution;
};

bool loadAlignInputData(const AlignConfig &cfg, std::vector<IMUData> &imu_datas, std::vector<GNSSData> &gnss_datas);
bool runInitialAlignment(const AlignConfig &cfg, const std::vector<IMUData> &imu_datas, const std::vector<GNSSData> &gnss_datas, AlignResult &result);
void printAlignResult(const AlignResult &result);
