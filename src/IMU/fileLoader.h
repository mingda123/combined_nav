#pragma once

#include <Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>

#include "COMMON/MyTime.h"

struct IMUData
{
    GPSTime gpstime; // GPS 周和周内秒
    double dt = 0.0;  // 与上一历元的时间间隔
    double frq = 0.0; // 采样频率

    Eigen::Vector3d dangel = Eigen::Vector3d::Zero(); // 角增量，建议单位为 rad
    Eigen::Vector3d dvel = Eigen::Vector3d::Zero();   // 速度增量，单位为 m/s
};

class FileLoader
{
private:
    std::ifstream file;
    IMUData imu_last; // 上一组IMU数据
    IMUData imu_cur;  // 当前读取的IMU数据

    bool open(const char *filename);
    bool readUnifiedTextRecord(double frq);

public:
    FileLoader() {}
    FileLoader(const char *filename);
    ~FileLoader();

    bool reset(const char *filename);

    IMUData readSingle(double frq, double T = -1.0); // 读取滑动窗口内平均的IMU数据
    bool readAll(std::vector<IMUData> &datas, double frq, double T = -1.0);

    bool isEndOfFile() const;
};
