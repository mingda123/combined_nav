#include "fileLoader.h"

#include "COMMON/const_info.h"

#include <cctype>
#include <iostream>
#include <sstream>

namespace
{
    std::string trim(const std::string &text)
    {
        size_t first = 0;
        while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])))
        {
            first++;
        }

        size_t last = text.size();
        while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])))
        {
            last--;
        }

        return text.substr(first, last - first);
    }
}

FileLoader::FileLoader(const char *filename)
{
    reset(filename);
}

FileLoader::~FileLoader()
{
    if (file.is_open())
    {
        file.close();
    }
}

bool FileLoader::open(const char *filename)
{
    file.open(filename);
    if (!file.is_open())
    {
        std::cerr << "错误：无法打开文件 " << filename << std::endl;
        return false;
    }

    return true;
}

/*
 * @brief 重新打开文件
 */
bool FileLoader::reset(const char *filename)
{
    if (file.is_open())
    {
        file.close();
    }

    imu_last = IMUData();
    imu_cur = IMUData();

    return open(filename);
}

/*
 * @brief 读取统一IMU文本格式
 * @param frq 采样率
 * @return 是否读取成功
 *
 * 统一格式字段：
 * Time(sow) gx(deg/s) gy(deg/s) gz(deg/s) ax(m/s^2) ay(m/s^2) az(m/s^2)
 */
bool FileLoader::readUnifiedTextRecord(double frq)
{
    if (frq <= 0.0)
    {
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        return false;
    }

    line = trim(line);
    if (line.empty() || line.find("Time") != std::string::npos)
    {
        return false;
    }

    std::stringstream ss(line);
    double sow = 0.0;
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;

    if (!(ss >> sow >> gx >> gy >> gz >> ax >> ay >> az))
    {
        return false;
    }

    imu_cur = IMUData();
    imu_cur.gpstime.sow = sow;
    imu_cur.frq = frq;
    imu_cur.dt = 1.0 / frq;
    imu_cur.dangel << gx * DEG2RAD * imu_cur.dt, gy * DEG2RAD * imu_cur.dt, gz * DEG2RAD * imu_cur.dt;
    imu_cur.dvel << ax * imu_cur.dt, ay * imu_cur.dt, az * imu_cur.dt;

    return true;
}

/*
 * @brief 读取时间间隔T内的均值
 * @param T 时间间隔(<0：单历元；0：所有数据；>0：T时间间隔)
 * @return 一组IMU数据
 */
IMUData FileLoader::readSingle(double frq, double T)
{
    IMUData mean_imu;
    GPSTime start_time;
    int sample_counts = 0;

    while (file.is_open())
    {
        imu_last = imu_cur;
        if (!readUnifiedTextRecord(frq))
        {
            if (isEndOfFile())
            {
                break;
            }
            continue;
        }

        if (T < 0.0)
        {
            return imu_cur;
        }

        sample_counts++;
        mean_imu.dangel += imu_cur.dangel;
        mean_imu.dvel += imu_cur.dvel;

        if (sample_counts == 1)
        {
            start_time = imu_cur.gpstime;
            mean_imu.frq = imu_cur.frq;
            mean_imu.gpstime = imu_cur.gpstime;
        }

        mean_imu.dt = sample_counts / frq;

        if (T > 0.0)
        {
            const double total_time = imu_cur.gpstime.sow - start_time.sow;
            if (total_time >= T || mean_imu.dt >= T)
            {
                break;
            }
        }
    }

    return mean_imu;
}

/*
 * @brief 读取文件中所有数据(滑动窗口取平均)
 * @param T 时间间隔(默认<0：单历元；0：所有数据；>0：T时间间隔)
 * @return 文件全部IMU数据
 */
bool FileLoader::readAll(std::vector<IMUData> &datas, double frq, double T)
{
    while (file.is_open())
    {
        IMUData data = readSingle(frq, T);
        if (data.frq > 0.0)
        {
            datas.push_back(data);
        }
        else if (isEndOfFile())
        {
            break;
        }
    }

    return !datas.empty();
}

/*
 * @brief 检查是否到达文件末尾
 */
bool FileLoader::isEndOfFile() const
{
    return !file.is_open() || file.eof();
}

