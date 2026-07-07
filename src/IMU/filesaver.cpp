#include <iomanip>
#include <iostream>

#include "filesaver.h"

FileSaver::FileSaver(const char *filename)
{
    open(filename);
}

FileSaver::~FileSaver()
{
    if (file.is_open())
    {
        file.close();
    }
}

bool FileSaver::open(const char *filename)
{
    file.open(filename, std::ios::out | std::ios::trunc);

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
bool FileSaver::reset(const char *filename)
{
    // 关闭当前文件
    if (file.is_open())
    {
        file.close();
    }
    // 打开新文件
    return open(filename);
}

/*
 * @brief 保存单行数据
 */
bool FileSaver::saveSingleInT(const PVA &pva_cur, const IMUData &imu_cur)
{
    if (!file.is_open())
    {
        std::cerr << "错误：文件未打开，无法写入。" << std::endl;
        return false;
    }

    file << std::fixed;
    file << imu_cur.gpstime.week << "," << std::setprecision(3) << imu_cur.gpstime.sow << ",";
    file << std::setprecision(10) << pva_cur.blh[0] * RAD2DEG << "," << pva_cur.blh[1] * RAD2DEG << "," << pva_cur.blh[2] << ",";
    file << std::setprecision(10) << pva_cur.vel_n[0] << "," << pva_cur.vel_n[1] << "," << pva_cur.vel_n[2] << ",";
    file << std::setprecision(10) << pva_cur.att.roll * RAD2DEG << "," << pva_cur.att.pitch * RAD2DEG << "," << pva_cur.att.yaw * RAD2DEG << std::endl;

    return true;
}
