#include <cmath>
#include "const_info.h"

/*
 * @brief 根据传入椭球体模型名称构造椭球体
 */
Ellipsoid::Ellipsoid(const std::string &type)
{
    if (type == "WGS84")
        createWGS84();
    else if (type == "CGCS2000")
        createCGCS2000();
    else if (type == "GRS80")
        createGRS80();
}

/*
 * @brief 计算子午圈半径
 * @param latitude 纬度(rad)
 * @return 子午圈半径
 */
double Ellipsoid::Rm(double latitude) const
{
    return (a * (1 - e * e)) / (pow(1 - e * e * sin(latitude) * sin(latitude), 1.5));
}

/*
 * @brief 计算卯酉圈半径
 * @param latitude 纬度(rad)
 * @return 卯酉圈半径
 */
double Ellipsoid::Rn(double latitude) const
{
    return a / sqrt(1 - e * e * sin(latitude) * sin(latitude));
}

/*
 * @brief 获取地球自转角速度
 */
double Ellipsoid::get_we() const
{
    return we;
}

// 全局GRS80椭球体对象
const Ellipsoid GRS80("GRS80");

/*
 * @brief 设置GRS80相关参数
*/
void Ellipsoid::createGRS80()
{
    a = 6378137.0;
    b = 6356752.3141;
    f = (a - b) / a;
    e = sqrt(a * a - b * b) / a;
    ee = sqrt(a * a - b * b) / b;

    we = 7.292115 * 1e-5;
    GM = 3.986005 * 1e14;
}
/*
 * @brief 设置WGS84相关参数
*/
void Ellipsoid::createWGS84()
{
    a = 6378137.0;
    b = 6356752.3142;
    f = 1 / 298.257223563;
    e = sqrt(a * a - b * b) / a;
    ee = sqrt(a * a - b * b) / b;

    we = 7.292115 * 1e-5;
    GM = 3.986004418 * 1e14;
}

/*
 * @brief 设置CGCS2000相关参数
*/
void Ellipsoid::createCGCS2000()
{
}

/*
 * @brief GRS80地球椭球模型下g0
 * @param latitude 纬度(rad)
 * @return g0
 */
double GRS80_G0(double latitude)
{
    return (9.7803267715 * (1 + 0.0052790414 * pow(sin(latitude), 2) + 0.0000232718 * pow(sin(latitude), 4)));
}

/*
 * @brief GRS80地球椭球模型下g
 * @param latitude 纬度(rad)
 * @param h 高程(m)
 * @return g
 */
double GRS80_G(double latitude, double h)
{
    return GRS80_G0(latitude) - (3.087691089 * 1e-6 - 4.397731 * 1e-9 * pow(sin(latitude), 2)) * h + 0.721 * 1e-12 * h * h;
}