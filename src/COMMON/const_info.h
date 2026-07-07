#pragma once
#include <string>
#include <Eigen/Dense>

#define G_VAL 9.7936174
#define PI 3.1415926535898
#define We 7.292115E-5 // 地球自转角速度(rad/s)

const double DEG2RAD = PI / 180.0;
const double RAD2DEG = 180.0 / PI;

/*
地球椭球结构
*/
class Ellipsoid
{
public:
    Ellipsoid();
    Ellipsoid(const std::string &type);

    double Rm(double latitude) const; // 子午圈曲率半径
    double Rn(double latitude) const; // 卯酉圈曲率半径

    double get_we() const; // 地球自转角速度

private:
    double a = 0.;  // 长半轴
    double b = 0.;  // 短半轴
    double f = 0.;  // 扁率
    double e = 0.;  // 第一偏心率
    double ee = 0.; // 第二偏心率

    double we = 0.; // 地球自转角速度
    double GM = 0.; // 地球引力常数

    void createWGS84();
    void createCGCS2000();
    void createGRS80();
};

// const Ellipsoid WGS84("WGS84");
// const Ellipsoid CGCS2000("CGCS2000");
extern const Ellipsoid GRS80;

// GRS80地球椭球模型，正常重力计算公式
double GRS80_G0(double latitude);
double GRS80_G(double latitude, double h);
