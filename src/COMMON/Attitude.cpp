#include <iostream>
#include <cmath>

#include "Attitude.h"
#include "const_info.h"

/*
@brief 欧拉角转方向余弦矩阵
*/
void Attitude::Euler2DCM()
{
    double cy = std::cos(yaw), sy = std::sin(yaw);
    double ct = std::cos(pitch), st = std::sin(pitch);
    double cf = std::cos(roll), sf = std::sin(roll);

    this->Cb_n << ct * cy, -cf * sy + sf * st * cy, sf * sy + cf * st * cy,
        ct * sy, cf * cy + sf * st * sy, -sf * cy + cf * st * sy,
        -st, sf * ct, cf * ct;
}

/*
@brief 欧拉角转四元数
*/
void Attitude::Euler2Quarternion()
{
    double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
    double ct = std::cos(pitch / 2), st = std::sin(pitch / 2);
    double cf = std::cos(roll / 2), sf = std::sin(roll / 2);

    this->Qb_n.w() = cf * ct * cy + sf * st * sy;
    this->Qb_n.x() = sf * ct * cy - cf * st * sy;
    this->Qb_n.y() = cf * st * cy + sf * ct * sy;
    this->Qb_n.z() = cf * ct * sy - sf * st * cy;
}

/*
@brief 方向余弦转欧拉角
*/
void Attitude::DCM2Euler()
{

    this->pitch = atan(-this->Cb_n(2, 0) / sqrt(pow(this->Cb_n(2, 1), 2) + pow(this->Cb_n(2, 2), 2)));

    if (fabs(this->Cb_n(2, 0)) < 0.999999999)
    {
        this->yaw = atan2(this->Cb_n(1, 0), this->Cb_n(0, 0));
        this->roll = atan2(this->Cb_n(2, 1), this->Cb_n(2, 2));
    }
    else
    {
        double combination = 0;
        if (this->Cb_n(2, 0) <= -0.999999999)
            combination = atan2(this->Cb_n(1, 2) - this->Cb_n(0, 1), this->Cb_n(0, 2) + this->Cb_n(1, 1));
        else
            combination = PI + atan2(this->Cb_n(1, 2) + this->Cb_n(0, 1), this->Cb_n(0, 2) - this->Cb_n(1, 1));

        std::cerr << "we can only calaulate the combination of the yaw and the roll (yaw - roll) :" << combination << std::endl;
    }
}

/*
@brief 方向余弦转四元数
*/
void Attitude::DCM2Quarternion()
{
    double Ctr = this->Cb_n.trace();
    double P1 = 1 + Ctr, P2 = 1 + 2 * this->Cb_n(0, 0) - Ctr, P3 = 1 + 2 * this->Cb_n(1, 1) - Ctr, P4 = 1 + 2 * this->Cb_n(2, 2) - Ctr;

    int max_index = 0;
    double max_value = P1;
    if (P2 > max_value)
    {
        max_value = P2;
        max_index = 1;
    }
    if (P3 > max_value)
    {
        max_value = P3;
        max_index = 2;
    }
    if (P4 > max_value)
    {
        max_value = P4;
        max_index = 3;
    }

    switch (max_index)
    {
    case 0:
        this->Qb_n.w() = 0.5 * sqrt(P1);
        this->Qb_n.x() = (this->Cb_n(2, 1) - this->Cb_n(1, 2)) / (4 * this->Qb_n.w());
        this->Qb_n.y() = (this->Cb_n(0, 2) - this->Cb_n(2, 0)) / (4 * this->Qb_n.w());
        this->Qb_n.z() = (this->Cb_n(1, 0) - this->Cb_n(0, 1)) / (4 * this->Qb_n.w());
        break;
    case 1:
        this->Qb_n.x() = 0.5 * sqrt(P2);
        this->Qb_n.y() = (this->Cb_n(1, 0) + this->Cb_n(0, 1)) / (4 * this->Qb_n.x());
        this->Qb_n.z() = (this->Cb_n(0, 2) + this->Cb_n(2, 0)) / (4 * this->Qb_n.x());
        this->Qb_n.w() = (this->Cb_n(2, 1) - this->Cb_n(1, 2)) / (4 * this->Qb_n.x());
        break;
    case 2:
        this->Qb_n.y() = 0.5 * sqrt(P3);
        this->Qb_n.z() = (this->Cb_n(2, 1) + this->Cb_n(1, 2)) / (4 * this->Qb_n.y());
        this->Qb_n.w() = (this->Cb_n(0, 2) - this->Cb_n(2, 0)) / (4 * this->Qb_n.y());
        this->Qb_n.x() = (this->Cb_n(0, 1) + this->Cb_n(1, 0)) / (4 * this->Qb_n.y());
        break;
    case 3:
        this->Qb_n.z() = 0.5 * sqrt(P4);
        this->Qb_n.w() = (this->Cb_n(1, 0) - this->Cb_n(0, 1)) / (4 * this->Qb_n.z());
        this->Qb_n.x() = (this->Cb_n(0, 2) + this->Cb_n(2, 0)) / (4 * this->Qb_n.z());
        this->Qb_n.y() = (this->Cb_n(2, 1) + this->Cb_n(1, 2)) / (4 * this->Qb_n.z());
        break;
    default:
        break;
    }
}

/*
@brief 四元数转等效旋转矢量
*/
void Attitude::Quarternion2ERV()
{
    if (this->Qb_n.w() < 0)
    {
        this->Qb_n.w() = -this->Qb_n.w();
        this->Qb_n.x() = -this->Qb_n.x();
        this->Qb_n.y() = -this->Qb_n.y();
        this->Qb_n.z() = -this->Qb_n.z();
    }

    if (abs(this->Qb_n.w()) < 1e-10)
    {
        this->EqualRotateVec = PI * Eigen::Vector3d(this->Qb_n.x(), this->Qb_n.y(), this->Qb_n.z());
    }
    else
    {
        double normERV_half = atan2(sqrt(pow(this->Qb_n.x(), 2) + pow(this->Qb_n.y(), 2) + pow(this->Qb_n.z(), 2)), this->Qb_n.w());
        double f = abs(normERV_half) < 1e-10 ? 0.5 : sin(normERV_half) / (2 * normERV_half);
        this->EqualRotateVec = (1 / f) * Eigen::Vector3d(this->Qb_n.x(), this->Qb_n.y(), this->Qb_n.z());
    }
}

/*
@brief 等效旋转矢量转方向余弦
*/
void Attitude::ERV2DCM()
{
    Eigen::Matrix3d SsMat; // 等效旋转矩阵的反对称矩阵
    SsMat << 0, -this->EqualRotateVec(2), this->EqualRotateVec(1),
        this->EqualRotateVec(2), 0, -this->EqualRotateVec(0),
        -this->EqualRotateVec(1), this->EqualRotateVec(0), 0;
    double normSsMat = this->EqualRotateVec.norm();

    double a, b;
    if (abs(normSsMat) < 1e-10)
    {
        // 泰勒展开近似
        a = 1.0;
        b = 0.5;
    }
    else
    {
        a = std::sin(normSsMat) / normSsMat;
        b = (1.0 - std::cos(normSsMat)) / (normSsMat * normSsMat);
    }

    this->Cb_n = Eigen::Matrix3d::Identity() + a * SsMat + b * SsMat * SsMat;
}

/*
@brief 等效旋转矢量转四元数
*/
void Attitude::ERV2Quarternion()
{
    double normERV2 = this->EqualRotateVec.norm() * 0.5;

    this->Qb_n.w() = std::cos(normERV2);

    double s = (abs(normERV2) > 1e-10) ? (std::sin(normERV2) * 0.5 / normERV2) : 0.5;
    Eigen::Vector3d quarVec = s * this->EqualRotateVec;
    this->Qb_n.x() = quarVec(0), this->Qb_n.y() = quarVec(1), this->Qb_n.z() = quarVec(2);
}

/*
 *@brief 四元数转方向余弦
 */
void Attitude::Quarternion2DCM()
{
    double w = this->Qb_n.w();
    double x = this->Qb_n.x();
    double y = this->Qb_n.y();
    double z = this->Qb_n.z();

    // 预计算平方项
    double xx = x * x;
    double yy = y * y;
    double zz = z * z;
    double xy = x * y;
    double xz = x * z;
    double yz = y * z;
    double wx = w * x;
    double wy = w * y;
    double wz = w * z;

    // 标准转换(B->N)
    this->Cb_n(0, 0) = 1.0 - 2.0 * (yy + zz);
    this->Cb_n(0, 1) = 2.0 * (xy - wz);
    this->Cb_n(0, 2) = 2.0 * (xz + wy);

    this->Cb_n(1, 0) = 2.0 * (xy + wz);
    this->Cb_n(1, 1) = 1.0 - 2.0 * (xx + zz);
    this->Cb_n(1, 2) = 2.0 * (yz - wx);

    this->Cb_n(2, 0) = 2.0 * (xz - wy);
    this->Cb_n(2, 1) = 2.0 * (yz + wx);
    this->Cb_n(2, 2) = 1.0 - 2.0 * (xx + yy);
}
