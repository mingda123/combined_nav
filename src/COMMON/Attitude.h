#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

class Attitude
{
public:
    Eigen::Matrix3d Cb_n;           // 方向余弦矩阵(b系到n系)
    double yaw, pitch, roll;        // 载体姿态角(航向、俯仰、横滚) ZYX
    Eigen::Quaterniond Qb_n;        // 姿态四元数 (b系到n系)
    Eigen::Vector3d EqualRotateVec; // 等效旋转矢量(n:b)

    Attitude()
    {
        Cb_n.setZero();
        Qb_n.setIdentity();
        EqualRotateVec.setZero();
        yaw = pitch = roll = 0.0;
    }

    void Euler2DCM();         // 欧拉角转方向余弦矩阵
    void Euler2Quarternion(); // 欧拉角转四元数
    void DCM2Euler();         // 方向余弦转欧拉角
    void DCM2Quarternion();   // 方向余弦转四元数
    void Quarternion2ERV();   // 四元数转等效旋转矢量
    void Quarternion2DCM();   // 四元数转方向余弦
    void ERV2DCM();           // 等效旋转矢量转方向余弦
    void ERV2Quarternion();   // 等效旋转矢量转四元数
};