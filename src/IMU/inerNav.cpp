#include <iostream>

#include "inerNav.h"

/*
 * @brief 加速度计六位置标定
 * @param data 六个位置的观测数据组(up->down)
 */
void Accer::demarcate(const std::array<std::vector<IMUData>, 6> &data)
{
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(3, 6);
    // 计算各个位置的平均值(xyz,up->down)
    for (int i = 0; i < 6; i++)
    {
        double dt = 0;
        for (int j = 0; j < data[i].size(); j++)
        {
            L.col(i) += data[i][j].dvel;
            dt += data[i][j].dt;
        }
        L.col(i) /= dt;
    }

    // 构造方程组
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(4, 6);
    A(0, 0) = G_VAL;
    A(0, 1) = -G_VAL;
    A(1, 2) = G_VAL;
    A(1, 3) = -G_VAL;
    A(2, 4) = G_VAL;
    A(2, 5) = -G_VAL;
    A(3, 0) = 1;
    A(3, 1) = 1;
    A(3, 2) = 1;
    A(3, 3) = 1;
    A(3, 4) = 1;
    A(3, 5) = 1;

    // 求解耦合后的零偏/比例因子/交轴耦合
    Eigen::MatrixXd M = L * A.transpose() * (A * A.transpose()).inverse();

    // 赋值
    bias(0) = M(0, 3), bias(1) = M(1, 3), bias(2) = M(2, 3);
    Sa(0, 0) = M(0, 0) - 1, Sa(1, 1) = M(1, 1) - 1, Sa(2, 2) = M(2, 2) - 1;
    Na(0, 1) = M(0, 1), Na(0, 2) = M(0, 2), Na(1, 0) = M(1, 0), Na(1, 2) = M(1, 2), Na(2, 0) = M(2, 0), Na(2, 1) = M(2, 1);
}

/*
 * @brief 陀螺仪标定
 * @param sta_data 静态两位置法数据(up->down)
 * @param dy_data 角位置法数据(+->-)
 */
void Gyro::demarcate(const std::array<std::vector<IMUData>, 6> &sta_data, const std::array<std::vector<IMUData>, 6> &dy_data)
{
    // 静态两位置法计算零偏
    for (int i = 0; i < 3; i++)
    {
        double w_up = 0., w_down = 0.;
        double dt1 = 0., dt2 = 0.;
        for (int j = 0; j < sta_data[2 * i].size(); j++)
        {
            w_up += sta_data[2 * i][j].dangel[i];
            dt1 += sta_data[2 * i][j].dt;
        }
        for (int j = 0; j < sta_data[2 * i + 1].size(); j++)
        {
            w_down += sta_data[2 * i + 1][j].dangel[i];
            dt2 += sta_data[2 * i + 1][j].dt;
        }
        w_up /= dt1, w_down /= dt2;
        bias(i) = (w_up + w_down) / 2.0;
    }

    // 时间区间严格相等
    std::array<IMUData, 6> prc_data;
    double frq = sta_data[0][0].frq;
    for (int i = 0; i < 3; i++)
    {
        double angle_bias = bias(i) / frq; // 角增量零偏改正
        int len1 = dy_data[2 * i].size(), len2 = dy_data[2 * i + 1].size();
        int num_samples = std::min(len1, len2);
        int start1 = (len1 > len2) ? (len1 - len2) / 2 : 0;
        int start2 = (len2 > len1) ? (len2 - len1) / 2 : 0;

        (void)angle_bias;

        // 在同步区间内进行累加
        for (int j = 0; j < num_samples; j++)
        {
            // 累加角增量
            for (int k = 0; k < 3; ++k)
            {
                prc_data[2 * i].dangel[k] += dy_data[2 * i][j + start1].dangel[k];
                prc_data[2 * i + 1].dangel[k] += dy_data[2 * i + 1][j + start2].dangel[k];
            }
        }
    }

    // 角位置法计算比例因子
    for (int i = 0; i < 3; i++)
    {
        double angle1 = prc_data[2 * i].dangel[i];
        double angle2 = prc_data[2 * i + 1].dangel[i];
        Sg(i, i) = (angle1 - angle2) / (2 * 2 * PI) - 1;
    }
}

/*
 * @brief 加速度计观测值改正
 * @param acc_data 原始加速度观测值(xyz)
 * @return 改正后的加速度xyz
 */
void Accer::compensate(Eigen::Vector3d &acc_data)
{
    Eigen::Vector3d compg = (Eigen::Matrix3d::Identity() + Sa + Na).inverse() * (acc_data - bias);
    acc_data = compg;
}

/*
 * @brief 陀螺仪观测值改正
 * @param gyro_data 原始角速度观测值(xyz)
 * @return 改正后的角速度xyz
 */
void Gyro::compensate(Eigen::Vector3d &gyro_data)
{
    Eigen::Vector3d compw = (Eigen::Matrix3d::Identity() + Sg).inverse() * (gyro_data - bias);
    gyro_data = compw;
}

/*
 * @brief 综合加速度计和陀螺仪观测值改正
 * @param raw_data IMUData数据
 * @param need_trans 是否需要坐标转换(ENU->NED)
 * @return 改正成功返回true
 */
bool InerNav::compensate(IMUData &raw_data, bool need_trans)
{
    Eigen::Vector3d original_acc = raw_data.dvel / raw_data.dt;
    Eigen::Vector3d original_gyro = raw_data.dangel / raw_data.dt;

    Eigen::Vector3d transformed_acc;
    Eigen::Vector3d transformed_gyro;

    if (need_trans) // 进行轴系调整
    {
        transformed_acc[0] = original_acc[1];  // 新X = 旧Y
        transformed_acc[1] = original_acc[0];  // 新Y = 旧X
        transformed_acc[2] = -original_acc[2]; // 新Z = -旧Z

        transformed_gyro[0] = original_gyro[1];  // 新X = 旧Y
        transformed_gyro[1] = original_gyro[0];  // 新Y = 旧X
        transformed_gyro[2] = -original_gyro[2]; // 新Z = -旧Z
    }
    else
    {
        transformed_acc = original_acc;
        transformed_gyro = original_gyro;
    }

    acc.compensate(transformed_acc);
    gyro.compensate(transformed_gyro);

    raw_data.dvel = transformed_acc * raw_data.dt;
    raw_data.dangel = transformed_gyro * raw_data.dt;

    return true;
}

/*
 * @brief 显示相关信息
 */
void Accer::showInfo() const
{
    std::cout << "Accelerometer bias\n"
              << bias.transpose() << std::endl;
    std::cout << "Accelerometer scale factor matrix\n"
              << Sa << std::endl;
    std::cout << "Accelerometer cross-coupling matrix\n"
              << Na << std::endl;
}

/*
 * @brief 显示相关信息
 */
void Gyro::showInfo() const
{
    std::cout << "Gyroscope bias\n"
              << bias.transpose() << std::endl;
    std::cout << "Gyroscope scale factor matrix\n"
              << Sg << std::endl;
}

/*
 * @brief 静态粗对准
 * @param lat 纬度(rad)
 * @param imudata imu观测数据
 * @param pva 结果
 */
void InerNav::staticCoarseAlign(const double lat, const IMUData &imudata, PVA &pva)
{
    double loacl_g = GRS80_G(lat, pva.blh[2]);
    // 理想输出向量并单位化
    Eigen::Vector3d gn = {0.0, 0.0, loacl_g}, wie_n = {We * cos(lat), 0.0, -We * sin(lat)};
    Eigen::Vector3d vg = gn, vw = gn.cross(wie_n), vgw = vw.cross(gn);
    vg.normalize();
    vw.normalize();
    vgw.normalize();

    // 观测向量并单位化
    Eigen::Vector3d gb = -imudata.dvel / imudata.dt, wieb = imudata.dangel / imudata.dt;
    Eigen::Vector3d wg = gb, ww = gb.cross(wieb), wgw = ww.cross(gb);
    wg.normalize();
    ww.normalize();
    wgw.normalize();

    // 求解姿态矩阵
    Eigen::Matrix3d A, B;
    A.col(0) = vg, A.col(1) = vw, A.col(2) = vgw;
    B.row(0) = wg.transpose(), B.row(1) = ww.transpose(), B.row(2) = wgw.transpose();
    pva.att.Cb_n = A * B;

    // 求解姿态角
    pva.att.yaw = atan2(pva.att.Cb_n(1, 0), pva.att.Cb_n(0, 0));
    pva.att.pitch = atan(-pva.att.Cb_n(2, 0) / sqrt(pow(pva.att.Cb_n(2, 1), 2) + pow(pva.att.Cb_n(2, 2), 2)));
    pva.att.roll = atan2(pva.att.Cb_n(2, 1), pva.att.Cb_n(2, 2));

    // 四元数
    pva.att.DCM2Quarternion();
}

/*
 * @brief 速度更新
 * @pva_pre2 前两个历元pva结果
 * @param pva_cur 当前历元pva结果
 * @param imu_pre 上历元IMU测量值
 * @param imu_cur 当前历元IMU测量值
 * @return 更新是否成功
 */
bool InerNav::velUpdate(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur)
{
    // 1.重力哥式积分项
    Eigen::Vector3d dv_gc;

    // 预测中间时刻的参数
    PVA pva_mid;
    pva_mid.blh = 1.5 * pva_pre2[0].blh - 0.5 * pva_pre2[1].blh;
    pva_mid.vel_n = 1.5 * pva_pre2[0].vel_n - 0.5 * pva_pre2[1].vel_n;

    // 获取中间时刻的地球参数(GRS80)
    double gp_mid = GRS80_G(pva_mid.blh[0], pva_mid.blh[2]);
    Eigen::Vector3d gpn_mid;
    gpn_mid << 0., 0., gp_mid;
    double Rm_mid = GRS80.Rm(pva_mid.blh[0]), Rn_mid = GRS80.Rn(pva_mid.blh[0]);
    double we_mid = GRS80.get_we();

    // 中间时刻的地球自转角速度和位移角速度
    Eigen::Vector3d wie_mid, wen_mid;
    wie_mid << we_mid * cos(pva_mid.blh[0]), 0.0, -we_mid * sin(pva_mid.blh[0]);
    wen_mid << pva_mid.vel_n[1] / (Rn_mid + pva_mid.blh[2]),
        -pva_mid.vel_n[0] / (Rm_mid + pva_mid.blh[2]),
        -pva_mid.vel_n[1] * tan(pva_mid.blh[0]) / (Rn_mid + pva_mid.blh[2]);

    // 重力哥式积分项
    dv_gc = (gpn_mid - (2 * wie_mid + wen_mid).cross(pva_mid.vel_n)) * imu_cur.dt;

    // 2.比力积分项
    Eigen::Vector3d dv_fn;

    // 等效旋转矢量n(k-1)n(k)(线性外推)
    Attitude att_nk1_nk;
    att_nk1_nk.EqualRotateVec = (wie_mid + wen_mid) * imu_cur.dt;
    Eigen::Matrix3d ssERM;
    ssERM << 0, -att_nk1_nk.EqualRotateVec(2), att_nk1_nk.EqualRotateVec(1),
        att_nk1_nk.EqualRotateVec(2), 0, -att_nk1_nk.EqualRotateVec(0),
        -att_nk1_nk.EqualRotateVec(1), att_nk1_nk.EqualRotateVec(0), 0;

    // b系比力积分项(双子样假设)
    Eigen::Vector3d dv_fb = imu_cur.dvel + 0.5 * imu_cur.dangel.cross(imu_cur.dvel) + (1.0 / 12.0) * (imu_pre.dangel.cross(imu_cur.dvel) + imu_pre.dvel.cross(imu_cur.dangel));

    // 比力积分项
    dv_fn = (Eigen::Matrix3d::Identity() - 0.5 * ssERM) * pva_pre2[0].att.Cb_n * dv_fb;

    // 3.速度更新
    pva_cur.vel_n = pva_pre2[0].vel_n + dv_gc + dv_fn;

    return true;
}

/*
 * @brief blh位置更新
 * @pva_pre 上历元pva结果
 * @param pva_cur 当前历元pva结果
 * @param imu_pre 上历元IMU测量值
 * @param imu_cur 当前历元IMU测量值
 * @return 更新是否成功
 */
bool InerNav::blhUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur)
{
    // 1.高程更新
    pva_cur.blh[2] = pva_pre.blh[2] - 0.5 * (pva_pre.vel_n[2] + pva_cur.vel_n[2]) * imu_cur.dt;

    // 2.纬度更新
    double h_mean = 0.5 * (pva_pre.blh[2] + pva_cur.blh[2]);
    double Rm = GRS80.Rm(pva_pre.blh[0]);
    pva_cur.blh[0] = pva_pre.blh[0] + (pva_pre.vel_n[0] + pva_cur.vel_n[0]) * imu_cur.dt / (2 * (Rm + h_mean));

    // 3.经度更新
    double latitude_mean = 0.5 * (pva_pre.blh[0] + pva_cur.blh[0]);
    double Rn = GRS80.Rn(latitude_mean);
    pva_cur.blh[1] = pva_pre.blh[1] + (pva_pre.vel_n[1] + pva_cur.vel_n[1]) * imu_cur.dt / (2 * (Rn + h_mean) * cos(latitude_mean));

    return true;
}

/*
 * @brief 姿态更新
 * @pva_pre 上历元pva结果
 * @param pva_cur 当前历元pva结果
 * @param imu_pre 上历元IMU测量值
 * @param imu_cur 当前历元IMU测量值
 * @return 更新是否成功
 */
bool InerNav::attitudeUpdate(const PVA &pva_pre, PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur)
{
    Attitude Abk1_k, Ank1_k;
    // 1.b系相当于i系的等效旋转矢量(b(k-1):b(k))
    Abk1_k.EqualRotateVec = imu_cur.dangel + (1.0 / 12.0) * imu_pre.dangel.cross(imu_cur.dangel);
    Abk1_k.ERV2Quarternion();

    // 2.n系相当于i系的等效旋转矢量(n(k-1):n(k))
    PVA pva_mid;
    pva_mid.blh = (pva_cur.blh + pva_pre.blh) / 2.0;
    pva_mid.vel_n = (pva_cur.vel_n + pva_pre.vel_n) / 2.0;

    double gp_mid = GRS80_G(pva_mid.blh[0], pva_mid.blh[2]);
    Eigen::Vector3d gpn_mid;
    gpn_mid << 0., 0., gp_mid;
    double Rm_mid = GRS80.Rm(pva_mid.blh[0]), Rn_mid = GRS80.Rn(pva_mid.blh[0]);
    double we_mid = GRS80.get_we();

    Eigen::Vector3d wie_mid, wen_mid;
    wie_mid << we_mid * cos(pva_mid.blh[0]), 0.0, -we_mid * sin(pva_mid.blh[0]);
    wen_mid << pva_mid.vel_n[1] / (Rn_mid + pva_mid.blh[2]),
        -pva_mid.vel_n[0] / (Rm_mid + pva_mid.blh[2]),
        -pva_mid.vel_n[1] * tan(pva_mid.blh[0]) / (Rn_mid + pva_mid.blh[2]);

    (void)gpn_mid;

    Ank1_k.EqualRotateVec = -(wie_mid + wen_mid) * imu_cur.dt;
    Ank1_k.ERV2Quarternion();

    // 3.四元数和旋转矩阵更新
    pva_cur.att.Qb_n = Ank1_k.Qb_n * pva_pre.att.Qb_n * Abk1_k.Qb_n;
    pva_cur.att.Qb_n.normalize();
    pva_cur.att.Quarternion2DCM();
    pva_cur.att.DCM2Euler();

    return true;
}

/*
 * @brief 判断是否零速
 * @param imu_cur 当前历元IMU测量值
 * @return 是否为零速
 */
bool InerNav::isZeroVel(const IMUData &imu_cur) const
{
    double acc_norm = imu_cur.dvel.norm() / imu_cur.dt;
    double gyro_norm = imu_cur.dangel.norm() / imu_cur.dt;

    bool is_acc_static = std::abs(acc_norm - 9.806) < STATIC_THRESHOLD_ACCEL;
    bool is_gyro_static = gyro_norm < STATIC_THRESHOLD_GYRO;

    return is_acc_static && is_gyro_static;
}

/*
 * @brief 纯惯导更新
 * @param pva_pre2 前两个历元的pva
 * @param pva_cur 当前历元的pva
 * @param imu_pre 上历元IMU测量值
 * @param imu_cur 当前历元IMU测量值
 */
void InerNav::update(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur)
{
    this->velUpdate(pva_pre2, pva_cur, imu_pre, imu_cur);
    this->blhUpdate(pva_pre2[0], pva_cur, imu_pre, imu_cur);
    this->attitudeUpdate(pva_pre2[0], pva_cur, imu_pre, imu_cur);
}

/*
 * @brief 纯惯导更新(含零速更新)
 * @param pva_pre2 前两个历元的pva
 * @param pva_cur 当前历元的pva
 * @param imu_pre 上历元IMU测量值
 * @param imu_cur 当前历元IMU测量值
 */
void InerNav::update_with_zupt(const PVA pva_pre2[2], PVA &pva_cur, const IMUData &imu_pre, const IMUData &imu_cur)
{
    pva_cur.vel_n = Eigen::Vector3d::Zero();
    this->blhUpdate(pva_pre2[0], pva_cur, imu_pre, imu_cur);
    this->attitudeUpdate(pva_pre2[0], pva_cur, imu_pre, imu_cur);
}
