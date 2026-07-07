#pragma once

#include <string>

#include <Eigen/Dense>

#include "COMMON/navState.h"
#include "GNSS/gnssData.h"
#include "IMU/fileLoader.h"

struct LciFilterConfig
{
    Eigen::Vector3d gnss_imu_lever_b = Eigen::Vector3d::Zero();
    ImuNoiseParam imu_noise;

    Eigen::Vector3d init_pos_std_ned = Eigen::Vector3d::Constant(5.0);
    Eigen::Vector3d init_vel_std_ned = Eigen::Vector3d::Constant(0.5);
    Eigen::Vector3d init_att_std_rad = Eigen::Vector3d::Constant(3.0 * DEG2RAD);
    Eigen::Vector3d init_gyro_bias_std = Eigen::Vector3d::Constant(0.005);
    Eigen::Vector3d init_acc_bias_std = Eigen::Vector3d::Constant(0.05);
    Eigen::Vector3d init_gyro_scale_std = Eigen::Vector3d::Constant(1e-4);
    Eigen::Vector3d init_acc_scale_std = Eigen::Vector3d::Constant(1e-4);

    double corr_time = 3600.0;
    double gnss_time_tolerance = 0.001;
    bool use_gnss_velocity = false;
    bool use_zupt = false;
    bool use_nhc = false;
    Eigen::Vector3d zupt_std_ned = Eigen::Vector3d::Constant(0.05);
    Eigen::Vector3d nhc_std_body = Eigen::Vector3d(0.0, 0.10, 0.10); // 按车体系 x、y、z 顺序给定标准差，NHC 实际只使用 y、z

    double zupt_acc_threshold = 0.15;
    double zupt_gyro_threshold = 0.02;
    double zupt_min_duration = 0.5;
};

struct LciFilterResult
{
    bool valid = false;
    int imu_epochs = 0;
    int gnss_pos_updates = 0;
    int gnss_vel_updates = 0;
    int zupt_updates = 0;
    int nhc_updates = 0;

    double start_sow = 0.0;
    double end_sow = 0.0;

    IntegratedNavState final_state;
    Vector21d final_error_state = Vector21d::Zero();
    Matrix21d final_covariance = Matrix21d::Zero();

    std::string note;
};

class LciFilter
{
public:
    enum Direction
    {
        FORWARD = 0,
        BACKWARD = 1
    };

    explicit LciFilter(const LciFilterConfig &cfg);

    void setDirection(Direction direction);
    void initialize(const IntegratedNavState &init_state);
    void addGnssData(const GNSSData &gnss);
    void addImuData(const IMUData &imu);
    void processCurrentEpoch(LciFilterResult &result);

    double timestamp() const;
    IntegratedNavState getState() const;
    Matrix21d getCovariance() const;

private:
    enum UpdateMode
    {
        UPDATE_NONE = 0,
        UPDATE_PREVIOUS = 1,
        UPDATE_CURRENT = 2,
        UPDATE_INTERPOLATED = 3
    };

    // 初始化滤波协方差
    void initializeCovariance();

    // 判断 GNSS 时间落在当前两帧 IMU 的哪一种关系下
    int judgeUpdateMode(double imu_time_1, double imu_time_2, double update_time) const;
    bool isGnssStdValid(const GNSSData &gnss_data) const;

    // 当 GNSS 时间落在两帧 IMU 中间时，对后一帧 IMU 做拆分插值
    void interpolateImu(const IMUData &imu1, IMUData &imu2, double timestamp, IMUData &mid_imu) const;

    // 完成一拍惯导传播以及误差状态传播
    void propagate(IMUData &imu_prev, IMUData &imu_cur);

    // 前向/后向惯导机械编排
    void mechanizeForward(IMUData &imu_prev, IMUData &imu_cur);
    void mechanizeBackward(IMUData &imu_prev, IMUData &imu_cur);

    // 使用 GNSS 位置做量测更新，位置量测已考虑杆臂
    void positionUpdate(const GNSSData &gnss_data);

    // 使用 GNSS 速度做量测更新，速度量测已考虑杆臂旋转项
    void velocityUpdate(const GNSSData &gnss_data, const IMUData &imu_at_update);

    // 零速约束更新
    void zeroVelocityUpdate();

    // 非完整性约束更新，约束车体系侧向和竖向速度
    void nonHolonomicUpdate();

    // 在传播结束后统一处理零速和 NHC 约束
    void applyMotionConstraintUpdates(LciFilterResult &result);

    // 判断当前 IMU 历元是否满足零速静止条件
    bool isStaticForZupt(const IMUData &imu) const;

    // 计算补偿零偏与比例因子后的机体系角速度
    Eigen::Vector3d computeCorrectedBodyRate(const IMUData &imu) const;

    // 计算导航系相对惯性系的角速度
    Eigen::Vector3d computeNavigationAngularRateN() const;

    // 误差状态预测
    void predict(const Matrix21d &phi, const Matrix21d &qd);

    // 卡尔曼量测更新
    void update(const Eigen::VectorXd &innovation, const Eigen::MatrixXd &h, const Eigen::MatrixXd &r);

    // 将误差状态反馈回导航主状态
    void feedbackState();

    // 对单帧 IMU 增量做零偏和比例因子补偿
    void compensateSingleImu(IMUData &imu) const;

private:
    LciFilterConfig cfg_;
    Direction direction_ = FORWARD;
    IntegratedNavState state_;
    PVA pva_prev_;
    PVA pva_prev2_;
    IMUData imu_prev_;
    IMUData imu_cur_;
    GNSSData gnss_data_;
    bool has_imu_prev_ = false;
    bool has_imu_cur_ = false;
    bool has_gnss_ = false;
    double timestamp_ = 0.0;
    double static_candidate_duration_ = 0.0;

    Vector21d dx_ = Vector21d::Zero();
    Matrix21d cov_ = Matrix21d::Zero();
};
