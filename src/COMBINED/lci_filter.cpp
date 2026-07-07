#include "lci_filter.h"

#include "IMU/inerNav.h"

namespace
{
    constexpr double GNSS_STD_THRESHOLD_NE = 10.0;
    constexpr double GNSS_STD_THRESHOLD_U = 20.0;
}

LciFilter::LciFilter(const LciFilterConfig &cfg) : cfg_(cfg)
{
    initializeCovariance();
}

void LciFilter::setDirection(Direction direction)
{
    direction_ = direction;
}

void LciFilter::initialize(const IntegratedNavState &init_state)
{
    state_ = init_state;
    pva_prev_ = state_.pva;
    pva_prev2_ = state_.pva;
    has_imu_prev_ = false;
    has_imu_cur_ = false;
    has_gnss_ = false;
    timestamp_ = 0.0;
    static_candidate_duration_ = 0.0;
    dx_.setZero();
    initializeCovariance();
}

void LciFilter::addGnssData(const GNSSData &gnss)
{
    gnss_data_ = gnss;
    has_gnss_ = gnss.is_valid;
}

void LciFilter::addImuData(const IMUData &imu)
{
    if (!has_imu_prev_)
    {
        imu_prev_ = imu;
        has_imu_prev_ = true;
        has_imu_cur_ = false;
        return;
    }

    imu_cur_ = imu;
    has_imu_cur_ = true;
}

void LciFilter::processCurrentEpoch(LciFilterResult &result)
{
    if (!has_imu_prev_ || !has_imu_cur_)
    {
        return;
    }

    timestamp_ = imu_cur_.gpstime.sow;
    const bool can_update_with_gnss = has_gnss_ && isGnssStdValid(gnss_data_);
    double update_time = can_update_with_gnss ? gnss_data_.gpstime.sow : -1.0;
    int update_mode = judgeUpdateMode(imu_prev_.gpstime.sow, imu_cur_.gpstime.sow, update_time);

    if (update_mode == UPDATE_NONE)
    {
        IMUData imu_prev = imu_prev_;
        IMUData imu_cur = imu_cur_;
        propagate(imu_prev, imu_cur);
        applyMotionConstraintUpdates(result);
    }
    else if (update_mode == UPDATE_PREVIOUS)
    {
        positionUpdate(gnss_data_);
        result.gnss_pos_updates += 1;
        if (cfg_.use_gnss_velocity)
        {
            velocityUpdate(gnss_data_, imu_prev_);
            result.gnss_vel_updates += 1;
        }
        feedbackState();

        pva_prev_ = state_.pva;

        IMUData imu_prev = imu_prev_;
        IMUData imu_cur = imu_cur_;
        propagate(imu_prev, imu_cur);
        applyMotionConstraintUpdates(result);
    }
    else if (update_mode == UPDATE_CURRENT)
    {
        IMUData imu_prev = imu_prev_;
        IMUData imu_cur = imu_cur_;
        propagate(imu_prev, imu_cur);
        applyMotionConstraintUpdates(result);

        positionUpdate(gnss_data_);
        result.gnss_pos_updates += 1;
        if (cfg_.use_gnss_velocity)
        {
            velocityUpdate(gnss_data_, imu_cur_);
            result.gnss_vel_updates += 1;
        }
        feedbackState();
    }
    else
    {
        IMUData imu_prev = imu_prev_;
        IMUData imu_cur = imu_cur_;
        IMUData mid_imu;
        interpolateImu(imu_prev, imu_cur, update_time, mid_imu);
        IMUData mid_imu_raw = mid_imu;

        propagate(imu_prev, mid_imu);

        positionUpdate(gnss_data_);
        result.gnss_pos_updates += 1;
        if (cfg_.use_gnss_velocity)
        {
            velocityUpdate(gnss_data_, mid_imu);
            result.gnss_vel_updates += 1;
        }
        feedbackState();

        pva_prev_ = state_.pva;
        pva_prev2_ = state_.pva;

        imu_prev = mid_imu_raw;
        propagate(imu_prev, imu_cur);
        applyMotionConstraintUpdates(result);
    }

    pva_prev2_ = pva_prev_;
    pva_prev_ = state_.pva;
    imu_prev_ = imu_cur_;
    has_gnss_ = false;

    result.imu_epochs += 1;
    result.end_sow = timestamp_;
    result.final_state = state_;
    result.final_error_state = dx_;
    result.final_covariance = cov_;
}

double LciFilter::timestamp() const
{
    return timestamp_;
}

IntegratedNavState LciFilter::getState() const
{
    return state_;
}

Matrix21d LciFilter::getCovariance() const
{
    return cov_;
}

void LciFilter::initializeCovariance()
{
    cov_.setZero();
    cov_.block<3, 3>(ERR_POS_ID, ERR_POS_ID) = cfg_.init_pos_std_ned.cwiseProduct(cfg_.init_pos_std_ned).asDiagonal();
    cov_.block<3, 3>(ERR_VEL_ID, ERR_VEL_ID) = cfg_.init_vel_std_ned.cwiseProduct(cfg_.init_vel_std_ned).asDiagonal();
    cov_.block<3, 3>(ERR_ATT_ID, ERR_ATT_ID) = cfg_.init_att_std_rad.cwiseProduct(cfg_.init_att_std_rad).asDiagonal();
    cov_.block<3, 3>(ERR_GYRO_BIAS_ID, ERR_GYRO_BIAS_ID) = cfg_.init_gyro_bias_std.cwiseProduct(cfg_.init_gyro_bias_std).asDiagonal();
    cov_.block<3, 3>(ERR_ACC_BIAS_ID, ERR_ACC_BIAS_ID) = cfg_.init_acc_bias_std.cwiseProduct(cfg_.init_acc_bias_std).asDiagonal();
    cov_.block<3, 3>(ERR_GYRO_SCALE_ID, ERR_GYRO_SCALE_ID) = cfg_.init_gyro_scale_std.cwiseProduct(cfg_.init_gyro_scale_std).asDiagonal();
    cov_.block<3, 3>(ERR_ACC_SCALE_ID, ERR_ACC_SCALE_ID) = cfg_.init_acc_scale_std.cwiseProduct(cfg_.init_acc_scale_std).asDiagonal();
}

int LciFilter::judgeUpdateMode(double imu_time_1, double imu_time_2, double update_time) const
{
    if (!has_gnss_)
    {
        return UPDATE_NONE;
    }

    if (std::abs(imu_time_1 - update_time) < cfg_.gnss_time_tolerance)
    {
        return UPDATE_PREVIOUS;
    }
    if (std::abs(imu_time_2 - update_time) <= cfg_.gnss_time_tolerance)
    {
        return UPDATE_CURRENT;
    }
    const double lower_time = std::min(imu_time_1, imu_time_2);
    const double upper_time = std::max(imu_time_1, imu_time_2);
    if (lower_time < update_time && update_time < upper_time)
    {
        return UPDATE_INTERPOLATED;
    }
    return UPDATE_NONE;
}

bool LciFilter::isGnssStdValid(const GNSSData &gnss_data) const
{
    return gnss_data.pos_std_ned[0] <= GNSS_STD_THRESHOLD_NE &&
           gnss_data.pos_std_ned[1] <= GNSS_STD_THRESHOLD_NE &&
           gnss_data.pos_std_ned[2] <= GNSS_STD_THRESHOLD_U;
}

void LciFilter::interpolateImu(const IMUData &imu1, IMUData &imu2, double timestamp, IMUData &mid_imu) const
{
    const double time_1 = imu1.gpstime.sow;
    const double time_2 = imu2.gpstime.sow;
    const double lower_time = std::min(time_1, time_2);
    const double upper_time = std::max(time_1, time_2);
    if (timestamp < lower_time || timestamp > upper_time)
    {
        mid_imu = IMUData();
        return;
    }

    const double total_dt = imu2.gpstime.sow - imu1.gpstime.sow;
    if (std::abs(total_dt) <= std::numeric_limits<double>::epsilon())
    {
        mid_imu = IMUData();
        return;
    }

    double ratio = (timestamp - imu1.gpstime.sow) / total_dt;
    mid_imu = imu2;
    mid_imu.gpstime.sow = timestamp;
    mid_imu.dangel = imu2.dangel * ratio;
    mid_imu.dvel = imu2.dvel * ratio;
    mid_imu.dt = timestamp - imu1.gpstime.sow;

    imu2.dangel = imu2.dangel - mid_imu.dangel;
    imu2.dvel = imu2.dvel - mid_imu.dvel;
    imu2.dt = imu2.dt - mid_imu.dt;
}

void LciFilter::mechanizeForward(IMUData &imu_prev, IMUData &imu_cur)
{
    InerNav mechanization;
    PVA history[2];
    history[0] = pva_prev_;
    history[1] = pva_prev2_;
    mechanization.update(history, state_.pva, imu_prev, imu_cur);
}

void LciFilter::mechanizeBackward(IMUData &imu_prev, IMUData &imu_cur)
{
    /* 后向惯导机械编排
     * 参考: Erensu/ignav ins-back-mech.cc updateinsbn()
     *
     * 在伪时间线中 dt > 0, 但积分方向与物理时间相反:
     *   - 速度: 减去速度增量 (v_cur = v_prev - dv_nav - dv_cor)
     *   - 姿态: 角速度增量取反后做姿态更新
     *   - 位置: 中值速度公式是时间对称的, 沿用前向公式
     */
    (void)imu_prev;
    const double dt = imu_cur.dt;

    if (dt <= 0.0)
        return;

    const Eigen::Vector3d &blh = pva_prev_.blh;
    const Eigen::Vector3d &vel_n = pva_prev_.vel_n;
    const double lat = blh[0];
    const double h = blh[2];
    const double rm = GRS80.Rm(lat) + h;
    const double rn = GRS80.Rn(lat) + h;
    const double gravity = GRS80_G(lat, h);

    Eigen::Vector3d wie_n;
    wie_n << We * std::cos(lat), 0.0, -We * std::sin(lat);

    Eigen::Vector3d wen_n;
    wen_n << vel_n[1] / rn,
             -vel_n[0] / rm,
             -vel_n[1] * std::tan(lat) / rn;

    Eigen::Vector3d dv_body = imu_cur.dvel;
    Eigen::Vector3d dv_nav = pva_prev_.att.Cb_n * dv_body;

    Eigen::Vector3d omega_sum = 2.0 * wie_n + wen_n;
    Eigen::Vector3d dv_cor = omega_sum.cross(vel_n) * dt;
    dv_cor[2] += gravity * dt;

    state_.pva.vel_n = vel_n - dv_nav - dv_cor;

    Eigen::Vector3d gyro_b = imu_cur.dangel / dt;
    Eigen::Vector3d da_body = -gyro_b * dt;

    Attitude qb;
    qb.EqualRotateVec = da_body;
    qb.ERV2Quarternion();

    Attitude q;
    q.Qb_n = pva_prev_.att.Qb_n * qb.Qb_n;
    q.Qb_n.normalize();

    Eigen::Vector3d w_n = (wen_n + wie_n) * dt;
    Attitude qn;
    qn.EqualRotateVec = w_n;
    qn.ERV2Quarternion();

    state_.pva.att.Qb_n = qn.Qb_n * q.Qb_n;
    state_.pva.att.Qb_n.normalize();
    state_.pva.att.Quarternion2DCM();
    state_.pva.att.DCM2Euler();

    const Eigen::Vector3d vel_mid = 0.5 * (vel_n + state_.pva.vel_n);
    const double h_mean = 0.5 * (h + state_.pva.blh[2]);
    const double rm_mid = GRS80.Rm(lat) + h_mean;
    const double rn_mid = GRS80.Rn(lat) + h_mean;

    state_.pva.blh[2] = h - vel_mid[2] * dt;
    state_.pva.blh[0] = lat + vel_mid[0] * dt / rm_mid;

    const double lat_mean = 0.5 * (lat + state_.pva.blh[0]);
    const double rn_mean = GRS80.Rn(lat_mean) + h_mean;
    state_.pva.blh[1] = blh[1] + vel_mid[1] * dt / (rn_mean * std::cos(lat_mean));
}

void LciFilter::propagate(IMUData &imu_prev, IMUData &imu_cur)
{
    compensateSingleImu(imu_prev);
    compensateSingleImu(imu_cur);

    if (direction_ == FORWARD)
        mechanizeForward(imu_prev, imu_cur);
    else
        mechanizeBackward(imu_prev, imu_cur);

    Matrix21d f = Matrix21d::Zero();
    Matrix21d phi = Matrix21d::Identity();
    Matrix21d qd = Matrix21d::Zero();

    const Eigen::Vector3d &blh = pva_prev_.blh;
    const Eigen::Vector3d &vel = pva_prev_.vel_n;
    double lat = blh[0];
    double h = blh[2];
    double rm = GRS80.Rm(lat) + h;
    double rn = GRS80.Rn(lat) + h;
    double gravity = GRS80_G(lat, h);

    Eigen::Vector3d wie_n;
    wie_n << We * std::cos(lat), 0.0, -We * std::sin(lat);

    Eigen::Vector3d wen_n;
    wen_n << vel[1] / rn,
        -vel[0] / rm,
        -vel[1] * std::tan(lat) / rn;

    Eigen::Vector3d accel_b = imu_cur.dvel / imu_cur.dt;
    Eigen::Vector3d gyro_b = imu_cur.dangel / imu_cur.dt;

    Eigen::Matrix3d temp = Eigen::Matrix3d::Zero();
    temp(0, 0) = -vel[2] / rm;
    temp(0, 2) = vel[0] / rm;
    temp(1, 0) = vel[1] * std::tan(lat) / rn;
    temp(1, 1) = -(vel[2] + vel[0] * std::tan(lat)) / rn;
    temp(1, 2) = vel[1] / rn;
    f.block<3, 3>(ERR_POS_ID, ERR_POS_ID) = temp;
    f.block<3, 3>(ERR_POS_ID, ERR_VEL_ID) = Eigen::Matrix3d::Identity();

    temp.setZero();
    temp(2, 2) = 2.0 * gravity / std::sqrt(rm * rn);
    f.block<3, 3>(ERR_VEL_ID, ERR_POS_ID) = temp;
    f.block<3, 3>(ERR_VEL_ID, ERR_VEL_ID) = -2.0 * skewSymmetric(wie_n) - skewSymmetric(wen_n);
    f.block<3, 3>(ERR_VEL_ID, ERR_ATT_ID) = skewSymmetric(pva_prev_.att.Cb_n * accel_b);
    f.block<3, 3>(ERR_VEL_ID, ERR_ACC_BIAS_ID) = pva_prev_.att.Cb_n;
    f.block<3, 3>(ERR_VEL_ID, ERR_ACC_SCALE_ID) = pva_prev_.att.Cb_n * diagVector(accel_b);

    temp.setZero();
    temp(0, 1) = 1.0 / rn;
    temp(1, 0) = -1.0 / rm;
    temp(2, 1) = -std::tan(lat) / rn;
    f.block<3, 3>(ERR_ATT_ID, ERR_VEL_ID) = temp;
    f.block<3, 3>(ERR_ATT_ID, ERR_ATT_ID) = -skewSymmetric(wie_n + wen_n);
    f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_BIAS_ID) = -pva_prev_.att.Cb_n;
    f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_SCALE_ID) = -pva_prev_.att.Cb_n * diagVector(gyro_b);

    f.block<3, 3>(ERR_GYRO_BIAS_ID, ERR_GYRO_BIAS_ID) = -Eigen::Matrix3d::Identity() / cfg_.corr_time;
    f.block<3, 3>(ERR_ACC_BIAS_ID, ERR_ACC_BIAS_ID) = -Eigen::Matrix3d::Identity() / cfg_.corr_time;
    f.block<3, 3>(ERR_GYRO_SCALE_ID, ERR_GYRO_SCALE_ID) = -Eigen::Matrix3d::Identity() / cfg_.corr_time;
    f.block<3, 3>(ERR_ACC_SCALE_ID, ERR_ACC_SCALE_ID) = -Eigen::Matrix3d::Identity() / cfg_.corr_time;

    // 反向滤波时，需要对 F 矩阵中速度无关项做符号翻转
    // （速度相关项通过伪速度取反已自动正确，速度无关项则需显式处理）
    if (direction_ == BACKWARD)
    {
        // 速度-姿态、偏置、比例因子：反向速度更新中减 dv_nav，符号与正向相反
        f.block<3, 3>(ERR_VEL_ID, ERR_ATT_ID) = -f.block<3, 3>(ERR_VEL_ID, ERR_ATT_ID);
        f.block<3, 3>(ERR_VEL_ID, ERR_ACC_BIAS_ID) = -f.block<3, 3>(ERR_VEL_ID, ERR_ACC_BIAS_ID);
        f.block<3, 3>(ERR_VEL_ID, ERR_ACC_SCALE_ID) = -f.block<3, 3>(ERR_VEL_ID, ERR_ACC_SCALE_ID);
        // 姿态-陀螺偏置/比例因子：反向姿态用 -dangel，偏置贡献方向与正向相反
        f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_BIAS_ID) = -f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_BIAS_ID);
        f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_SCALE_ID) = -f.block<3, 3>(ERR_ATT_ID, ERR_GYRO_SCALE_ID);
        // GM 偏置/比例因子：反向协方差应增长而非衰减（+I/tau 而非 -I/tau）
        f.block<3, 3>(ERR_GYRO_BIAS_ID, ERR_GYRO_BIAS_ID) = -f.block<3, 3>(ERR_GYRO_BIAS_ID, ERR_GYRO_BIAS_ID);
        f.block<3, 3>(ERR_ACC_BIAS_ID, ERR_ACC_BIAS_ID) = -f.block<3, 3>(ERR_ACC_BIAS_ID, ERR_ACC_BIAS_ID);
        f.block<3, 3>(ERR_GYRO_SCALE_ID, ERR_GYRO_SCALE_ID) = -f.block<3, 3>(ERR_GYRO_SCALE_ID, ERR_GYRO_SCALE_ID);
        f.block<3, 3>(ERR_ACC_SCALE_ID, ERR_ACC_SCALE_ID) = -f.block<3, 3>(ERR_ACC_SCALE_ID, ERR_ACC_SCALE_ID);
        // f_vel_vel 中 wie 项：通过伪速度翻转了 wen 但未翻转 wie，需修正
        f.block<3, 3>(ERR_VEL_ID, ERR_VEL_ID) += 4.0 * skewSymmetric(wie_n);
        // f_att_att 中 wie 项：同上
        f.block<3, 3>(ERR_ATT_ID, ERR_ATT_ID) += 2.0 * skewSymmetric(wie_n);
    }

    const double dt = imu_cur.dt;
    phi = phi + f * dt;

    qd.block<3, 3>(ERR_VEL_ID, ERR_VEL_ID) =
        pva_prev_.att.Cb_n * cfg_.imu_noise.acc_white_noise.cwiseProduct(cfg_.imu_noise.acc_white_noise).asDiagonal() *
        pva_prev_.att.Cb_n.transpose() * dt;
    qd.block<3, 3>(ERR_ATT_ID, ERR_ATT_ID) =
        pva_prev_.att.Cb_n * cfg_.imu_noise.gyro_white_noise.cwiseProduct(cfg_.imu_noise.gyro_white_noise).asDiagonal() *
        pva_prev_.att.Cb_n.transpose() * dt;
    qd.block<3, 3>(ERR_GYRO_BIAS_ID, ERR_GYRO_BIAS_ID) =
        cfg_.imu_noise.gyro_bias_random_walk.cwiseProduct(cfg_.imu_noise.gyro_bias_random_walk).asDiagonal() * dt;
    qd.block<3, 3>(ERR_ACC_BIAS_ID, ERR_ACC_BIAS_ID) =
        cfg_.imu_noise.acc_bias_random_walk.cwiseProduct(cfg_.imu_noise.acc_bias_random_walk).asDiagonal() * dt;

    qd = (phi * qd * phi.transpose() + qd) * 0.5;
    predict(phi, qd);
}

void LciFilter::positionUpdate(const GNSSData &gnss_data)
{
    Eigen::Vector3d lever_n = state_.pva.att.Cb_n * cfg_.gnss_imu_lever_b;
    Eigen::Vector3d antenna_blh = state_.pva.blh + nedDeltaToBlh(state_.pva.blh, lever_n);
    Eigen::Vector3d innovation_ned = blhDeltaToNed(state_.pva.blh, antenna_blh - gnss_data.blh);

    Eigen::Matrix<double, 3, 21> h = Eigen::Matrix<double, 3, 21>::Zero();
    h.block<3, 3>(0, ERR_POS_ID) = Eigen::Matrix3d::Identity();
    h.block<3, 3>(0, ERR_ATT_ID) = skewSymmetric(lever_n);

    Eigen::Matrix3d r = gnss_data.pos_std_ned.cwiseProduct(gnss_data.pos_std_ned).asDiagonal();
    update(innovation_ned, h, r);
}

void LciFilter::velocityUpdate(const GNSSData &gnss_data, const IMUData &imu_at_update)
{
    Eigen::Vector3d omega_ib_b = computeCorrectedBodyRate(imu_at_update);
    Eigen::Vector3d omega_in_n = computeNavigationAngularRateN();
    Eigen::Vector3d omega_in_b = state_.pva.att.Cb_n.transpose() * omega_in_n;
    Eigen::Vector3d omega_nb_b = omega_ib_b - omega_in_b;
    Eigen::Vector3d lever_vel_n = state_.pva.att.Cb_n * omega_nb_b.cross(cfg_.gnss_imu_lever_b);
    Eigen::Vector3d antenna_vel_n = state_.pva.vel_n + lever_vel_n;
    Eigen::Vector3d innovation_ned = antenna_vel_n - gnss_data.vel_n;

    Eigen::Matrix<double, 3, 21> h = Eigen::Matrix<double, 3, 21>::Zero();
    h.block<3, 3>(0, ERR_VEL_ID) = Eigen::Matrix3d::Identity();
    h.block<3, 3>(0, ERR_ATT_ID) = -skewSymmetric(lever_vel_n);
    h.block<3, 3>(0, ERR_GYRO_BIAS_ID) = state_.pva.att.Cb_n * skewSymmetric(cfg_.gnss_imu_lever_b);
    h.block<3, 3>(0, ERR_GYRO_SCALE_ID) = -state_.pva.att.Cb_n * skewSymmetric(cfg_.gnss_imu_lever_b) * diagVector(imu_at_update.dangel / imu_at_update.dt);

    Eigen::Matrix3d r = gnss_data.vel_std_ned.cwiseProduct(gnss_data.vel_std_ned).asDiagonal();
    update(innovation_ned, h, r);
}

void LciFilter::zeroVelocityUpdate()
{
    Eigen::Vector3d innovation_ned = state_.pva.vel_n;

    Eigen::Matrix<double, 3, 21> h = Eigen::Matrix<double, 3, 21>::Zero();
    h.block<3, 3>(0, ERR_VEL_ID) = Eigen::Matrix3d::Identity();

    Eigen::Matrix3d r = cfg_.zupt_std_ned.cwiseProduct(cfg_.zupt_std_ned).asDiagonal();
    update(innovation_ned, h, r);
}

void LciFilter::nonHolonomicUpdate()
{
    Eigen::Vector3d body_vel = state_.pva.att.Cb_n.transpose() * state_.pva.vel_n;
    Eigen::Vector2d innovation_body;
    innovation_body << body_vel[1], body_vel[2];

    Eigen::Matrix<double, 2, 21> h = Eigen::Matrix<double, 2, 21>::Zero();
    Eigen::Matrix3d cbn_t = state_.pva.att.Cb_n.transpose();
    h.block<2, 3>(0, ERR_VEL_ID) = cbn_t.block<2, 3>(1, 0);
    h.block<2, 3>(0, ERR_ATT_ID) = (-cbn_t * skewSymmetric(state_.pva.vel_n)).block<2, 3>(1, 0);

    Eigen::Matrix2d r = Eigen::Matrix2d::Zero();
    r(0, 0) = cfg_.nhc_std_body[1] * cfg_.nhc_std_body[1];
    r(1, 1) = cfg_.nhc_std_body[2] * cfg_.nhc_std_body[2];
    update(innovation_body, h, r);
}

void LciFilter::applyMotionConstraintUpdates(LciFilterResult &result)
{
    bool updated = false;

    if (cfg_.use_zupt)
    {
        if (isStaticForZupt(imu_cur_))
        {
            static_candidate_duration_ += imu_cur_.dt;
        }
        else
        {
            static_candidate_duration_ = 0.0;
        }

        if (static_candidate_duration_ >= cfg_.zupt_min_duration)
        {
            zeroVelocityUpdate();
            result.zupt_updates += 1;
            updated = true;
        }
    }

    if (cfg_.use_nhc)
    {
        nonHolonomicUpdate();
        result.nhc_updates += 1;
        updated = true;
    }

    if (updated)
    {
        feedbackState();
    }
}

bool LciFilter::isStaticForZupt(const IMUData &imu) const
{
    if (imu.dt <= 0.0)
    {
        return false;
    }

    Eigen::Vector3d acc = imu.dvel / imu.dt;
    Eigen::Vector3d gyro = imu.dangel / imu.dt;
    return std::abs(acc.norm() - G_VAL) < cfg_.zupt_acc_threshold &&
           gyro.norm() < cfg_.zupt_gyro_threshold;
}

Eigen::Vector3d LciFilter::computeCorrectedBodyRate(const IMUData &imu) const
{
    if (imu.dt <= 0.0)
    {
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d corrected_rate = imu.dangel / imu.dt;
    corrected_rate -= state_.imu_bias.gyro_bias;
    corrected_rate -= state_.imu_scale.gyro_scale.cwiseProduct(imu.dangel / imu.dt);
    return corrected_rate;
}

Eigen::Vector3d LciFilter::computeNavigationAngularRateN() const
{
    double lat = state_.pva.blh[0];
    double h = state_.pva.blh[2];
    double rm = GRS80.Rm(lat) + h;
    double rn = GRS80.Rn(lat) + h;

    Eigen::Vector3d wie_n;
    wie_n << We * std::cos(lat), 0.0, -We * std::sin(lat);

    Eigen::Vector3d wen_n;
    wen_n << state_.pva.vel_n[1] / rn,
        -state_.pva.vel_n[0] / rm,
        -state_.pva.vel_n[1] * std::tan(lat) / rn;

    return wie_n + wen_n;
}

void LciFilter::predict(const Matrix21d &phi, const Matrix21d &qd)
{
    cov_ = phi * cov_ * phi.transpose() + qd;
    dx_ = phi * dx_;
}

void LciFilter::update(const Eigen::VectorXd &innovation, const Eigen::MatrixXd &h, const Eigen::MatrixXd &r)
{
    Eigen::MatrixXd s = h * cov_ * h.transpose() + r;
    Eigen::MatrixXd k = cov_ * h.transpose() * s.inverse();
    Matrix21d identity = Matrix21d::Identity();
    Matrix21d ikh = identity - k * h;

    dx_ = dx_ + k * (innovation - h * dx_);
    cov_ = ikh * cov_ * ikh.transpose() + k * r * k.transpose();
    cov_ = 0.5 * (cov_ + cov_.transpose());
}

void LciFilter::feedbackState()
{
    Eigen::Vector3d delta_pos = dx_.segment<3>(ERR_POS_ID);
    Eigen::Vector3d delta_vel = dx_.segment<3>(ERR_VEL_ID);
    Eigen::Vector3d delta_att = dx_.segment<3>(ERR_ATT_ID);
    Eigen::Vector3d delta_bg = dx_.segment<3>(ERR_GYRO_BIAS_ID);
    Eigen::Vector3d delta_ba = dx_.segment<3>(ERR_ACC_BIAS_ID);
    Eigen::Vector3d delta_sg = dx_.segment<3>(ERR_GYRO_SCALE_ID);
    Eigen::Vector3d delta_sa = dx_.segment<3>(ERR_ACC_SCALE_ID);

    state_.pva.blh -= nedDeltaToBlh(state_.pva.blh, delta_pos);
    state_.pva.vel_n -= delta_vel;

    Attitude delta_q;
    delta_q.EqualRotateVec = delta_att;
    delta_q.ERV2Quarternion();
    state_.pva.att.Qb_n = delta_q.Qb_n * state_.pva.att.Qb_n;
    state_.pva.att.Qb_n.normalize();
    state_.pva.att.Quarternion2DCM();
    state_.pva.att.DCM2Euler();

    state_.imu_bias.gyro_bias += delta_bg;
    state_.imu_bias.acc_bias += delta_ba;
    state_.imu_scale.gyro_scale += delta_sg;
    state_.imu_scale.acc_scale += delta_sa;

    dx_.setZero();
}

void LciFilter::compensateSingleImu(IMUData &imu) const
{
    imu.dangel -= state_.imu_bias.gyro_bias * imu.dt;
    imu.dvel -= state_.imu_bias.acc_bias * imu.dt;
    imu.dangel -= state_.imu_scale.gyro_scale.cwiseProduct(imu.dangel);
    imu.dvel -= state_.imu_scale.acc_scale.cwiseProduct(imu.dvel);
}
