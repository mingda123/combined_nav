#include "alignment.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

#include "COMMON/Attitude.h"
#include "COMMON/const_info.h"
#include "GNSS/gnssFileLoader.h"

namespace
{
const double STATIC_ACC_THRESHOLD = 0.15;
const double STATIC_GYRO_THRESHOLD = 0.01;
const double STATIC_GNSS_SPEED_THRESHOLD = 0.10;
const double GNSS_TIME_TOLERANCE = 0.6;
const double MIN_STATIC_RATIO = 0.90;
const double MAX_STATIC_BREAK_TIME = 1.0;
const double GNSS_YAW_MIN_SPEED = 0.5;
const double YAW_FALLBACK_THRESHOLD = 45.0 * DEG2RAD;

struct PrefixStats
{
    std::vector<unsigned char> static_flags;
    std::vector<int> valid_count;
    std::vector<int> static_count;
    std::vector<double> dt_sum;
    std::vector<double> acc_norm_sum;
    std::vector<double> acc_norm_sq_sum;
    std::vector<double> gyro_norm_sum;
    std::vector<double> gyro_norm_sq_sum;
    std::vector<Eigen::Vector3d> dvel_sum;
    std::vector<Eigen::Vector3d> dangel_sum;
    std::vector<Eigen::Vector3d> acc_sum;
    std::vector<Eigen::Vector3d> gyro_sum;
};

double normalizeAlignAngle(double angle)
{
    while (angle > PI)
    {
        angle -= 2.0 * PI;
    }
    while (angle < -PI)
    {
        angle += 2.0 * PI;
    }
    return angle;
}

double horizontalSpeed(const GNSSData &gnss)
{
    return std::sqrt(gnss.vel_n[0] * gnss.vel_n[0] + gnss.vel_n[1] * gnss.vel_n[1]);
}

bool estimateYawFromGnssVelocity(const std::vector<GNSSData> &gnss_datas,
                                 double ref_sow,
                                 bool search_forward,
                                 double min_horizontal_speed,
                                 double &yaw_rad)
{
    if (search_forward)
    {
        for (size_t i = 0; i < gnss_datas.size(); ++i)
        {
            const GNSSData &gnss = gnss_datas[i];
            if (gnss.gpstime.sow < ref_sow)
            {
                continue;
            }
            if (horizontalSpeed(gnss) > min_horizontal_speed)
            {
                yaw_rad = std::atan2(gnss.vel_n[1], gnss.vel_n[0]);
                return true;
            }
        }
    }
    else
    {
        for (size_t i = gnss_datas.size(); i-- > 0;)
        {
            const GNSSData &gnss = gnss_datas[i];
            if (gnss.gpstime.sow > ref_sow)
            {
                continue;
            }
            if (horizontalSpeed(gnss) > min_horizontal_speed)
            {
                yaw_rad = std::atan2(gnss.vel_n[1], gnss.vel_n[0]);
                return true;
            }
        }
    }
    return false;
}

bool findNearestGnss(const std::vector<GNSSData> &gnss_datas, double sow, double max_dt, GNSSData &nearest)
{
    if (gnss_datas.empty())
    {
        return false;
    }

    std::vector<GNSSData>::const_iterator it = std::lower_bound(
        gnss_datas.begin(), gnss_datas.end(), sow,
        [](const GNSSData &data, double value)
        {
            return data.gpstime.sow < value;
        });

    double best_dt = std::numeric_limits<double>::max();
    bool found = false;

    if (it != gnss_datas.end())
    {
        best_dt = std::fabs(it->gpstime.sow - sow);
        nearest = *it;
        found = true;
    }

    if (it != gnss_datas.begin())
    {
        std::vector<GNSSData>::const_iterator prev_it = it - 1;
        double dt = std::fabs(prev_it->gpstime.sow - sow);
        if (!found || dt < best_dt)
        {
            best_dt = dt;
            nearest = *prev_it;
            found = true;
        }
    }

    return found && best_dt <= max_dt;
}

bool isStaticEpoch(const IMUData &imu, const std::vector<GNSSData> &gnss_datas)
{
    if (imu.dt <= 0.0)
    {
        return false;
    }

    Eigen::Vector3d acc = imu.dvel / imu.dt;
    Eigen::Vector3d gyro = imu.dangel / imu.dt;
    bool imu_static = std::fabs(acc.norm() - G_VAL) < STATIC_ACC_THRESHOLD &&
                      gyro.norm() < STATIC_GYRO_THRESHOLD;
    if (!imu_static)
    {
        return false;
    }

    GNSSData gnss;
    if (findNearestGnss(gnss_datas, imu.gpstime.sow, GNSS_TIME_TOLERANCE, gnss))
    {
        return horizontalSpeed(gnss) < STATIC_GNSS_SPEED_THRESHOLD;
    }

    return true;
}

PrefixStats buildPrefixStats(const std::vector<IMUData> &imu_datas, const std::vector<GNSSData> &gnss_datas)
{
    PrefixStats stats;
    size_t n = imu_datas.size();
    stats.static_flags.assign(n, 0);
    stats.valid_count.assign(n + 1, 0);
    stats.static_count.assign(n + 1, 0);
    stats.dt_sum.assign(n + 1, 0.0);
    stats.acc_norm_sum.assign(n + 1, 0.0);
    stats.acc_norm_sq_sum.assign(n + 1, 0.0);
    stats.gyro_norm_sum.assign(n + 1, 0.0);
    stats.gyro_norm_sq_sum.assign(n + 1, 0.0);
    stats.dvel_sum.assign(n + 1, Eigen::Vector3d::Zero());
    stats.dangel_sum.assign(n + 1, Eigen::Vector3d::Zero());
    stats.acc_sum.assign(n + 1, Eigen::Vector3d::Zero());
    stats.gyro_sum.assign(n + 1, Eigen::Vector3d::Zero());

    for (size_t i = 0; i < n; ++i)
    {
        stats.valid_count[i + 1] = stats.valid_count[i];
        stats.static_count[i + 1] = stats.static_count[i];
        stats.dt_sum[i + 1] = stats.dt_sum[i];
        stats.acc_norm_sum[i + 1] = stats.acc_norm_sum[i];
        stats.acc_norm_sq_sum[i + 1] = stats.acc_norm_sq_sum[i];
        stats.gyro_norm_sum[i + 1] = stats.gyro_norm_sum[i];
        stats.gyro_norm_sq_sum[i + 1] = stats.gyro_norm_sq_sum[i];
        stats.dvel_sum[i + 1] = stats.dvel_sum[i];
        stats.dangel_sum[i + 1] = stats.dangel_sum[i];
        stats.acc_sum[i + 1] = stats.acc_sum[i];
        stats.gyro_sum[i + 1] = stats.gyro_sum[i];

        const IMUData &imu = imu_datas[i];
        if (imu.dt <= 0.0)
        {
            continue;
        }

        Eigen::Vector3d acc = imu.dvel / imu.dt;
        Eigen::Vector3d gyro = imu.dangel / imu.dt;
        double acc_norm = acc.norm();
        double gyro_norm = gyro.norm();
        bool epoch_static = isStaticEpoch(imu, gnss_datas);

        stats.valid_count[i + 1] += 1;
        stats.static_count[i + 1] += epoch_static ? 1 : 0;
        stats.dt_sum[i + 1] += imu.dt;
        stats.acc_norm_sum[i + 1] += acc_norm;
        stats.acc_norm_sq_sum[i + 1] += acc_norm * acc_norm;
        stats.gyro_norm_sum[i + 1] += gyro_norm;
        stats.gyro_norm_sq_sum[i + 1] += gyro_norm * gyro_norm;
        stats.dvel_sum[i + 1] += imu.dvel;
        stats.dangel_sum[i + 1] += imu.dangel;
        stats.acc_sum[i + 1] += acc;
        stats.gyro_sum[i + 1] += gyro;
        stats.static_flags[i] = epoch_static ? 1 : 0;
    }

    return stats;
}

bool buildWindowFromPrefix(const std::vector<IMUData> &imu_datas,
                           const PrefixStats &stats,
                           size_t start_index,
                           size_t end_index,
                           StaticAlignWindow &window)
{
    if (imu_datas.empty() || start_index >= imu_datas.size() || end_index >= imu_datas.size() || start_index > end_index)
    {
        return false;
    }

    int sample_count = stats.valid_count[end_index + 1] - stats.valid_count[start_index];
    double dt_sum = stats.dt_sum[end_index + 1] - stats.dt_sum[start_index];
    if (sample_count <= 0 || dt_sum <= 0.0)
    {
        return false;
    }

    window = StaticAlignWindow();
    window.start_sow = imu_datas[start_index].gpstime.sow;
    window.end_sow = imu_datas[end_index].gpstime.sow;
    window.duration = window.end_sow - window.start_sow;
    window.sample_count = sample_count;
    window.static_ratio = static_cast<double>(stats.static_count[end_index + 1] - stats.static_count[start_index]) / sample_count;
    window.imu_mean.dvel = stats.dvel_sum[end_index + 1] - stats.dvel_sum[start_index];
    window.imu_mean.dangel = stats.dangel_sum[end_index + 1] - stats.dangel_sum[start_index];
    window.imu_mean.dt = dt_sum;
    window.imu_mean.gpstime = imu_datas[end_index].gpstime;
    window.imu_mean.frq = 1.0;
    window.acc_mean = (stats.acc_sum[end_index + 1] - stats.acc_sum[start_index]) / sample_count;
    window.gyro_mean = (stats.gyro_sum[end_index + 1] - stats.gyro_sum[start_index]) / sample_count;

    double acc_norm_mean = (stats.acc_norm_sum[end_index + 1] - stats.acc_norm_sum[start_index]) / sample_count;
    double gyro_norm_mean = (stats.gyro_norm_sum[end_index + 1] - stats.gyro_norm_sum[start_index]) / sample_count;
    double acc_norm_sq_mean = (stats.acc_norm_sq_sum[end_index + 1] - stats.acc_norm_sq_sum[start_index]) / sample_count;
    double gyro_norm_sq_mean = (stats.gyro_norm_sq_sum[end_index + 1] - stats.gyro_norm_sq_sum[start_index]) / sample_count;
    window.acc_norm_std = std::sqrt(std::max(0.0, acc_norm_sq_mean - acc_norm_mean * acc_norm_mean));
    window.gyro_norm_std = std::sqrt(std::max(0.0, gyro_norm_sq_mean - gyro_norm_mean * gyro_norm_mean));
    return true;
}

bool locateSearchRange(const std::vector<IMUData> &imu_datas,
                       double search_start_sow,
                       double search_end_sow,
                       size_t &begin_index,
                       size_t &end_index)
{
    if (imu_datas.empty() || search_end_sow <= search_start_sow)
    {
        return false;
    }

    begin_index = 0;
    while (begin_index < imu_datas.size() && imu_datas[begin_index].gpstime.sow < search_start_sow)
    {
        ++begin_index;
    }

    if (begin_index >= imu_datas.size())
    {
        return false;
    }

    end_index = begin_index;
    while (end_index + 1 < imu_datas.size() && imu_datas[end_index + 1].gpstime.sow <= search_end_sow)
    {
        ++end_index;
    }

    return begin_index <= end_index;
}

bool searchContinuousStaticWindowInRange(const std::vector<IMUData> &imu_datas,
                                         const PrefixStats &stats,
                                         double search_start_sow,
                                         double search_end_sow,
                                         double min_static_duration,
                                         bool prefer_later,
                                         StaticAlignWindow &best_window)
{
    size_t begin_index = 0;
    size_t end_index = 0;
    if (!locateSearchRange(imu_datas, search_start_sow, search_end_sow, begin_index, end_index))
    {
        return false;
    }

    bool found = false;
    size_t index = begin_index;
    while (index <= end_index)
    {
        while (index <= end_index && stats.static_flags[index] == 0)
        {
            ++index;
        }
        if (index > end_index)
        {
            break;
        }

        size_t start_index = index;
        size_t last_static_index = start_index;
        size_t cursor = start_index + 1;
        double break_time = 0.0;

        while (cursor <= end_index)
        {
            if (stats.static_flags[cursor] != 0)
            {
                last_static_index = cursor;
                break_time = 0.0;
                ++cursor;
                continue;
            }

            double break_dt = imu_datas[cursor].dt > 0.0
                                  ? imu_datas[cursor].dt
                                  : std::max(0.0, imu_datas[cursor].gpstime.sow - imu_datas[cursor - 1].gpstime.sow);
            break_time += break_dt;
            if (break_time > MAX_STATIC_BREAK_TIME)
            {
                break;
            }
            ++cursor;
        }

        StaticAlignWindow candidate;
        if (buildWindowFromPrefix(imu_datas, stats, start_index, last_static_index, candidate) &&
            candidate.duration >= min_static_duration &&
            candidate.static_ratio >= MIN_STATIC_RATIO)
        {
            if (!found)
            {
                best_window = candidate;
                found = true;
            }
            else if (prefer_later)
            {
                if (candidate.end_sow > best_window.end_sow)
                {
                    best_window = candidate;
                }
            }
            else
            {
                best_window = candidate;
                return true;
            }
        }

        index = std::max(cursor, last_static_index + 1);
    }

    return found;
}

bool averageGnssBlh(const std::vector<GNSSData> &gnss_datas, double start_sow, double end_sow, Eigen::Vector3d &blh)
{
    int count = 0;
    blh.setZero();

    for (size_t i = 0; i < gnss_datas.size(); ++i)
    {
        const GNSSData &gnss = gnss_datas[i];
        if (gnss.gpstime.sow < start_sow || gnss.gpstime.sow > end_sow)
        {
            continue;
        }
        blh += gnss.blh;
        ++count;
    }

    if (count == 0)
    {
        return false;
    }

    blh /= count;
    return true;
}

void setPvaAttitudeAlign(PVA &pva, double yaw, double pitch, double roll)
{
    pva.att.yaw = normalizeAlignAngle(yaw);
    pva.att.pitch = pitch;
    pva.att.roll = roll;
    pva.att.Euler2DCM();
    pva.att.Euler2Quarternion();
}

PVA runLevelingCompassAlignment(const Eigen::Vector3d &blh, const IMUData &imu_mean)
{
    PVA pva;
    pva.blh = blh;
    pva.vel_n.setZero();

    Eigen::Vector3d fb = -imu_mean.dvel / imu_mean.dt;
    double roll = std::atan2(fb[1], fb[2]);
    double pitch = std::atan2(-fb[0], std::sqrt(fb[1] * fb[1] + fb[2] * fb[2]));

    Attitude level_att;
    level_att.yaw = 0.0;
    level_att.pitch = pitch;
    level_att.roll = roll;
    level_att.Euler2DCM();

    Eigen::Vector3d wb = imu_mean.dangel / imu_mean.dt;
    Eigen::Vector3d omega_n = level_att.Cb_n * wb;
    double yaw = std::atan2(-omega_n[1], omega_n[0]);

    setPvaAttitudeAlign(pva, yaw, pitch, roll);
    return pva;
}

IMUData applyAlignBiasToMeanImu(const IMUData &imu_mean,
                                const Eigen::Vector3d &acc_bias,
                                const Eigen::Vector3d &gyro_bias)
{
    IMUData corrected_imu = imu_mean;
    corrected_imu.dvel -= acc_bias * corrected_imu.dt;
    corrected_imu.dangel -= gyro_bias * corrected_imu.dt;
    return corrected_imu;
}

PVA runAnalyticAlignment(const Eigen::Vector3d &blh, const IMUData &imu_mean)
{
    InerNav align_nav;
    PVA pva;
    pva.blh = blh;
    pva.vel_n.setZero();
    align_nav.staticCoarseAlign(blh[0], imu_mean, pva);
    return pva;
}

PVA runLevelingWithGivenYaw(const Eigen::Vector3d &blh, const IMUData &imu_mean, double yaw_rad)
{
    PVA pva;
    pva.blh = blh;
    pva.vel_n.setZero();

    Eigen::Vector3d fb = -imu_mean.dvel / imu_mean.dt;
    double roll = std::atan2(fb[1], fb[2]);
    double pitch = std::atan2(-fb[0], std::sqrt(fb[1] * fb[1] + fb[2] * fb[2]));
    setPvaAttitudeAlign(pva, yaw_rad, pitch, roll);
    return pva;
}

bool isStaticAlignmentYawAbnormal(const PVA &analytic_pva, double gnss_yaw_rad)
{
    double yaw_diff = normalizeAlignAngle(analytic_pva.att.yaw - gnss_yaw_rad);
    return std::fabs(yaw_diff) > YAW_FALLBACK_THRESHOLD;
}

bool hasManualGyroBiasForAlignment(const Eigen::Vector3d &gyro_bias)
{
    return gyro_bias.norm() > 1e-12;
}

void buildSingleAlignmentSolution(Eigen::Vector3d blh,
                                  const IMUData &imu_mean,
                                  const Eigen::Vector3d &acc_bias,
                                  const Eigen::Vector3d &gyro_bias,
                                  double yaw_ref_sow,
                                  bool yaw_search_forward,
                                  const std::vector<GNSSData> &gnss_datas,
                                  SingleAlignSolution &solution)
{
    solution = SingleAlignSolution();
    solution.blh = blh;

    IMUData corrected_mean = applyAlignBiasToMeanImu(imu_mean, acc_bias, gyro_bias);
    solution.leveling_compass = runLevelingCompassAlignment(blh, corrected_mean);
    solution.analytic = runAnalyticAlignment(blh, corrected_mean);
    solution.final_pva = solution.analytic;
    solution.note = "采用静态解析法粗对准结果。";

    if (hasManualGyroBiasForAlignment(gyro_bias))
    {
        solution.note = "已提供初始零偏，采用静态解析法粗对准结果。";
        return;
    }

    double gnss_yaw_rad = 0.0;
    if (estimateYawFromGnssVelocity(gnss_datas, yaw_ref_sow, yaw_search_forward, GNSS_YAW_MIN_SPEED, gnss_yaw_rad) &&
        isStaticAlignmentYawAbnormal(solution.analytic, gnss_yaw_rad))
    {
        solution.use_gnss_yaw_fallback = true;
        solution.gnss_yaw_rad = gnss_yaw_rad;
        solution.final_pva = runLevelingWithGivenYaw(blh, corrected_mean, gnss_yaw_rad);
        solution.note = "静态粗对准航向与 GNSS 速度航向差异过大，已回退为调平 + GNSS 速度航向初始化。";
    }
}

void printPvaAttitude(const char *name, const PVA &pva)
{
    std::cout << std::fixed << std::setprecision(6)
              << name
              << " yaw=" << pva.att.yaw * RAD2DEG
              << " deg, pitch=" << pva.att.pitch * RAD2DEG
              << " deg, roll=" << pva.att.roll * RAD2DEG
              << " deg" << std::endl;
}
} // namespace

bool loadAlignInputData(const AlignConfig &cfg, std::vector<IMUData> &imu_datas, std::vector<GNSSData> &gnss_datas)
{
    if (cfg.imu_file.empty() || cfg.gnss_file.empty())
    {
        std::cerr << "错误：IMU 或 GNSS 输入文件为空。" << std::endl;
        return false;
    }

    FileLoader imu_loader(cfg.imu_file.c_str());
    if (!imu_loader.readAll(imu_datas, cfg.imu_sample_rate))
    {
        std::cerr << "错误：未读取到有效 IMU 数据。" << std::endl;
        return false;
    }

    InerNav iner_nav;
    for (size_t i = 0; i < imu_datas.size(); ++i)
    {
        iner_nav.compensate(imu_datas[i], cfg.imu_need_trans);
    }

    GnssFileLoader gnss_loader(cfg.gnss_file.c_str());
    if (!gnss_loader.readAll(gnss_datas))
    {
        std::cerr << "错误：未读取到有效 GNSS 数据。" << std::endl;
        return false;
    }

    return true;
}

bool runInitialAlignment(const AlignConfig &cfg,
                         const std::vector<IMUData> &imu_datas,
                         const std::vector<GNSSData> &gnss_datas,
                         AlignResult &result)
{
    result = AlignResult();
    if (imu_datas.empty() || gnss_datas.empty())
    {
        std::cerr << "错误：对准输入数据为空。" << std::endl;
        return false;
    }

    PrefixStats stats = buildPrefixStats(imu_datas, gnss_datas);
    double data_start_sow = imu_datas.front().gpstime.sow;
    double data_end_sow = imu_datas.back().gpstime.sow;

    double start_search_end = std::min(data_end_sow, data_start_sow + cfg.start_search_length);
    if (!searchContinuousStaticWindowInRange(imu_datas, stats, data_start_sow, start_search_end, cfg.min_static_duration, false, result.start_window))
    {
        std::cerr << "错误：未在起始搜索区间内找到满足条件的静止窗口。" << std::endl;
        return false;
    }

    double end_search_start = std::max(data_start_sow, data_end_sow - cfg.end_search_length);
    result.has_end_window = searchContinuousStaticWindowInRange(imu_datas, stats, end_search_start, data_end_sow, cfg.min_static_duration, true, result.end_window);

    if (!averageGnssBlh(gnss_datas, result.start_window.start_sow, result.start_window.end_sow, result.start_solution.blh))
    {
        GNSSData nearest;
        if (!findNearestGnss(gnss_datas, result.start_window.start_sow, GNSS_TIME_TOLERANCE + 5.0, nearest))
        {
            std::cerr << "错误：起始静止窗口内没有可用 GNSS 位置。" << std::endl;
            return false;
        }
        result.start_solution.blh = nearest.blh;
    }

    buildSingleAlignmentSolution(result.start_solution.blh,
                                 result.start_window.imu_mean,
                                 cfg.start_align_acc_bias,
                                 cfg.start_align_gyro_bias,
                                 result.start_window.end_sow,
                                 true,
                                 gnss_datas,
                                 result.start_solution);

    if (result.has_end_window)
    {
        if (!averageGnssBlh(gnss_datas, result.end_window.start_sow, result.end_window.end_sow, result.end_solution.blh))
        {
            GNSSData nearest;
            if (findNearestGnss(gnss_datas, result.end_window.end_sow, GNSS_TIME_TOLERANCE + 5.0, nearest))
            {
                result.end_solution.blh = nearest.blh;
            }
        }

        if (result.end_solution.blh.norm() != 0.0 || result.end_solution.blh[2] != 0.0)
        {
            buildSingleAlignmentSolution(result.end_solution.blh,
                                         result.end_window.imu_mean,
                                         cfg.end_align_acc_bias,
                                         cfg.end_align_gyro_bias,
                                         result.end_window.start_sow,
                                         false,
                                         gnss_datas,
                                         result.end_solution);
        }
        else
        {
            result.has_end_window = false;
        }
    }

    return true;
}

void printAlignResult(const AlignResult &result)
{
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "起始静止窗口：" << result.start_window.start_sow << " - " << result.start_window.end_sow
              << "，时长 " << result.start_window.duration
              << " s，静止比例 " << result.start_window.static_ratio * 100.0 << "%" << std::endl;
    std::cout << "起始静止均值：加速度 " << result.start_window.acc_mean.transpose()
              << "，角速度 " << result.start_window.gyro_mean.transpose() << std::endl;

    if (result.has_end_window)
    {
        std::cout << "末尾静止窗口：" << result.end_window.start_sow << " - " << result.end_window.end_sow
                  << "，时长 " << result.end_window.duration
                  << " s，静止比例 " << result.end_window.static_ratio * 100.0 << "%" << std::endl;
    }

    std::cout << std::setprecision(10)
              << "GNSS 初始位置：lat=" << result.start_solution.blh[0] * RAD2DEG
              << " deg, lon=" << result.start_solution.blh[1] * RAD2DEG
              << " deg, h=" << result.start_solution.blh[2] << " m" << std::endl;

    printPvaAttitude("调平+直接罗经对准", result.start_solution.leveling_compass);
    printPvaAttitude("解析法对准", result.start_solution.analytic);
    printPvaAttitude("最终采用姿态", result.start_solution.final_pva);
    std::cout << "结果说明：" << result.start_solution.note << std::endl;

    if (result.has_end_window)
    {
        std::cout << std::setprecision(10)
                  << "GNSS 末端位置：lat=" << result.end_solution.blh[0] * RAD2DEG
                  << " deg, lon=" << result.end_solution.blh[1] * RAD2DEG
                  << " deg, h=" << result.end_solution.blh[2] << " m" << std::endl;
        printPvaAttitude("末端调平+直接罗经对准", result.end_solution.leveling_compass);
        printPvaAttitude("末端解析法对准", result.end_solution.analytic);
        printPvaAttitude("末端最终采用姿态", result.end_solution.final_pva);
        std::cout << "末端结果说明：" << result.end_solution.note << std::endl;
    }
}
