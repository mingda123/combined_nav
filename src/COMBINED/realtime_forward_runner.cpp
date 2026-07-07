#include "realtime_forward_runner.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "COMMON/Attitude.h"
#include "COMMON/const_info.h"
#include "IMU/inerNav.h"

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

struct StartupStaticWindow
{
    size_t start_index = 0;
    size_t end_index = 0;
    double start_sow = 0.0;
    double end_sow = 0.0;
    double duration = 0.0;
    double static_ratio = 0.0;
    int sample_count = 0;
    IMUData imu_mean;
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro_mean = Eigen::Vector3d::Zero();
};

struct StartupAlignmentResult
{
    StartupStaticWindow window;
    Eigen::Vector3d blh = Eigen::Vector3d::Zero();
    PVA analytic_pva;
    PVA final_pva;
    bool use_gnss_yaw_fallback = false;
    double gnss_yaw_rad = 0.0;
    std::string note;
};

double normalizeAngle(double angle)
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

bool findNearestGnss(const std::vector<GNSSData> &gnss_datas, double sow, double max_dt, GNSSData &nearest)
{
    if (gnss_datas.empty())
    {
        return false;
    }

    bool found = false;
    double best_dt = max_dt;
    for (size_t i = 0; i < gnss_datas.size(); ++i)
    {
        const double dt = std::fabs(gnss_datas[i].gpstime.sow - sow);
        if (dt <= best_dt)
        {
            best_dt = dt;
            nearest = gnss_datas[i];
            found = true;
        }
    }

    return found;
}

bool isStaticEpoch(const IMUData &imu, const std::vector<GNSSData> &gnss_datas)
{
    if (imu.dt <= 0.0)
    {
        return false;
    }

    const Eigen::Vector3d acc = imu.dvel / imu.dt;
    const Eigen::Vector3d gyro = imu.dangel / imu.dt;
    if (std::fabs(acc.norm() - G_VAL) >= STATIC_ACC_THRESHOLD || gyro.norm() >= STATIC_GYRO_THRESHOLD)
    {
        return false;
    }

    GNSSData nearest;
    if (findNearestGnss(gnss_datas, imu.gpstime.sow, GNSS_TIME_TOLERANCE, nearest))
    {
        return horizontalSpeed(nearest) < STATIC_GNSS_SPEED_THRESHOLD;
    }

    return true;
}

bool averageGnssBlh(const std::vector<GNSSData> &gnss_datas, double start_sow, double end_sow, Eigen::Vector3d &blh)
{
    blh.setZero();
    int count = 0;
    for (size_t i = 0; i < gnss_datas.size(); ++i)
    {
        if (gnss_datas[i].gpstime.sow < start_sow || gnss_datas[i].gpstime.sow > end_sow)
        {
            continue;
        }
        blh += gnss_datas[i].blh;
        ++count;
    }

    if (count <= 0)
    {
        return false;
    }

    blh /= static_cast<double>(count);
    return true;
}

bool estimateYawFromGnssVelocity(const std::vector<GNSSData> &gnss_datas,
                                 double ref_sow,
                                 double min_horizontal_speed,
                                 double &yaw_rad)
{
    for (size_t i = 0; i < gnss_datas.size(); ++i)
    {
        if (gnss_datas[i].gpstime.sow < ref_sow)
        {
            continue;
        }
        if (horizontalSpeed(gnss_datas[i]) > min_horizontal_speed)
        {
            yaw_rad = std::atan2(gnss_datas[i].vel_n[1], gnss_datas[i].vel_n[0]);
            return true;
        }
    }
    return false;
}

bool hasManualGyroBiasForAlignment(const Eigen::Vector3d &gyro_bias)
{
    return gyro_bias.norm() > 1e-12;
}

void setPvaAttitude(PVA &pva, double yaw, double pitch, double roll)
{
    pva.att.yaw = normalizeAngle(yaw);
    pva.att.pitch = pitch;
    pva.att.roll = roll;
    pva.att.Euler2DCM();
    pva.att.Euler2Quarternion();
}

PVA runLevelingWithYaw(const Eigen::Vector3d &blh, const IMUData &imu_mean, double yaw_rad)
{
    PVA pva;
    pva.blh = blh;
    pva.vel_n.setZero();

    const Eigen::Vector3d fb = -imu_mean.dvel / imu_mean.dt;
    const double roll = std::atan2(fb[1], fb[2]);
    const double pitch = std::atan2(-fb[0], std::sqrt(fb[1] * fb[1] + fb[2] * fb[2]));
    setPvaAttitude(pva, yaw_rad, pitch, roll);
    return pva;
}

bool buildStartupStaticWindow(const std::vector<IMUData> &imu_datas,
                              const std::vector<GNSSData> &gnss_datas,
                              const AlignConfig &cfg,
                              StartupStaticWindow &window)
{
    if (imu_datas.empty())
    {
        return false;
    }

    const double data_start_sow = imu_datas.front().gpstime.sow;
    const double search_end_sow = data_start_sow + cfg.start_search_length;

    size_t index = 0;
    while (index < imu_datas.size() && imu_datas[index].gpstime.sow <= search_end_sow)
    {
        while (index < imu_datas.size() &&
               imu_datas[index].gpstime.sow <= search_end_sow &&
               !isStaticEpoch(imu_datas[index], gnss_datas))
        {
            ++index;
        }
        if (index >= imu_datas.size() || imu_datas[index].gpstime.sow > search_end_sow)
        {
            break;
        }

        const size_t start_index = index;
        size_t end_index = start_index;
        int static_count = 0;
        int valid_count = 0;
        double break_time = 0.0;
        IMUData imu_sum;
        Eigen::Vector3d acc_sum = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyro_sum = Eigen::Vector3d::Zero();

        while (end_index < imu_datas.size() && imu_datas[end_index].gpstime.sow <= search_end_sow)
        {
            const IMUData &imu = imu_datas[end_index];
            if (imu.dt <= 0.0)
            {
                ++end_index;
                continue;
            }

            const bool is_static = isStaticEpoch(imu, gnss_datas);
            if (is_static)
            {
                break_time = 0.0;
            }
            else
            {
                break_time += imu.dt;
                if (break_time > MAX_STATIC_BREAK_TIME)
                {
                    break;
                }
            }

            imu_sum.dvel += imu.dvel;
            imu_sum.dangel += imu.dangel;
            imu_sum.dt += imu.dt;
            imu_sum.gpstime = imu.gpstime;
            imu_sum.frq = imu.frq;
            acc_sum += imu.dvel / imu.dt;
            gyro_sum += imu.dangel / imu.dt;
            ++valid_count;
            static_count += is_static ? 1 : 0;

            const double duration = imu.gpstime.sow - imu_datas[start_index].gpstime.sow;
            const double static_ratio = valid_count > 0 ? static_cast<double>(static_count) / valid_count : 0.0;
            if (duration >= cfg.min_static_duration && static_ratio >= MIN_STATIC_RATIO)
            {
                window = StartupStaticWindow();
                window.start_index = start_index;
                window.end_index = end_index;
                window.start_sow = imu_datas[start_index].gpstime.sow;
                window.end_sow = imu.gpstime.sow;
                window.duration = duration;
                window.static_ratio = static_ratio;
                window.sample_count = valid_count;
                window.imu_mean = imu_sum;
                window.acc_mean = acc_sum / valid_count;
                window.gyro_mean = gyro_sum / valid_count;
                return true;
            }

            ++end_index;
        }

        index = std::max(end_index, start_index + 1);
    }

    return false;
}

bool buildStartupAlignmentResult(const AlignConfig &cfg,
                                 const std::vector<IMUData> &imu_datas,
                                 const std::vector<GNSSData> &gnss_datas,
                                 StartupAlignmentResult &result)
{
    result = StartupAlignmentResult();
    if (!buildStartupStaticWindow(imu_datas, gnss_datas, cfg, result.window))
    {
        return false;
    }

    if (!averageGnssBlh(gnss_datas, result.window.start_sow, result.window.end_sow, result.blh))
    {
        GNSSData nearest;
        if (!findNearestGnss(gnss_datas, result.window.start_sow, GNSS_TIME_TOLERANCE + 5.0, nearest))
        {
            return false;
        }
        result.blh = nearest.blh;
    }

    IMUData corrected_imu = result.window.imu_mean;
    corrected_imu.dvel -= cfg.start_align_acc_bias * corrected_imu.dt;
    corrected_imu.dangel -= cfg.start_align_gyro_bias * corrected_imu.dt;

    InerNav align_nav;
    result.analytic_pva.blh = result.blh;
    result.analytic_pva.vel_n.setZero();
    align_nav.staticCoarseAlign(result.blh[0], corrected_imu, result.analytic_pva);
    result.final_pva = result.analytic_pva;
    result.note = "initialized with analytic static coarse alignment";

    if (!hasManualGyroBiasForAlignment(cfg.start_align_gyro_bias))
    {
        double gnss_yaw_rad = 0.0;
        if (estimateYawFromGnssVelocity(gnss_datas, result.window.end_sow, GNSS_YAW_MIN_SPEED, gnss_yaw_rad))
        {
            const double yaw_diff = std::fabs(normalizeAngle(result.analytic_pva.att.yaw - gnss_yaw_rad));
            if (yaw_diff > YAW_FALLBACK_THRESHOLD)
            {
                result.use_gnss_yaw_fallback = true;
                result.gnss_yaw_rad = gnss_yaw_rad;
                result.final_pva = runLevelingWithYaw(result.blh, corrected_imu, gnss_yaw_rad);
                result.note = "analytic yaw deviates from GNSS course, fallback to leveling + GNSS yaw";
            }
        }
    }

    return true;
}

IntegratedNavState buildForwardInitState(const AlignConfig &align_cfg, const StartupAlignmentResult &align_result)
{
    IntegratedNavState init_state;
    init_state.pva = align_result.final_pva;
    init_state.imu_bias.acc_bias = align_cfg.start_align_acc_bias;
    init_state.imu_bias.gyro_bias = align_cfg.start_align_gyro_bias;
    init_state.imu_scale.acc_scale.setZero();
    init_state.imu_scale.gyro_scale.setZero();
    return init_state;
}

void printStartupAlignmentResult(const StartupAlignmentResult &result)
{
    std::cout << std::fixed << std::setprecision(3)
              << "Startup static window: " << result.window.start_sow
              << " - " << result.window.end_sow
              << " s, duration=" << result.window.duration
              << " s, static_ratio=" << result.window.static_ratio * 100.0 << "%" << std::endl;
    std::cout << std::setprecision(10)
              << "Startup BLH: lat=" << result.blh[0] * RAD2DEG
              << " deg, lon=" << result.blh[1] * RAD2DEG
              << " deg, h=" << result.blh[2] << " m" << std::endl;
    std::cout << std::setprecision(6)
              << "Analytic attitude: yaw=" << result.analytic_pva.att.yaw * RAD2DEG
              << " deg, pitch=" << result.analytic_pva.att.pitch * RAD2DEG
              << " deg, roll=" << result.analytic_pva.att.roll * RAD2DEG << " deg" << std::endl;
    std::cout << "Final init attitude: yaw=" << result.final_pva.att.yaw * RAD2DEG
              << " deg, pitch=" << result.final_pva.att.pitch * RAD2DEG
              << " deg, roll=" << result.final_pva.att.roll * RAD2DEG << " deg" << std::endl;
    std::cout << "Alignment note: " << result.note << std::endl;
}

class RealtimeResultWriter
{
public:
    bool open(const std::string &output_file)
    {
        stream_.open(output_file.c_str(), std::ios::out | std::ios::trunc);
        if (!stream_.is_open())
        {
            std::cerr << "Error: cannot open output file " << output_file << std::endl;
            return false;
        }

        stream_ << "week,sow,lat_deg,lon_deg,h_m,vn,ve,vd,roll_deg,pitch_deg,yaw_deg,"
                   "bgx,bgy,bgz,bax,bay,baz,sgx,sgy,sgz,sax,say,saz\n";
        return true;
    }

    void append(const GPSTime &time, const IntegratedNavState &state)
    {
        if (!stream_.is_open())
        {
            return;
        }

        stream_ << std::fixed << std::setprecision(10)
                << time.week << ","
                << time.sow << ","
                << state.pva.blh[0] * RAD2DEG << ","
                << state.pva.blh[1] * RAD2DEG << ","
                << state.pva.blh[2] << ","
                << state.pva.vel_n[0] << ","
                << state.pva.vel_n[1] << ","
                << state.pva.vel_n[2] << ","
                << state.pva.att.roll * RAD2DEG << ","
                << state.pva.att.pitch * RAD2DEG << ","
                << state.pva.att.yaw * RAD2DEG << ","
                << state.imu_bias.gyro_bias[0] << ","
                << state.imu_bias.gyro_bias[1] << ","
                << state.imu_bias.gyro_bias[2] << ","
                << state.imu_bias.acc_bias[0] << ","
                << state.imu_bias.acc_bias[1] << ","
                << state.imu_bias.acc_bias[2] << ","
                << state.imu_scale.gyro_scale[0] << ","
                << state.imu_scale.gyro_scale[1] << ","
                << state.imu_scale.gyro_scale[2] << ","
                << state.imu_scale.acc_scale[0] << ","
                << state.imu_scale.acc_scale[1] << ","
                << state.imu_scale.acc_scale[2] << "\n";
    }

private:
    std::ofstream stream_;
};

class RealtimeForwardLciRunner
{
public:
    RealtimeForwardLciRunner(const AlignConfig &align_cfg,
                             const LciFilterConfig &lci_cfg,
                             const RealtimeStreamConfig &stream_cfg,
                             const RealtimeForwardLciConfig &runtime_cfg)
        : align_cfg_(align_cfg),
          lci_cfg_(lci_cfg),
          stream_cfg_(stream_cfg),
          runtime_cfg_(runtime_cfg),
          filter_(lci_cfg)
    {
    }

    bool run()
    {
        std::string error_message;
        if (!receiver_.open(stream_cfg_, error_message))
        {
            std::cerr << "Error: " << error_message << std::endl;
            return false;
        }

        std::cout << "Realtime forward LCI started." << std::endl;
        std::cout << "Protocol=" << (stream_cfg_.transport == RealtimeTransport::UDP ? "UDP" : "TCP")
                  << ", bind=" << stream_cfg_.bind_host
                  << ":" << stream_cfg_.port << std::endl;
        std::cout << "Expected IMU format: IMU week sow gx_deg_s gy_deg_s gz_deg_s ax ay az" << std::endl;
        std::cout << "Expected GNSS format: GNSS week sow lat_deg lon_deg h pos_std_n pos_std_e pos_std_d vn ve vu vel_std_n vel_std_e vel_std_u" << std::endl;

        for (;;)
        {
            std::vector<std::string> lines;
            if (!receiver_.pollLines(lines, error_message))
            {
                std::cerr << "Error: " << error_message << std::endl;
                return false;
            }

            for (size_t i = 0; i < lines.size(); ++i)
            {
                RealtimeRecord record;
                if (!parseRealtimeRecord(lines[i], align_cfg_.imu_sample_rate, current_week_, record, error_message))
                {
                    if (error_message != "empty line")
                    {
                        std::cerr << "Warning: drop malformed stream line: " << lines[i] << std::endl;
                    }
                    continue;
                }

                if (record.type == RealtimeRecord::GNSS)
                {
                    handleGnss(record.gnss);
                }
                else if (record.type == RealtimeRecord::IMU)
                {
                    handleImu(record.imu);
                }
            }
        }
    }

private:
    void preprocessImu(IMUData &imu)
    {
        imu_transformer_.compensate(imu, align_cfg_.imu_need_trans);
    }

    void handleGnss(const GNSSData &gnss)
    {
        if (gnss.gpstime.week > 0)
        {
            current_week_ = gnss.gpstime.week;
        }

        if (!initialized_)
        {
            init_gnss_cache_.push_back(gnss);
            return;
        }

        if (gnss.gpstime.sow + lci_cfg_.gnss_time_tolerance < current_solution_sow_)
        {
            std::cerr << "Warning: drop late GNSS epoch at sow=" << gnss.gpstime.sow << std::endl;
            return;
        }

        pending_gnss_.push_back(gnss);
    }

    void handleImu(IMUData imu)
    {
        if (imu.gpstime.week > 0)
        {
            current_week_ = imu.gpstime.week;
        }
        else
        {
            imu.gpstime.week = current_week_;
        }

        preprocessImu(imu);

        if (!initialized_)
        {
            init_imu_cache_.push_back(imu);
            ++imu_since_alignment_attempt_;
            if (imu_since_alignment_attempt_ >= static_cast<size_t>(std::max(1, runtime_cfg_.alignment_check_interval)))
            {
                imu_since_alignment_attempt_ = 0;
                tryInitialize();
            }
            return;
        }

        processPendingGnss(imu.gpstime.sow);
        filter_.addImuData(imu);
        filter_.processCurrentEpoch(filter_result_);
        emitCurrentState(imu.gpstime);
    }

    void processPendingGnss(double imu_sow)
    {
        while (!pending_gnss_.empty() &&
               pending_gnss_.front().gpstime.sow + lci_cfg_.gnss_time_tolerance < current_solution_sow_)
        {
            std::cerr << "Warning: drop stale GNSS epoch at sow=" << pending_gnss_.front().gpstime.sow << std::endl;
            pending_gnss_.pop_front();
        }

        if (!pending_gnss_.empty() &&
            pending_gnss_.front().gpstime.sow <= imu_sow + lci_cfg_.gnss_time_tolerance)
        {
            filter_.addGnssData(pending_gnss_.front());
            pending_gnss_.pop_front();
        }
    }

    void emitCurrentState(const GPSTime &fallback_time)
    {
        if (filter_.timestamp() <= 0.0)
        {
            return;
        }

        GPSTime output_time = fallback_time;
        output_time.sow = filter_.timestamp();
        const IntegratedNavState state = filter_.getState();
        writer_.append(output_time, state);
        current_solution_sow_ = output_time.sow;
        ++output_count_;

        if (runtime_cfg_.print_interval > 0 && (output_count_ % static_cast<size_t>(runtime_cfg_.print_interval) == 0))
        {
            std::cout << std::fixed << std::setprecision(3)
                      << "RT LCI sow=" << output_time.sow
                      << ", lat=" << state.pva.blh[0] * RAD2DEG
                      << ", lon=" << state.pva.blh[1] * RAD2DEG
                      << ", h=" << state.pva.blh[2]
                      << ", vn=" << state.pva.vel_n[0]
                      << ", ve=" << state.pva.vel_n[1]
                      << ", vd=" << state.pva.vel_n[2] << std::endl;
        }
    }

    bool tryInitialize()
    {
        if (initialized_ || init_imu_cache_.size() < 2 || init_gnss_cache_.empty())
        {
            return false;
        }

        const double buffered_duration = init_imu_cache_.back().gpstime.sow - init_imu_cache_.front().gpstime.sow;
        if (buffered_duration < align_cfg_.min_static_duration)
        {
            return false;
        }

        StartupAlignmentResult align_result;
        if (!buildStartupAlignmentResult(align_cfg_, init_imu_cache_, init_gnss_cache_, align_result))
        {
            return false;
        }

        const IntegratedNavState init_state = buildForwardInitState(align_cfg_, align_result);
        size_t imu_start_index = 0;
        while (imu_start_index < init_imu_cache_.size() &&
               init_imu_cache_[imu_start_index].gpstime.sow < align_result.window.end_sow)
        {
            ++imu_start_index;
        }
        if (imu_start_index >= init_imu_cache_.size())
        {
            return false;
        }

        if (!writer_.open(runtime_cfg_.output_file))
        {
            return false;
        }

        filter_.initialize(init_state);
        filter_result_ = LciFilterResult();
        filter_result_.valid = true;
        filter_result_.start_sow = init_imu_cache_[imu_start_index].gpstime.sow;
        filter_result_.end_sow = filter_result_.start_sow;
        current_solution_sow_ = init_imu_cache_[imu_start_index].gpstime.sow;

        writer_.append(init_imu_cache_[imu_start_index].gpstime, init_state);
        output_count_ = 1;
        initialized_ = true;

        printStartupAlignmentResult(align_result);
        std::cout << "Realtime forward LCI initialized, replaying buffered IMU epochs..." << std::endl;

        size_t gnss_index = 0;
        while (gnss_index < init_gnss_cache_.size() &&
               init_gnss_cache_[gnss_index].gpstime.sow < align_result.window.end_sow)
        {
            ++gnss_index;
        }

        for (size_t i = imu_start_index; i < init_imu_cache_.size(); ++i)
        {
            while (gnss_index < init_gnss_cache_.size() &&
                   init_gnss_cache_[gnss_index].gpstime.sow <= init_imu_cache_[i].gpstime.sow + lci_cfg_.gnss_time_tolerance)
            {
                filter_.addGnssData(init_gnss_cache_[gnss_index]);
                ++gnss_index;
                break;
            }

            filter_.addImuData(init_imu_cache_[i]);
            filter_.processCurrentEpoch(filter_result_);
            emitCurrentState(init_imu_cache_[i].gpstime);
        }

        while (gnss_index < init_gnss_cache_.size())
        {
            pending_gnss_.push_back(init_gnss_cache_[gnss_index]);
            ++gnss_index;
        }

        init_imu_cache_.clear();
        init_gnss_cache_.clear();
        std::cout << "Realtime forward LCI switched to live mode." << std::endl;
        return true;
    }

private:
    AlignConfig align_cfg_;
    LciFilterConfig lci_cfg_;
    RealtimeStreamConfig stream_cfg_;
    RealtimeForwardLciConfig runtime_cfg_;
    RealtimeLineReceiver receiver_;
    InerNav imu_transformer_;
    LciFilter filter_;
    LciFilterResult filter_result_;
    RealtimeResultWriter writer_;
    std::vector<IMUData> init_imu_cache_;
    std::vector<GNSSData> init_gnss_cache_;
    std::deque<GNSSData> pending_gnss_;
    bool initialized_ = false;
    int current_week_ = 0;
    double current_solution_sow_ = 0.0;
    size_t output_count_ = 0;
    size_t imu_since_alignment_attempt_ = 0;
};
} // namespace

bool runRealtimeForwardLci(const AlignConfig &align_cfg,
                           const LciFilterConfig &lci_cfg,
                           const RealtimeStreamConfig &stream_cfg,
                           const RealtimeForwardLciConfig &realtime_cfg)
{
    RealtimeForwardLciRunner runner(align_cfg, lci_cfg, stream_cfg, realtime_cfg);
    return runner.run();
}
