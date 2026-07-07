#include "lci_runner.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

namespace
{
struct ReverseImuEpoch
{
    IMUData pseudo_imu;
    GPSTime physical_time;
};

struct ReverseGnssEpoch
{
    GNSSData pseudo_gnss;
};

bool findFirstGnssAfterTime(const std::vector<GNSSData> &gnss_datas, double sow, size_t &index)
{
    for (size_t i = 0; i < gnss_datas.size(); ++i)
    {
        if (gnss_datas[i].gpstime.sow >= sow)
        {
            index = i;
            return true;
        }
    }
    return false;
}

IntegratedNavState buildForwardInitState(const AlignConfig &align_cfg, const AlignResult &align_result)
{
    IntegratedNavState init_state;
    init_state.pva = align_result.start_solution.final_pva;
    init_state.imu_bias.acc_bias = align_cfg.start_align_acc_bias;
    init_state.imu_bias.gyro_bias = align_cfg.start_align_gyro_bias;
    init_state.imu_scale.acc_scale.setZero();
    init_state.imu_scale.gyro_scale.setZero();
    return init_state;
}

IntegratedNavState buildBackwardInitState(const AlignConfig &align_cfg,
                                          const AlignResult &align_result,
                                          const IntegratedNavState *imu_init_reference)
{
    IntegratedNavState init_state;
    if (imu_init_reference != nullptr)
    {
        init_state = *imu_init_reference;
    }
    else
    {
        init_state.pva = align_result.end_solution.final_pva;
        init_state.imu_bias.acc_bias = align_cfg.end_align_acc_bias;
        init_state.imu_bias.gyro_bias = align_cfg.end_align_gyro_bias;
        init_state.imu_scale.acc_scale.setZero();
        init_state.imu_scale.gyro_scale.setZero();
    }
    return init_state;
}

IntegratedNavState buildBackwardPseudoInitState(const IntegratedNavState &physical_state)
{
    IntegratedNavState pseudo_state = physical_state;
    pseudo_state.pva.vel_n = -physical_state.pva.vel_n;
    return pseudo_state;
}

IntegratedNavState exportBackwardPhysicalState(const IntegratedNavState &pseudo_state)
{
    IntegratedNavState physical_state = pseudo_state;
    physical_state.pva.vel_n = -pseudo_state.pva.vel_n;
    return physical_state;
}

void reverseTrajectoryOrder(LciTrajectory &trajectory)
{
    std::reverse(trajectory.states.begin(), trajectory.states.end());
    std::reverse(trajectory.times.begin(), trajectory.times.end());
    std::reverse(trajectory.covariances.begin(), trajectory.covariances.end());
    std::swap(trajectory.result.start_sow, trajectory.result.end_sow);
}

double covarianceWeight(double variance)
{
    const double min_variance = 1e-8;
    return 1.0 / std::max(variance, min_variance);
}

size_t findFirstImuAtOrAfter(const std::vector<IMUData> &imu_datas, double sow)
{
    size_t index = 0;
    while (index < imu_datas.size() && imu_datas[index].gpstime.sow < sow)
    {
        ++index;
    }
    return index;
}

size_t findLastImuAtOrBefore(const std::vector<IMUData> &imu_datas, double sow)
{
    if (imu_datas.empty())
    {
        return 0;
    }

    size_t index = imu_datas.size() - 1;
    while (index > 0 && imu_datas[index].gpstime.sow > sow)
    {
        --index;
    }
    return index;
}

double computePseudoSow(double physical_sow, double physical_end_sow, double pseudo_base_sow)
{
    return pseudo_base_sow + (physical_end_sow - physical_sow);
}

void buildBackwardImuSequence(const std::vector<IMUData> &raw_imu_datas,
                              size_t begin_index,
                              size_t end_index,
                              double pseudo_base_sow,
                              std::vector<ReverseImuEpoch> &reversed_imu)
{
    reversed_imu.clear();
    if (begin_index > end_index || end_index >= raw_imu_datas.size())
    {
        return;
    }

    // 必须用全局最后 IMU 时间（pseudo_base_sow）做基准，
    // 否则两段 IMU 的伪时间会从同一值开始，无法连续衔接
    const double physical_end_sow = pseudo_base_sow;
    reversed_imu.reserve(end_index - begin_index + 1);

    for (size_t physical_index = end_index + 1; physical_index-- > begin_index;)
    {
        ReverseImuEpoch epoch;
        epoch.physical_time = raw_imu_datas[physical_index].gpstime;
        epoch.pseudo_imu = raw_imu_datas[physical_index];
        epoch.pseudo_imu.gpstime.sow = computePseudoSow(epoch.physical_time.sow, physical_end_sow, pseudo_base_sow);

        if (physical_index == end_index)
        {
            epoch.pseudo_imu.dangel = raw_imu_datas[physical_index].dangel;
            epoch.pseudo_imu.dvel = raw_imu_datas[physical_index].dvel;
        }
        else
        {
            const IMUData &forward_interval = raw_imu_datas[physical_index + 1];
            epoch.pseudo_imu.dangel = forward_interval.dangel;
            epoch.pseudo_imu.dvel = forward_interval.dvel;
            epoch.pseudo_imu.frq = forward_interval.frq;
        }

        reversed_imu.push_back(epoch);

        if (physical_index == begin_index)
        {
            break;
        }
    }

    for (size_t i = 1; i < reversed_imu.size(); ++i)
    {
        reversed_imu[i].pseudo_imu.dt = reversed_imu[i].pseudo_imu.gpstime.sow - reversed_imu[i - 1].pseudo_imu.gpstime.sow;
    }
}

void buildBackwardGnssSequence(const std::vector<GNSSData> &gnss_datas,
                               double lower_sow,
                               double upper_sow,
                               double physical_end_sow,
                               double pseudo_base_sow,
                               std::vector<ReverseGnssEpoch> &reversed_gnss)
{
    reversed_gnss.clear();
    reversed_gnss.reserve(gnss_datas.size());

    for (size_t i = gnss_datas.size(); i-- > 0;)
    {
        if (gnss_datas[i].gpstime.sow < lower_sow || gnss_datas[i].gpstime.sow > upper_sow)
        {
            continue;
        }

        ReverseGnssEpoch epoch;
        epoch.pseudo_gnss = gnss_datas[i];
        epoch.pseudo_gnss.gpstime.sow = computePseudoSow(gnss_datas[i].gpstime.sow, physical_end_sow, pseudo_base_sow);
        epoch.pseudo_gnss.vel_n = -gnss_datas[i].vel_n;
        reversed_gnss.push_back(epoch);
    }
}

void appendBackwardState(const LciFilter &filter,
                         const ReverseImuEpoch &imu_epoch,
                         LciTrajectory &trajectory)
{
    if (filter.timestamp() <= 0.0)
    {
        return;
    }

    trajectory.states.push_back(exportBackwardPhysicalState(filter.getState()));
    trajectory.times.push_back(imu_epoch.physical_time);
    trajectory.covariances.push_back(filter.getCovariance());
}

void runBackwardSegment(LciFilter &filter,
                        const LciFilterConfig &lci_cfg,
                        const std::vector<ReverseImuEpoch> &reversed_imu,
                        const std::vector<ReverseGnssEpoch> &reversed_gnss,
                        bool seed_with_first_imu,
                        double previous_pseudo_sow,
                        LciTrajectory &trajectory)
{
    if (reversed_imu.empty())
    {
        return;
    }

    size_t imu_index = 0;
    double imu_time_prev = previous_pseudo_sow;
    if (seed_with_first_imu)
    {
        filter.addImuData(reversed_imu.front().pseudo_imu);
        imu_index = 1;
        imu_time_prev = reversed_imu.front().pseudo_imu.gpstime.sow;
    }

    size_t gnss_index = 0;
    for (; imu_index < reversed_imu.size(); ++imu_index)
    {
        const double imu_time_prev_raw = imu_time_prev;
        const double imu_time_cur = reversed_imu[imu_index].pseudo_imu.gpstime.sow;
        const double lower_time = std::min(imu_time_prev_raw, imu_time_cur);
        const double upper_time = std::max(imu_time_prev_raw, imu_time_cur);

        while (gnss_index < reversed_gnss.size() &&
               reversed_gnss[gnss_index].pseudo_gnss.gpstime.sow < lower_time - lci_cfg.gnss_time_tolerance)
        {
            ++gnss_index;
        }
        if (gnss_index < reversed_gnss.size())
        {
            const double gnss_time = reversed_gnss[gnss_index].pseudo_gnss.gpstime.sow;
            if (gnss_time <= upper_time + lci_cfg.gnss_time_tolerance &&
                gnss_time >= lower_time - lci_cfg.gnss_time_tolerance)
            {
                filter.addGnssData(reversed_gnss[gnss_index].pseudo_gnss);
                ++gnss_index;
            }
        }

        filter.addImuData(reversed_imu[imu_index].pseudo_imu);
        filter.processCurrentEpoch(trajectory.result);
        appendBackwardState(filter, reversed_imu[imu_index], trajectory);

        imu_time_prev = imu_time_cur;
    }
}
} // namespace

bool loadLciRawData(const AlignConfig &align_cfg,
                    std::vector<IMUData> &raw_imu_datas,
                    std::vector<IMUData> &align_imu_datas,
                    std::vector<GNSSData> &gnss_datas)
{
    raw_imu_datas.clear();
    align_imu_datas.clear();
    gnss_datas.clear();

    FileLoader raw_loader(align_cfg.imu_file.c_str());
    if (!raw_loader.readAll(raw_imu_datas, align_cfg.imu_sample_rate))
    {
        std::cerr << "Error: failed to load raw IMU data." << std::endl;
        return false;
    }

    if (!loadAlignInputData(align_cfg, align_imu_datas, gnss_datas))
    {
        return false;
    }

    if (raw_imu_datas.size() != align_imu_datas.size())
    {
        std::cerr << "Error: raw IMU data count does not match alignment IMU data count." << std::endl;
        return false;
    }

    return true;
}

void saveLciResult(const std::string &output_file,
                   const std::vector<IntegratedNavState> &states,
                   const std::vector<GPSTime> &times)
{
    std::ofstream ofs(output_file.c_str());
    if (!ofs.is_open())
    {
        std::cerr << "Error: cannot write result file " << output_file << std::endl;
        return;
    }

    ofs << "week,sow,lat_deg,lon_deg,h_m,vn,ve,vd,roll_deg,pitch_deg,yaw_deg,"
           "bgx,bgy,bgz,bax,bay,baz,sgx,sgy,sgz,sax,say,saz\n";
    ofs << std::fixed << std::setprecision(10);

    const size_t count = std::min(states.size(), times.size());
    for (size_t i = 0; i < count; ++i)
    {
        const IntegratedNavState &state = states[i];
        const GPSTime &time = times[i];
        ofs << time.week << ","
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
}

bool runForwardLci(const AlignConfig &align_cfg,
                   const LciFilterConfig &lci_cfg,
                   const std::vector<IMUData> &raw_imu_datas,
                   const std::vector<IMUData> &align_imu_datas,
                   const std::vector<GNSSData> &gnss_datas,
                   LciTrajectory &trajectory)
{
    trajectory = LciTrajectory();
    if (raw_imu_datas.empty() || align_imu_datas.empty() || gnss_datas.empty())
    {
        std::cerr << "Error: forward LCI input data is empty." << std::endl;
        return false;
    }

    AlignResult align_result;
    if (!runInitialAlignment(align_cfg, align_imu_datas, gnss_datas, align_result))
    {
        return false;
    }

    IntegratedNavState init_state = buildForwardInitState(align_cfg, align_result);
    trajectory.init_state = init_state;
    trajectory.init_sow = align_result.start_window.end_sow;

    size_t imu_start_index = 0;
    while (imu_start_index < raw_imu_datas.size() &&
           raw_imu_datas[imu_start_index].gpstime.sow < trajectory.init_sow)
    {
        ++imu_start_index;
    }
    if (imu_start_index >= raw_imu_datas.size())
    {
        std::cerr << "Error: forward LCI could not find IMU data after initial alignment." << std::endl;
        return false;
    }

    size_t gnss_index = 0;
    findFirstGnssAfterTime(gnss_datas, trajectory.init_sow, gnss_index);

    LciFilter filter(lci_cfg);
    filter.initialize(init_state);

    trajectory.result.valid = true;
    trajectory.result.start_sow = raw_imu_datas[imu_start_index].gpstime.sow;
    trajectory.result.end_sow = trajectory.result.start_sow;
    trajectory.states.push_back(init_state);
    trajectory.times.push_back(raw_imu_datas[imu_start_index].gpstime);
    trajectory.covariances.push_back(filter.getCovariance());

    for (size_t i = imu_start_index; i < raw_imu_datas.size(); ++i)
    {
        if (gnss_index < gnss_datas.size() &&
            gnss_datas[gnss_index].gpstime.sow <= raw_imu_datas[i].gpstime.sow + lci_cfg.gnss_time_tolerance)
        {
            filter.addGnssData(gnss_datas[gnss_index]);
            ++gnss_index;
        }

        filter.addImuData(raw_imu_datas[i]);
        filter.processCurrentEpoch(trajectory.result);

        if (filter.timestamp() > 0.0)
        {
            trajectory.states.push_back(filter.getState());
            GPSTime current_time = raw_imu_datas[i].gpstime;
            current_time.sow = filter.timestamp();
            trajectory.times.push_back(current_time);
            trajectory.covariances.push_back(filter.getCovariance());
        }
    }

    if (trajectory.states.empty())
    {
        std::cerr << "Error: forward LCI did not produce valid states." << std::endl;
        return false;
    }

    trajectory.result.final_state = trajectory.states.back();
    trajectory.result.final_covariance = trajectory.covariances.back();
    return true;
}

bool runBackwardLci(const AlignConfig &align_cfg,
                    const LciFilterConfig &lci_cfg,
                    const std::vector<IMUData> &raw_imu_datas,
                    const std::vector<IMUData> &align_imu_datas,
                    const std::vector<GNSSData> &gnss_datas,
                    const IntegratedNavState *imu_init_reference,
                    LciTrajectory &trajectory)
{
    trajectory = LciTrajectory();
    if (raw_imu_datas.empty() || align_imu_datas.empty() || gnss_datas.empty())
    {
        std::cerr << "Error: backward LCI input data is empty." << std::endl;
        return false;
    }

    AlignResult align_result;
    if (!runInitialAlignment(align_cfg, align_imu_datas, gnss_datas, align_result))
    {
        return false;
    }
    if (!align_result.has_end_window)
    {
        std::cerr << "Error: backward LCI requires a valid static window at the end." << std::endl;
        return false;
    }

    const IntegratedNavState physical_init_state = buildBackwardInitState(align_cfg, align_result, imu_init_reference);
    const IntegratedNavState pseudo_init_state = buildBackwardPseudoInitState(physical_init_state);
    trajectory.init_state = physical_init_state;

    const double backward_start_sow = align_result.start_window.end_sow;
    const double backward_align_start_sow = align_result.end_window.start_sow;
    const double backward_end_sow = align_result.end_window.end_sow;
    if (backward_align_start_sow <= backward_start_sow)
    {
        std::cerr << "Error: backward alignment window overlaps the earlier reverse-propagation interval." << std::endl;
        return false;
    }

    const size_t imu_begin_index = findFirstImuAtOrAfter(raw_imu_datas, backward_start_sow);
    if (imu_begin_index >= raw_imu_datas.size())
    {
        std::cerr << "Error: backward LCI could not find IMU data after initial alignment." << std::endl;
        return false;
    }

    const size_t imu_end_index = findLastImuAtOrBefore(raw_imu_datas, backward_end_sow);
    if (imu_end_index <= imu_begin_index)
    {
        std::cerr << "Error: backward LCI does not have enough IMU data for reverse propagation." << std::endl;
        return false;
    }

    const size_t imu_align_begin_index = findFirstImuAtOrAfter(raw_imu_datas, backward_align_start_sow);
    if (imu_align_begin_index >= raw_imu_datas.size() || imu_align_begin_index > imu_end_index)
    {
        std::cerr << "Error: could not find IMU data for backward alignment segment." << std::endl;
        return false;
    }
    if (imu_end_index <= imu_align_begin_index)
    {
        std::cerr << "Error: backward alignment segment is too short." << std::endl;
        return false;
    }

    const double pseudo_base_sow = raw_imu_datas[imu_end_index].gpstime.sow;
    trajectory.init_sow = raw_imu_datas[imu_end_index].gpstime.sow;

    std::vector<ReverseImuEpoch> reversed_align_imu;
    std::vector<ReverseImuEpoch> reversed_main_imu;
    std::vector<ReverseGnssEpoch> reversed_align_gnss;
    std::vector<ReverseGnssEpoch> reversed_main_gnss;
    buildBackwardImuSequence(raw_imu_datas, imu_align_begin_index, imu_end_index, pseudo_base_sow, reversed_align_imu);
    if (imu_align_begin_index > imu_begin_index)
    {
        buildBackwardImuSequence(raw_imu_datas, imu_begin_index, imu_align_begin_index - 1, pseudo_base_sow, reversed_main_imu);
    }
    buildBackwardGnssSequence(gnss_datas,
                              backward_align_start_sow,
                              backward_end_sow,
                              raw_imu_datas[imu_end_index].gpstime.sow,
                              pseudo_base_sow,
                              reversed_align_gnss);
    buildBackwardGnssSequence(gnss_datas,
                              backward_start_sow,
                              raw_imu_datas[imu_align_begin_index].gpstime.sow - lci_cfg.gnss_time_tolerance,
                              raw_imu_datas[imu_end_index].gpstime.sow,
                              pseudo_base_sow,
                              reversed_main_gnss);

    LciFilter filter(lci_cfg);
    filter.setDirection(LciFilter::BACKWARD);
    filter.initialize(pseudo_init_state);

    trajectory.result.valid = true;
    trajectory.result.start_sow = reversed_align_imu.front().physical_time.sow;
    trajectory.result.end_sow = reversed_align_imu.front().physical_time.sow;
    trajectory.states.push_back(physical_init_state);
    trajectory.times.push_back(reversed_align_imu.front().physical_time);
    trajectory.covariances.push_back(filter.getCovariance());

    runBackwardSegment(filter,
                       lci_cfg,
                       reversed_align_imu,
                       reversed_align_gnss,
                       true,
                       reversed_align_imu.front().pseudo_imu.gpstime.sow,
                       trajectory);
    if (!reversed_main_imu.empty())
    {
        runBackwardSegment(filter,
                           lci_cfg,
                           reversed_main_imu,
                           reversed_main_gnss,
                           false,
                           reversed_main_imu.front().pseudo_imu.gpstime.sow - reversed_main_imu.front().pseudo_imu.dt,
                           trajectory);
    }

    if (trajectory.states.empty())
    {
        std::cerr << "Error: backward LCI did not produce valid states." << std::endl;
        return false;
    }

    trajectory.result.note = "Backward LCI uses an independent pseudo-time implementation: reverse alignment first, then reverse epoch-by-epoch propagation.";
    reverseTrajectoryOrder(trajectory);
    if (!trajectory.times.empty())
    {
        trajectory.result.start_sow = trajectory.times.front().sow;
        trajectory.result.end_sow = trajectory.times.back().sow;
    }
    trajectory.result.final_state = trajectory.states.back();
    trajectory.result.final_covariance = trajectory.covariances.back();
    return true;
}

bool fuseForwardBackwardLci(const LciTrajectory &forward_trajectory,
                            const LciTrajectory &backward_trajectory,
                            LciTrajectory &fused_trajectory)
{
    fused_trajectory = LciTrajectory();
    if (forward_trajectory.states.empty() || backward_trajectory.states.empty())
    {
        return false;
    }

    const size_t count = std::min(forward_trajectory.states.size(), backward_trajectory.states.size());
    fused_trajectory.states.reserve(count);
    fused_trajectory.times.reserve(count);
    fused_trajectory.covariances.reserve(count);
    fused_trajectory.init_state = forward_trajectory.init_state;
    fused_trajectory.init_sow = forward_trajectory.init_sow;
    fused_trajectory.result.valid = true;
    fused_trajectory.result.start_sow = forward_trajectory.result.start_sow;
    fused_trajectory.result.end_sow = forward_trajectory.result.end_sow;
    fused_trajectory.result.imu_epochs = static_cast<int>(count);

    for (size_t i = 0; i < count; ++i)
    {
        const IntegratedNavState &forward_state = forward_trajectory.states[i];
        const IntegratedNavState &backward_state = backward_trajectory.states[i];
        const Matrix21d &forward_cov = forward_trajectory.covariances[i];
        const Matrix21d &backward_cov = backward_trajectory.covariances[i];

        IntegratedNavState fused_state = forward_state;
        Matrix21d fused_cov = forward_cov;

        for (int j = 0; j < 3; ++j)
        {
            double wp = covarianceWeight(forward_cov(ERR_POS_ID + j, ERR_POS_ID + j));
            double wb = covarianceWeight(backward_cov(ERR_POS_ID + j, ERR_POS_ID + j));
            fused_state.pva.blh[j] = (forward_state.pva.blh[j] * wp + backward_state.pva.blh[j] * wb) / (wp + wb);

            wp = covarianceWeight(forward_cov(ERR_VEL_ID + j, ERR_VEL_ID + j));
            wb = covarianceWeight(backward_cov(ERR_VEL_ID + j, ERR_VEL_ID + j));
            fused_state.pva.vel_n[j] = (forward_state.pva.vel_n[j] * wp + backward_state.pva.vel_n[j] * wb) / (wp + wb);

            wp = covarianceWeight(forward_cov(ERR_GYRO_BIAS_ID + j, ERR_GYRO_BIAS_ID + j));
            wb = covarianceWeight(backward_cov(ERR_GYRO_BIAS_ID + j, ERR_GYRO_BIAS_ID + j));
            fused_state.imu_bias.gyro_bias[j] = (forward_state.imu_bias.gyro_bias[j] * wp + backward_state.imu_bias.gyro_bias[j] * wb) / (wp + wb);

            wp = covarianceWeight(forward_cov(ERR_ACC_BIAS_ID + j, ERR_ACC_BIAS_ID + j));
            wb = covarianceWeight(backward_cov(ERR_ACC_BIAS_ID + j, ERR_ACC_BIAS_ID + j));
            fused_state.imu_bias.acc_bias[j] = (forward_state.imu_bias.acc_bias[j] * wp + backward_state.imu_bias.acc_bias[j] * wb) / (wp + wb);

            wp = covarianceWeight(forward_cov(ERR_GYRO_SCALE_ID + j, ERR_GYRO_SCALE_ID + j));
            wb = covarianceWeight(backward_cov(ERR_GYRO_SCALE_ID + j, ERR_GYRO_SCALE_ID + j));
            fused_state.imu_scale.gyro_scale[j] = (forward_state.imu_scale.gyro_scale[j] * wp + backward_state.imu_scale.gyro_scale[j] * wb) / (wp + wb);

            wp = covarianceWeight(forward_cov(ERR_ACC_SCALE_ID + j, ERR_ACC_SCALE_ID + j));
            wb = covarianceWeight(backward_cov(ERR_ACC_SCALE_ID + j, ERR_ACC_SCALE_ID + j));
            fused_state.imu_scale.acc_scale[j] = (forward_state.imu_scale.acc_scale[j] * wp + backward_state.imu_scale.acc_scale[j] * wb) / (wp + wb);
        }

        fused_state.pva.att = forward_state.pva.att;
        fused_trajectory.states.push_back(fused_state);
        fused_trajectory.times.push_back(forward_trajectory.times[i]);
        fused_trajectory.covariances.push_back(fused_cov);
    }

    fused_trajectory.result.final_state = fused_trajectory.states.back();
    fused_trajectory.result.final_covariance = fused_trajectory.covariances.back();
    fused_trajectory.result.gnss_pos_updates = forward_trajectory.result.gnss_pos_updates;
    fused_trajectory.result.gnss_vel_updates = forward_trajectory.result.gnss_vel_updates;
    fused_trajectory.result.zupt_updates = forward_trajectory.result.zupt_updates;
    fused_trajectory.result.nhc_updates = forward_trajectory.result.nhc_updates;
    return true;
}

void printLciSummary(const char *title, const LciTrajectory &trajectory)
{
    if (!trajectory.result.valid || trajectory.states.empty())
    {
        std::cout << title << ": no valid result." << std::endl;
        return;
    }

    const IntegratedNavState &final_state = trajectory.states.back();
    std::cout << title << ":" << std::endl;
    std::cout << std::fixed << std::setprecision(3)
              << "  start sow: " << trajectory.result.start_sow << " s" << std::endl
              << "  end sow: " << trajectory.result.end_sow << " s" << std::endl
              << "  imu epochs: " << trajectory.result.imu_epochs << std::endl
              << "  gnss pos updates: " << trajectory.result.gnss_pos_updates << std::endl
              << "  gnss vel updates: " << trajectory.result.gnss_vel_updates << std::endl
              << "  zupt updates: " << trajectory.result.zupt_updates << std::endl
              << "  nhc updates: " << trajectory.result.nhc_updates << std::endl;

    std::cout << std::setprecision(10)
              << "  final pos: lat=" << final_state.pva.blh[0] * RAD2DEG
              << " deg, lon=" << final_state.pva.blh[1] * RAD2DEG
              << " deg, h=" << final_state.pva.blh[2] << " m" << std::endl;

    std::cout << std::setprecision(6)
              << "  final att: yaw=" << final_state.pva.att.yaw * RAD2DEG
              << " deg, pitch=" << final_state.pva.att.pitch * RAD2DEG
              << " deg, roll=" << final_state.pva.att.roll * RAD2DEG << " deg" << std::endl
              << "  final bias: bg=" << final_state.imu_bias.gyro_bias.transpose()
              << ", ba=" << final_state.imu_bias.acc_bias.transpose() << std::endl
              << "  final scale: sg=" << final_state.imu_scale.gyro_scale.transpose()
              << ", sa=" << final_state.imu_scale.acc_scale.transpose() << std::endl;
}
