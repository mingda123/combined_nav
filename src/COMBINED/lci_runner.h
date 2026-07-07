#pragma once

#include <string>
#include <vector>

#include "COMMON/navState.h"
#include "COMBINED/alignment.h"
#include "COMBINED/lci_filter.h"
#include "GNSS/gnssData.h"
#include "IMU/fileLoader.h"

struct LciTrajectory
{
    IntegratedNavState init_state;
    double init_sow = 0.0;
    std::vector<IntegratedNavState> states;
    std::vector<GPSTime> times;
    std::vector<Matrix21d> covariances;
    LciFilterResult result;
};

bool loadLciRawData(const AlignConfig &align_cfg,
                    std::vector<IMUData> &raw_imu_datas,
                    std::vector<IMUData> &align_imu_datas,
                    std::vector<GNSSData> &gnss_datas);

void saveLciResult(const std::string &output_file,
                   const std::vector<IntegratedNavState> &states,
                   const std::vector<GPSTime> &times);

bool runForwardLci(const AlignConfig &align_cfg,
                   const LciFilterConfig &lci_cfg,
                   const std::vector<IMUData> &raw_imu_datas,
                   const std::vector<IMUData> &align_imu_datas,
                   const std::vector<GNSSData> &gnss_datas,
                   LciTrajectory &trajectory);

bool runBackwardLci(const AlignConfig &align_cfg,
                    const LciFilterConfig &lci_cfg,
                    const std::vector<IMUData> &raw_imu_datas,
                    const std::vector<IMUData> &align_imu_datas,
                    const std::vector<GNSSData> &gnss_datas,
                    const IntegratedNavState *imu_init_reference,
                    LciTrajectory &trajectory);

bool fuseForwardBackwardLci(const LciTrajectory &forward_trajectory,
                            const LciTrajectory &backward_trajectory,
                            LciTrajectory &fused_trajectory);

void printLciSummary(const char *title, const LciTrajectory &trajectory);
