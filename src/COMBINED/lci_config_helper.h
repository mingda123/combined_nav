#pragma once

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>

#include <Eigen/Dense>

#include "simpleini/SimpleIni.h"

#include "COMBINED/alignment.h"
#include "COMBINED/lci_filter.h"

inline double getIniDouble(CSimpleIniA &ini, const char *section, const char *key, double default_value)
{
    return ini.GetDoubleValue(section, key, default_value);
}

inline bool getIniBool(CSimpleIniA &ini, const char *section, const char *key, bool default_value)
{
    return ini.GetBoolValue(section, key, default_value);
}

inline Eigen::Vector3d getIniVector3(CSimpleIniA &ini,
                                     const char *section,
                                     const char *key,
                                     const Eigen::Vector3d &default_value)
{
    const char *raw_value = ini.GetValue(section, key, nullptr);
    if (raw_value == nullptr)
    {
        return default_value;
    }

    std::string text(raw_value);
    text.erase(std::remove_if(text.begin(), text.end(),
                              [](unsigned char ch)
                              {
                                  return std::isspace(ch) != 0;
                              }),
               text.end());

    if (!text.empty() && text.front() == '[')
    {
        text.erase(text.begin());
    }
    if (!text.empty() && text.back() == ']')
    {
        text.pop_back();
    }
    std::replace(text.begin(), text.end(), ',', ' ');

    std::istringstream iss(text);
    Eigen::Vector3d value = default_value;
    if (!(iss >> value[0] >> value[1] >> value[2]))
    {
        return default_value;
    }
    return value;
}

inline Eigen::Vector3d getIniVector3DegToRad(CSimpleIniA &ini,
                                             const char *section,
                                             const char *key,
                                             const Eigen::Vector3d &default_value_deg)
{
    return getIniVector3(ini, section, key, default_value_deg).array() * DEG2RAD;
}

inline AlignConfig loadAlignConfig(CSimpleIniA &ini)
{
    AlignConfig cfg;
    cfg.imu_file = ini.GetValue("imu", "input_file", "");
    cfg.gnss_file = ini.GetValue("gnss", "input_file", "");
    cfg.imu_sample_rate = getIniDouble(ini, "imu", "sample_rate", 200.0);
    cfg.imu_need_trans = getIniBool(ini, "imu", "need_trans", false);
    cfg.start_search_length = getIniDouble(ini, "alignment", "start_search_length", 600.0);
    cfg.end_search_length = getIniDouble(ini, "alignment", "end_search_length", 400.0);
    cfg.min_static_duration = getIniDouble(ini, "alignment", "min_static_duration", 180.0);

    cfg.start_align_acc_bias = getIniVector3(ini, "alignment", "start_align_acc_bias", Eigen::Vector3d::Zero());
    cfg.start_align_gyro_bias = getIniVector3(ini, "alignment", "start_align_gyro_bias", Eigen::Vector3d::Zero());
    cfg.end_align_acc_bias = getIniVector3(ini, "alignment", "end_align_acc_bias", cfg.start_align_acc_bias);
    cfg.end_align_gyro_bias = getIniVector3(ini, "alignment", "end_align_gyro_bias", cfg.start_align_gyro_bias);
    cfg.imu_noise.acc_white_noise = getIniVector3(ini, "imu", "acc_noise", Eigen::Vector3d::Zero());
    cfg.imu_noise.gyro_white_noise = getIniVector3DegToRad(ini, "imu", "gyro_noise", Eigen::Vector3d::Zero());
    cfg.imu_noise.acc_bias_random_walk = getIniVector3(ini, "imu", "acc_random_walk", Eigen::Vector3d::Zero());
    cfg.imu_noise.gyro_bias_random_walk = getIniVector3DegToRad(ini, "imu", "gyro_random_walk", Eigen::Vector3d::Zero());
    return cfg;
}

inline LciFilterConfig loadLciFilterConfig(CSimpleIniA &ini, const char *section_name, const AlignConfig &align_cfg)
{
    LciFilterConfig cfg;
    cfg.gnss_imu_lever_b = getIniVector3(ini, "gnss", "lever_arm", Eigen::Vector3d::Zero());
    cfg.imu_noise = align_cfg.imu_noise;
    cfg.init_pos_std_ned = getIniVector3(ini, section_name, "init_pos_std", Eigen::Vector3d::Constant(5.0));
    cfg.init_vel_std_ned = getIniVector3(ini, section_name, "init_vel_std", Eigen::Vector3d::Constant(0.5));
    cfg.init_att_std_rad =
        getIniVector3(ini, section_name, "init_att_std_deg", Eigen::Vector3d::Constant(3.0)).array() * DEG2RAD;
    cfg.init_gyro_bias_std =
        getIniVector3DegToRad(ini, section_name, "init_gyro_bias_std", Eigen::Vector3d::Constant(0.005));
    cfg.init_acc_bias_std = getIniVector3(ini, section_name, "init_acc_bias_std", Eigen::Vector3d::Constant(0.05));
    cfg.init_gyro_scale_std = getIniVector3(ini, section_name, "init_gyro_scale_std", Eigen::Vector3d::Constant(1e-4));
    cfg.init_acc_scale_std = getIniVector3(ini, section_name, "init_acc_scale_std", Eigen::Vector3d::Constant(1e-4));
    cfg.corr_time = getIniDouble(ini, section_name, "corr_time", 3600.0);
    cfg.gnss_time_tolerance = getIniDouble(ini, section_name, "gnss_time_tolerance", 0.001);
    cfg.use_gnss_velocity = getIniBool(ini, section_name, "use_gnss_velocity", false);
    cfg.use_zupt = getIniBool(ini, section_name, "use_zupt", false);
    cfg.use_nhc = getIniBool(ini, section_name, "use_nhc", false);
    cfg.zupt_std_ned = getIniVector3(ini, section_name, "zupt_std", Eigen::Vector3d::Constant(0.05));
    cfg.zupt_acc_threshold = getIniDouble(ini, section_name, "zupt_acc_threshold", 0.15);
    cfg.zupt_gyro_threshold = getIniDouble(ini, section_name, "zupt_gyro_threshold", 0.02);
    cfg.zupt_min_duration = getIniDouble(ini, section_name, "zupt_min_duration", 0.5);
    cfg.nhc_std_body = getIniVector3(ini, section_name, "nhc_std", Eigen::Vector3d(0.0, 0.10, 0.10));
    return cfg;
}
