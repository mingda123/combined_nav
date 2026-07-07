#pragma once

#include <string>

#include "COMBINED/alignment.h"
#include "COMBINED/realtime_stream.h"

struct RealtimeFileSenderConfig
{
    std::string imu_file;
    std::string gnss_file;
    double speed = 1.0;
    bool loop = false;
    bool send_imu = true;
    bool send_gnss = true;
    int startup_wait_ms = 1000;
    int log_interval = 500;
};

bool runRealtimeFileSender(const AlignConfig &align_cfg,
                           const RealtimeStreamConfig &stream_cfg,
                           const RealtimeFileSenderConfig &sender_cfg);
