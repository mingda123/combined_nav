#include <cctype>
#include <iostream>
#include <string>

#include "simpleini/SimpleIni.h"

#include "COMBINED/lci_config_helper.h"
#include "COMBINED/realtime_file_sender.h"

namespace
{
RealtimeTransport parseTransport(const char *text)
{
    if (text == nullptr)
    {
        return RealtimeTransport::UDP;
    }

    std::string value(text);
    for (size_t i = 0; i < value.size(); ++i)
    {
        value[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(value[i])));
    }

    return value == "tcp" ? RealtimeTransport::TCP : RealtimeTransport::UDP;
}

RealtimeStreamConfig loadRealtimeStreamClientConfig(CSimpleIniA &ini)
{
    RealtimeStreamConfig cfg;
    cfg.transport = parseTransport(ini.GetValue("realtime_stream", "protocol", "udp"));
    cfg.bind_host = ini.GetValue("realtime_stream", "target_host", "127.0.0.1");
    cfg.port = static_cast<int>(getIniDouble(ini, "realtime_stream", "target_port", 9000.0));
    cfg.tcp_backlog = static_cast<int>(getIniDouble(ini, "realtime_stream", "tcp_backlog", 1.0));
    cfg.poll_timeout_ms = static_cast<int>(getIniDouble(ini, "realtime_stream", "poll_timeout_ms", 200.0));
    cfg.max_packet_size = static_cast<int>(getIniDouble(ini, "realtime_stream", "max_packet_size", 4096.0));
    cfg.default_gps_week = static_cast<int>(getIniDouble(ini, "realtime_stream", "default_gps_week", 0.0));
    return cfg;
}

RealtimeFileSenderConfig loadRealtimeFileSenderConfig(CSimpleIniA &ini, const AlignConfig &align_cfg)
{
    RealtimeFileSenderConfig cfg;
    cfg.imu_file = ini.GetValue("realtime_file_sender", "imu_file", align_cfg.imu_file.c_str());
    cfg.gnss_file = ini.GetValue("realtime_file_sender", "gnss_file", align_cfg.gnss_file.c_str());
    cfg.speed = getIniDouble(ini, "realtime_file_sender", "speed", 1.0);
    cfg.loop = getIniBool(ini, "realtime_file_sender", "loop", false);
    cfg.send_imu = getIniBool(ini, "realtime_file_sender", "send_imu", true);
    cfg.send_gnss = getIniBool(ini, "realtime_file_sender", "send_gnss", true);
    cfg.startup_wait_ms = static_cast<int>(getIniDouble(ini, "realtime_file_sender", "startup_wait_ms", 1000.0));
    cfg.log_interval = static_cast<int>(getIniDouble(ini, "realtime_file_sender", "log_interval", 500.0));
    return cfg;
}
} // namespace

int main()
{
    system("chcp 65001");

    CSimpleIniA ini;
    SI_Error rc = ini.LoadFile("./config/combined_nav.ini");
    if (rc < 0)
    {
        std::cerr << "load ./config/combined_nav.ini ini file failure" << std::endl;
        return -1;
    }

    const AlignConfig align_cfg = loadAlignConfig(ini);
    const RealtimeStreamConfig stream_cfg = loadRealtimeStreamClientConfig(ini);
    const RealtimeFileSenderConfig sender_cfg = loadRealtimeFileSenderConfig(ini, align_cfg);

    if (!runRealtimeFileSender(align_cfg, stream_cfg, sender_cfg))
    {
        return -1;
    }

    return 0;
}
