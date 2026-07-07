#include <iostream>

#include "simpleini/SimpleIni.h"

#include "COMBINED/lci_config_helper.h"
#include "COMBINED/realtime_forward_runner.h"

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

    if (value == "tcp")
    {
        return RealtimeTransport::TCP;
    }
    return RealtimeTransport::UDP;
}

RealtimeStreamConfig loadRealtimeStreamConfig(CSimpleIniA &ini)
{
    RealtimeStreamConfig cfg;
    cfg.transport = parseTransport(ini.GetValue("realtime_stream", "protocol", "udp"));
    cfg.bind_host = ini.GetValue("realtime_stream", "bind_host", "0.0.0.0");
    cfg.port = static_cast<int>(getIniDouble(ini, "realtime_stream", "port", 9000.0));
    cfg.tcp_backlog = static_cast<int>(getIniDouble(ini, "realtime_stream", "tcp_backlog", 1.0));
    cfg.poll_timeout_ms = static_cast<int>(getIniDouble(ini, "realtime_stream", "poll_timeout_ms", 200.0));
    cfg.max_packet_size = static_cast<int>(getIniDouble(ini, "realtime_stream", "max_packet_size", 4096.0));
    cfg.default_gps_week = static_cast<int>(getIniDouble(ini, "realtime_stream", "default_gps_week", 0.0));
    return cfg;
}

RealtimeForwardLciConfig loadRealtimeForwardLciConfig(CSimpleIniA &ini)
{
    RealtimeForwardLciConfig cfg;
    cfg.output_file = ini.GetValue("realtime_forward_lci", "output_file", "./assets2/results/realtime_forward_lci_result.csv");
    cfg.print_interval = static_cast<int>(getIniDouble(ini, "realtime_forward_lci", "print_interval", 200.0));
    cfg.alignment_check_interval = static_cast<int>(getIniDouble(ini, "realtime_forward_lci", "alignment_check_interval", 50.0));
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
    const LciFilterConfig lci_cfg = loadLciFilterConfig(ini, "forward_lci", align_cfg);
    const RealtimeStreamConfig stream_cfg = loadRealtimeStreamConfig(ini);
    const RealtimeForwardLciConfig realtime_cfg = loadRealtimeForwardLciConfig(ini);

    if (!runRealtimeForwardLci(align_cfg, lci_cfg, stream_cfg, realtime_cfg))
    {
        return -1;
    }

    return 0;
}
