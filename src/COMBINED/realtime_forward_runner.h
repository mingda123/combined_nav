#pragma once

#include <deque>
#include <fstream>
#include <string>
#include <vector>

#include "COMBINED/alignment.h"
#include "COMBINED/lci_filter.h"
#include "COMBINED/realtime_stream.h"

struct RealtimeForwardLciConfig
{
    std::string output_file = "./assets2/results/realtime_forward_lci_result.csv";
    int print_interval = 200;
    int alignment_check_interval = 50;
};

bool runRealtimeForwardLci(const AlignConfig &align_cfg,
                           const LciFilterConfig &lci_cfg,
                           const RealtimeStreamConfig &stream_cfg,
                           const RealtimeForwardLciConfig &realtime_cfg);
