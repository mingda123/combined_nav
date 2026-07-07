#include "realtime_file_sender.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <vector>

#include "GNSS/gnssFileLoader.h"
#include "IMU/fileLoader.h"

namespace
{
struct TimedRecord
{
    RealtimeRecord::Type type = RealtimeRecord::UNKNOWN;
    double sow = 0.0;
    std::string line;
};

std::string formatImuLine(const IMUData &imu)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "IMU "
        << imu.gpstime.week << " "
        << imu.gpstime.sow << " "
        << imu.dangel[0] / imu.dt * RAD2DEG << " "
        << imu.dangel[1] / imu.dt * RAD2DEG << " "
        << imu.dangel[2] / imu.dt * RAD2DEG << " "
        << imu.dvel[0] / imu.dt << " "
        << imu.dvel[1] / imu.dt << " "
        << imu.dvel[2] / imu.dt;
    return oss.str();
}

std::string formatGnssLine(const GNSSData &gnss)
{
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(9)
        << "GNSS "
        << gnss.gpstime.week << " "
        << gnss.gpstime.sow << " "
        << gnss.blh[0] * RAD2DEG << " "
        << gnss.blh[1] * RAD2DEG << " "
        << gnss.blh[2] << " "
        << gnss.pos_std_ned[0] << " "
        << gnss.pos_std_ned[1] << " "
        << gnss.pos_std_ned[2] << " "
        << gnss.vel_n[0] << " "
        << gnss.vel_n[1] << " "
        << -gnss.vel_n[2] << " "
        << gnss.vel_std_ned[0] << " "
        << gnss.vel_std_ned[1] << " "
        << gnss.vel_std_ned[2];
    return oss.str();
}

bool loadTimedRecords(const AlignConfig &align_cfg,
                      const RealtimeFileSenderConfig &sender_cfg,
                      std::vector<TimedRecord> &records)
{
    records.clear();

    if (sender_cfg.send_imu)
    {
        FileLoader imu_loader(sender_cfg.imu_file.c_str());
        std::vector<IMUData> imu_datas;
        if (!imu_loader.readAll(imu_datas, align_cfg.imu_sample_rate))
        {
            std::cerr << "Error: failed to load IMU file " << sender_cfg.imu_file << std::endl;
            return false;
        }

        records.reserve(records.size() + imu_datas.size());
        for (size_t i = 0; i < imu_datas.size(); ++i)
        {
            TimedRecord record;
            record.type = RealtimeRecord::IMU;
            record.sow = imu_datas[i].gpstime.sow;
            record.line = formatImuLine(imu_datas[i]);
            records.push_back(record);
        }
    }

    if (sender_cfg.send_gnss)
    {
        GnssFileLoader gnss_loader(sender_cfg.gnss_file.c_str());
        std::vector<GNSSData> gnss_datas;
        if (!gnss_loader.readAll(gnss_datas))
        {
            std::cerr << "Error: failed to load GNSS file " << sender_cfg.gnss_file << std::endl;
            return false;
        }

        records.reserve(records.size() + gnss_datas.size());
        for (size_t i = 0; i < gnss_datas.size(); ++i)
        {
            TimedRecord record;
            record.type = RealtimeRecord::GNSS;
            record.sow = gnss_datas[i].gpstime.sow;
            record.line = formatGnssLine(gnss_datas[i]);
            records.push_back(record);
        }
    }

    if (records.empty())
    {
        std::cerr << "Error: no records selected for sending." << std::endl;
        return false;
    }

    std::stable_sort(records.begin(), records.end(),
                     [](const TimedRecord &lhs, const TimedRecord &rhs)
                     { return lhs.sow < rhs.sow; });
    return true;
}

void sleepUntilRecordTime(const std::chrono::steady_clock::time_point &wall_start,
                          double first_sow,
                          double current_sow,
                          double speed)
{
    if (speed <= 0.0)
    {
        return;
    }

    const double elapsed_data_seconds = current_sow - first_sow;
    if (elapsed_data_seconds <= 0.0)
    {
        return;
    }

    const auto target_elapsed =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(elapsed_data_seconds / speed));
    const auto target_time = wall_start + target_elapsed;
    const auto now = std::chrono::steady_clock::now();
    if (target_time > now)
    {
        std::this_thread::sleep_until(target_time);
    }
}
} // namespace

bool runRealtimeFileSender(const AlignConfig &align_cfg,
                           const RealtimeStreamConfig &stream_cfg,
                           const RealtimeFileSenderConfig &sender_cfg)
{
    if (!sender_cfg.send_imu && !sender_cfg.send_gnss)
    {
        std::cerr << "Error: both send_imu and send_gnss are disabled." << std::endl;
        return false;
    }
    if (sender_cfg.speed <= 0.0)
    {
        std::cerr << "Error: sender speed must be positive." << std::endl;
        return false;
    }

    std::vector<TimedRecord> records;
    if (!loadTimedRecords(align_cfg, sender_cfg, records))
    {
        return false;
    }

    RealtimeLineSender sender;
    std::string error_message;
    if (!sender.open(stream_cfg, error_message))
    {
        std::cerr << "Error: " << error_message << std::endl;
        return false;
    }

    std::cout << "Realtime file sender started." << std::endl;
    std::cout << "Protocol=" << (stream_cfg.transport == RealtimeTransport::UDP ? "UDP" : "TCP")
              << ", target=" << stream_cfg.bind_host << ":" << stream_cfg.port << std::endl;
    std::cout << "Loaded " << records.size() << " records, speed=" << sender_cfg.speed
              << "x, loop=" << (sender_cfg.loop ? "true" : "false") << std::endl;

    if (sender_cfg.startup_wait_ms > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(sender_cfg.startup_wait_ms));
    }

    do
    {
        const double first_sow = records.front().sow;
        const auto wall_start = std::chrono::steady_clock::now();
        size_t sent_count = 0;

        for (size_t i = 0; i < records.size(); ++i)
        {
            sleepUntilRecordTime(wall_start, first_sow, records[i].sow, sender_cfg.speed);
            if (!sender.sendLine(records[i].line, error_message))
            {
                std::cerr << "Error: " << error_message << std::endl;
                return false;
            }

            ++sent_count;
            if (sender_cfg.log_interval > 0 && (sent_count % static_cast<size_t>(sender_cfg.log_interval) == 0))
            {
                std::cout << "Sent " << sent_count
                          << " records, current_sow=" << records[i].sow
                          << ", type=" << (records[i].type == RealtimeRecord::IMU ? "IMU" : "GNSS")
                          << std::endl;
            }
        }

        std::cout << "One replay pass finished." << std::endl;
    } while (sender_cfg.loop);

    return true;
}
