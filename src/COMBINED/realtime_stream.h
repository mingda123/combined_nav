#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "GNSS/gnssData.h"
#include "IMU/fileLoader.h"

enum class RealtimeTransport
{
    UDP = 0,
    TCP = 1
};

struct RealtimeStreamConfig
{
    RealtimeTransport transport = RealtimeTransport::UDP;
    std::string bind_host = "0.0.0.0";
    int port = 9000;
    int tcp_backlog = 1;
    int poll_timeout_ms = 200;
    int max_packet_size = 4096;
    int default_gps_week = 0;
};

struct RealtimeRecord
{
    enum Type
    {
        UNKNOWN = 0,
        IMU = 1,
        GNSS = 2
    };

    Type type = UNKNOWN;
    IMUData imu;
    GNSSData gnss;
};

bool parseRealtimeRecord(const std::string &line,
                         double imu_sample_rate,
                         int default_gps_week,
                         RealtimeRecord &record,
                         std::string &error_message);

class RealtimeLineReceiver
{
public:
#ifdef _WIN32
    using SocketHandle = uintptr_t;
#else
    using SocketHandle = int;
#endif

    RealtimeLineReceiver();
    ~RealtimeLineReceiver();

    bool open(const RealtimeStreamConfig &config, std::string &error_message);
    bool pollLines(std::vector<std::string> &lines, std::string &error_message);

private:
    bool openUdpSocket(std::string &error_message);
    bool openTcpSocket(std::string &error_message);
    bool pollUdp(std::vector<std::string> &lines, std::string &error_message);
    bool pollTcp(std::vector<std::string> &lines, std::string &error_message);
    void closeSockets();
    void consumeTextBuffer(const char *buffer, int length, std::vector<std::string> &lines);

private:
    RealtimeStreamConfig config_;
    bool is_open_ = false;
    bool wsa_started_ = false;
    std::string tcp_buffer_;

    SocketHandle server_socket_;
    SocketHandle client_socket_;
};

class RealtimeLineSender
{
public:
    RealtimeLineSender();
    ~RealtimeLineSender();

    bool open(const RealtimeStreamConfig &config, std::string &error_message);
    bool sendLine(const std::string &line, std::string &error_message);

private:
    bool openUdp(std::string &error_message);
    bool openTcp(std::string &error_message);
    void closeSocket();

private:
    RealtimeStreamConfig config_;
    bool is_open_ = false;
    bool wsa_started_ = false;
    RealtimeLineReceiver::SocketHandle socket_;
};
