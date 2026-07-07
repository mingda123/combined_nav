#include "realtime_stream.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <sstream>

#include "COMMON/const_info.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace
{
std::string trim(const std::string &text)
{
    size_t first = 0;
    while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])) != 0)
    {
        ++first;
    }

    size_t last = text.size();
    while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])) != 0)
    {
        --last;
    }

    return text.substr(first, last - first);
}

std::string toUpperCopy(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(),
                   [](unsigned char ch)
                   { return static_cast<char>(std::toupper(ch)); });
    return text;
}

bool parseNumericFields(const std::string &line, std::vector<double> &values)
{
    std::string normalized = line;
    std::replace(normalized.begin(), normalized.end(), ',', ' ');

    std::stringstream ss(normalized);
    double value = 0.0;
    values.clear();
    while (ss >> value)
    {
        values.push_back(value);
    }

    return !values.empty();
}

void setImuFromRates(int week,
                     double sow,
                     double imu_sample_rate,
                     double gx_deg_s,
                     double gy_deg_s,
                     double gz_deg_s,
                     double ax_mps2,
                     double ay_mps2,
                     double az_mps2,
                     IMUData &imu)
{
    const double dt = 1.0 / imu_sample_rate;
    imu = IMUData();
    imu.gpstime.week = week;
    imu.gpstime.sow = sow;
    imu.frq = imu_sample_rate;
    imu.dt = dt;
    imu.dangel << gx_deg_s * DEG2RAD * dt,
        gy_deg_s * DEG2RAD * dt,
        gz_deg_s * DEG2RAD * dt;
    imu.dvel << ax_mps2 * dt,
        ay_mps2 * dt,
        az_mps2 * dt;
}

bool parseImuValues(const std::vector<double> &values,
                    double imu_sample_rate,
                    int default_gps_week,
                    IMUData &imu)
{
    if (imu_sample_rate <= 0.0)
    {
        return false;
    }

    if (values.size() == 7)
    {
        setImuFromRates(default_gps_week,
                        values[0],
                        imu_sample_rate,
                        values[1],
                        values[2],
                        values[3],
                        values[4],
                        values[5],
                        values[6],
                        imu);
        return true;
    }

    if (values.size() == 8)
    {
        setImuFromRates(static_cast<int>(values[0]),
                        values[1],
                        imu_sample_rate,
                        values[2],
                        values[3],
                        values[4],
                        values[5],
                        values[6],
                        values[7],
                        imu);
        return true;
    }

    return false;
}

bool parseGnssValues(const std::vector<double> &values, GNSSData &gnss)
{
    if (values.size() != 14)
    {
        return false;
    }

    gnss = GNSSData();
    gnss.gpstime.week = static_cast<int>(values[0]);
    gnss.gpstime.sow = values[1];
    gnss.blh << values[2] * DEG2RAD, values[3] * DEG2RAD, values[4];
    gnss.pos_std_ned << values[5], values[6], values[7];
    gnss.vel_n << values[8], values[9], -values[10];
    gnss.vel_std_ned << values[11], values[12], values[13];
    gnss.is_valid = true;
    return true;
}

std::string socketErrorMessage(const char *prefix)
{
#ifdef _WIN32
    return std::string(prefix) + " (WSA error " + std::to_string(WSAGetLastError()) + ")";
#else
    return std::string(prefix) + " (" + std::strerror(errno) + ")";
#endif
}

bool setSocketNonBlocking(RealtimeLineReceiver::SocketHandle socket_handle, std::string &error_message)
{
#ifdef _WIN32
    u_long mode = 1;
    if (ioctlsocket(static_cast<SOCKET>(socket_handle), FIONBIO, &mode) != 0)
    {
        error_message = socketErrorMessage("failed to enable non-blocking socket");
        return false;
    }
#else
    int flags = fcntl(socket_handle, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_handle, F_SETFL, flags | O_NONBLOCK) < 0)
    {
        error_message = socketErrorMessage("failed to enable non-blocking socket");
        return false;
    }
#endif
    return true;
}

void closeSocketHandle(RealtimeLineReceiver::SocketHandle &socket_handle)
{
#ifdef _WIN32
    const RealtimeLineReceiver::SocketHandle invalid_socket = static_cast<RealtimeLineReceiver::SocketHandle>(INVALID_SOCKET);
    if (socket_handle != invalid_socket)
    {
        closesocket(static_cast<SOCKET>(socket_handle));
        socket_handle = invalid_socket;
    }
#else
    const RealtimeLineReceiver::SocketHandle invalid_socket = -1;
    if (socket_handle != invalid_socket)
    {
        close(socket_handle);
        socket_handle = invalid_socket;
    }
#endif
}
} // namespace

RealtimeLineReceiver::RealtimeLineReceiver()
{
#ifdef _WIN32
    server_socket_ = static_cast<SocketHandle>(INVALID_SOCKET);
    client_socket_ = static_cast<SocketHandle>(INVALID_SOCKET);
#else
    server_socket_ = -1;
    client_socket_ = -1;
#endif
}

RealtimeLineReceiver::~RealtimeLineReceiver()
{
    closeSockets();
}

bool parseRealtimeRecord(const std::string &line,
                         double imu_sample_rate,
                         int default_gps_week,
                         RealtimeRecord &record,
                         std::string &error_message)
{
    record = RealtimeRecord();
    error_message.clear();

    const std::string text = trim(line);
    if (text.empty() || text[0] == '#')
    {
        error_message = "empty line";
        return false;
    }

    std::stringstream header_stream(text);
    std::string first_token;
    header_stream >> first_token;
    const std::string normalized_token = toUpperCopy(first_token);

    std::string numeric_part = text;
    RealtimeRecord::Type forced_type = RealtimeRecord::UNKNOWN;
    if (normalized_token == "IMU" || normalized_token == "GNSS")
    {
        forced_type = (normalized_token == "IMU") ? RealtimeRecord::IMU : RealtimeRecord::GNSS;
        std::string remainder;
        std::getline(header_stream, remainder);
        numeric_part = remainder;
    }

    std::vector<double> values;
    if (!parseNumericFields(numeric_part, values))
    {
        error_message = "failed to parse numeric payload";
        return false;
    }

    if (forced_type == RealtimeRecord::IMU || (forced_type == RealtimeRecord::UNKNOWN && (values.size() == 7 || values.size() == 8)))
    {
        if (!parseImuValues(values, imu_sample_rate, default_gps_week, record.imu))
        {
            error_message = "invalid IMU payload";
            return false;
        }
        record.type = RealtimeRecord::IMU;
        return true;
    }

    if (forced_type == RealtimeRecord::GNSS || (forced_type == RealtimeRecord::UNKNOWN && values.size() == 14))
    {
        if (!parseGnssValues(values, record.gnss))
        {
            error_message = "invalid GNSS payload";
            return false;
        }
        record.type = RealtimeRecord::GNSS;
        return true;
    }

    error_message = "unsupported record format";
    return false;
}

bool RealtimeLineReceiver::open(const RealtimeStreamConfig &config, std::string &error_message)
{
    closeSockets();
    config_ = config;

#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
    {
        error_message = "failed to initialize Winsock";
        return false;
    }
    wsa_started_ = true;
#endif

    bool opened = false;
    if (config_.transport == RealtimeTransport::UDP)
    {
        opened = openUdpSocket(error_message);
    }
    else
    {
        opened = openTcpSocket(error_message);
    }

    if (!opened)
    {
        closeSockets();
        return false;
    }

    is_open_ = true;
    return true;
}

bool RealtimeLineReceiver::openUdpSocket(std::string &error_message)
{
    server_socket_ = static_cast<SocketHandle>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
#ifdef _WIN32
    if (server_socket_ == static_cast<SocketHandle>(INVALID_SOCKET))
#else
    if (server_socket_ < 0)
#endif
    {
        error_message = socketErrorMessage("failed to create UDP socket");
        return false;
    }

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config_.port));
    if (config_.bind_host.empty() || config_.bind_host == "0.0.0.0")
    {
        address.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else if (inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1)
    {
        error_message = "invalid UDP bind_host";
        return false;
    }

    if (bind(static_cast<SOCKET>(server_socket_), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
        error_message = socketErrorMessage("failed to bind UDP socket");
        return false;
    }

    return setSocketNonBlocking(server_socket_, error_message);
}

bool RealtimeLineReceiver::openTcpSocket(std::string &error_message)
{
    server_socket_ = static_cast<SocketHandle>(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
#ifdef _WIN32
    if (server_socket_ == static_cast<SocketHandle>(INVALID_SOCKET))
#else
    if (server_socket_ < 0)
#endif
    {
        error_message = socketErrorMessage("failed to create TCP socket");
        return false;
    }

    int reuse = 1;
    setsockopt(static_cast<SOCKET>(server_socket_), SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char *>(&reuse), static_cast<int>(sizeof(reuse)));

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config_.port));
    if (config_.bind_host.empty() || config_.bind_host == "0.0.0.0")
    {
        address.sin_addr.s_addr = htonl(INADDR_ANY);
    }
    else if (inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1)
    {
        error_message = "invalid TCP bind_host";
        return false;
    }

    if (bind(static_cast<SOCKET>(server_socket_), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
        error_message = socketErrorMessage("failed to bind TCP socket");
        return false;
    }

    if (listen(static_cast<SOCKET>(server_socket_), config_.tcp_backlog) != 0)
    {
        error_message = socketErrorMessage("failed to listen on TCP socket");
        return false;
    }

    return setSocketNonBlocking(server_socket_, error_message);
}

bool RealtimeLineReceiver::pollLines(std::vector<std::string> &lines, std::string &error_message)
{
    lines.clear();
    error_message.clear();
    if (!is_open_)
    {
        error_message = "receiver is not open";
        return false;
    }

    if (config_.transport == RealtimeTransport::UDP)
    {
        return pollUdp(lines, error_message);
    }
    return pollTcp(lines, error_message);
}

bool RealtimeLineReceiver::pollUdp(std::vector<std::string> &lines, std::string &error_message)
{
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(static_cast<SOCKET>(server_socket_), &read_set);

    timeval timeout;
    timeout.tv_sec = config_.poll_timeout_ms / 1000;
    timeout.tv_usec = (config_.poll_timeout_ms % 1000) * 1000;

    const int ready = select(0, &read_set, nullptr, nullptr, &timeout);
    if (ready < 0)
    {
        error_message = socketErrorMessage("UDP select failed");
        return false;
    }
    if (ready == 0)
    {
        return true;
    }

    std::vector<char> buffer(static_cast<size_t>(config_.max_packet_size), 0);
    int received = 0;
    do
    {
        received = recvfrom(static_cast<SOCKET>(server_socket_),
                            buffer.data(),
                            static_cast<int>(buffer.size()),
                            0,
                            nullptr,
                            nullptr);
        if (received > 0)
        {
            consumeTextBuffer(buffer.data(), received, lines);
        }
    } while (received > 0);

    return true;
}

bool RealtimeLineReceiver::pollTcp(std::vector<std::string> &lines, std::string &error_message)
{
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(static_cast<SOCKET>(server_socket_), &read_set);
    SOCKET max_socket = static_cast<SOCKET>(server_socket_);
    if (
#ifdef _WIN32
        client_socket_ != static_cast<SocketHandle>(INVALID_SOCKET)
#else
        client_socket_ >= 0
#endif
    )
    {
        FD_SET(static_cast<SOCKET>(client_socket_), &read_set);
        if (static_cast<SOCKET>(client_socket_) > max_socket)
        {
            max_socket = static_cast<SOCKET>(client_socket_);
        }
    }

    timeval timeout;
    timeout.tv_sec = config_.poll_timeout_ms / 1000;
    timeout.tv_usec = (config_.poll_timeout_ms % 1000) * 1000;

    const int ready = select(static_cast<int>(max_socket + 1), &read_set, nullptr, nullptr, &timeout);
    if (ready < 0)
    {
        error_message = socketErrorMessage("TCP select failed");
        return false;
    }
    if (ready == 0)
    {
        return true;
    }

    if (FD_ISSET(static_cast<SOCKET>(server_socket_), &read_set))
    {
        sockaddr_in client_address;
#ifdef _WIN32
        int client_length = sizeof(client_address);
#else
        socklen_t client_length = sizeof(client_address);
#endif
        SocketHandle accepted_socket = static_cast<SocketHandle>(
            accept(static_cast<SOCKET>(server_socket_), reinterpret_cast<sockaddr *>(&client_address), &client_length));
#ifdef _WIN32
        if (accepted_socket != static_cast<SocketHandle>(INVALID_SOCKET))
#else
        if (accepted_socket >= 0)
#endif
        {
            closeSocketHandle(client_socket_);
            client_socket_ = accepted_socket;
            if (!setSocketNonBlocking(client_socket_, error_message))
            {
                closeSocketHandle(client_socket_);
                return false;
            }
            tcp_buffer_.clear();
        }
    }

    if (
#ifdef _WIN32
        client_socket_ == static_cast<SocketHandle>(INVALID_SOCKET)
#else
        client_socket_ < 0
#endif
    )
    {
        return true;
    }

    if (FD_ISSET(static_cast<SOCKET>(client_socket_), &read_set))
    {
        std::vector<char> buffer(static_cast<size_t>(config_.max_packet_size), 0);
        int received = recv(static_cast<SOCKET>(client_socket_), buffer.data(), static_cast<int>(buffer.size()), 0);
        if (received > 0)
        {
            consumeTextBuffer(buffer.data(), received, lines);
        }
        else if (received == 0)
        {
            closeSocketHandle(client_socket_);
            tcp_buffer_.clear();
        }
    }

    return true;
}

void RealtimeLineReceiver::consumeTextBuffer(const char *buffer, int length, std::vector<std::string> &lines)
{
    if (length <= 0)
    {
        return;
    }

    tcp_buffer_.append(buffer, static_cast<size_t>(length));
    size_t line_end = tcp_buffer_.find('\n');
    while (line_end != std::string::npos)
    {
        std::string line = tcp_buffer_.substr(0, line_end);
        if (!line.empty() && line.back() == '\r')
        {
            line.pop_back();
        }
        line = trim(line);
        if (!line.empty())
        {
            lines.push_back(line);
        }
        tcp_buffer_.erase(0, line_end + 1);
        line_end = tcp_buffer_.find('\n');
    }
}

void RealtimeLineReceiver::closeSockets()
{
    closeSocketHandle(client_socket_);
    closeSocketHandle(server_socket_);
    is_open_ = false;
    tcp_buffer_.clear();

#ifdef _WIN32
    if (wsa_started_)
    {
        WSACleanup();
        wsa_started_ = false;
    }
#endif
}

RealtimeLineSender::RealtimeLineSender()
{
#ifdef _WIN32
    socket_ = static_cast<RealtimeLineReceiver::SocketHandle>(INVALID_SOCKET);
#else
    socket_ = -1;
#endif
}

RealtimeLineSender::~RealtimeLineSender()
{
    closeSocket();
}

bool RealtimeLineSender::open(const RealtimeStreamConfig &config, std::string &error_message)
{
    closeSocket();
    config_ = config;

#ifdef _WIN32
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0)
    {
        error_message = "failed to initialize Winsock";
        return false;
    }
    wsa_started_ = true;
#endif

    const bool opened = (config_.transport == RealtimeTransport::UDP)
                            ? openUdp(error_message)
                            : openTcp(error_message);
    if (!opened)
    {
        closeSocket();
        return false;
    }

    is_open_ = true;
    return true;
}

bool RealtimeLineSender::sendLine(const std::string &line, std::string &error_message)
{
    if (!is_open_)
    {
        error_message = "sender is not open";
        return false;
    }

    const std::string payload = line + "\n";
    const int sent = send(static_cast<SOCKET>(socket_),
                          payload.c_str(),
                          static_cast<int>(payload.size()),
                          0);
    if (sent != static_cast<int>(payload.size()))
    {
        error_message = socketErrorMessage("failed to send line");
        return false;
    }

    return true;
}

bool RealtimeLineSender::openUdp(std::string &error_message)
{
    socket_ = static_cast<RealtimeLineReceiver::SocketHandle>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
#ifdef _WIN32
    if (socket_ == static_cast<RealtimeLineReceiver::SocketHandle>(INVALID_SOCKET))
#else
    if (socket_ < 0)
#endif
    {
        error_message = socketErrorMessage("failed to create UDP socket");
        return false;
    }

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config_.port));
    if (inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1)
    {
        error_message = "invalid UDP target host";
        return false;
    }

    if (connect(static_cast<SOCKET>(socket_), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
        error_message = socketErrorMessage("failed to connect UDP target");
        return false;
    }

    return true;
}

bool RealtimeLineSender::openTcp(std::string &error_message)
{
    socket_ = static_cast<RealtimeLineReceiver::SocketHandle>(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
#ifdef _WIN32
    if (socket_ == static_cast<RealtimeLineReceiver::SocketHandle>(INVALID_SOCKET))
#else
    if (socket_ < 0)
#endif
    {
        error_message = socketErrorMessage("failed to create TCP socket");
        return false;
    }

    sockaddr_in address;
    std::memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(config_.port));
    if (inet_pton(AF_INET, config_.bind_host.c_str(), &address.sin_addr) != 1)
    {
        error_message = "invalid TCP target host";
        return false;
    }

    if (connect(static_cast<SOCKET>(socket_), reinterpret_cast<const sockaddr *>(&address), sizeof(address)) != 0)
    {
        error_message = socketErrorMessage("failed to connect TCP target");
        return false;
    }

    return true;
}

void RealtimeLineSender::closeSocket()
{
    closeSocketHandle(socket_);
    is_open_ = false;

#ifdef _WIN32
    if (wsa_started_)
    {
        WSACleanup();
        wsa_started_ = false;
    }
#endif
}
