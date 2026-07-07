#include "gnssFileLoader.h"

#include <cctype>
#include <iostream>
#include <sstream>
#include <vector>

#include "COMMON/const_info.h"

namespace
{
    std::string trim(const std::string &text)
    {
        size_t first = 0;
        while (first < text.size() && std::isspace(static_cast<unsigned char>(text[first])))
        {
            first++;
        }

        size_t last = text.size();
        while (last > first && std::isspace(static_cast<unsigned char>(text[last - 1])))
        {
            last--;
        }

        return text.substr(first, last - first);
    }
}

GnssFileLoader::GnssFileLoader(const char *filename)
{
    reset(filename);
}

GnssFileLoader::~GnssFileLoader()
{
    if (file.is_open())
    {
        file.close();
    }
}

bool GnssFileLoader::open(const char *filename)
{
    file.open(filename);
    if (!file.is_open())
    {
        std::cerr << "错误：无法打开 GNSS 文件 " << filename << std::endl;
        return false;
    }

    return true;
}

bool GnssFileLoader::reset(const char *filename)
{
    if (file.is_open())
    {
        file.close();
    }

    gnss_cur = GNSSData();
    return open(filename);
}

bool GnssFileLoader::parseLine(const std::string &line, GNSSData &data) const
{
    const std::string text = trim(line);
    if (text.empty())
    {
        return false;
    }

    std::stringstream ss(text);
    std::vector<double> values;
    double value = 0.0;
    while (ss >> value)
    {
        values.push_back(value);
    }

    if (values.size() < 14)
    {
        // 表头或无效行直接跳过。
        return false;
    }

    data = GNSSData();
    data.gpstime.week = static_cast<int>(values[0]);
    data.gpstime.sow = values[1];
    data.blh << values[2] * DEG2RAD, values[3] * DEG2RAD, values[4];

    // 输入文件给出的是北、东、上方向标准差，这里统一按 NED 约定保存。
    data.pos_std_ned << values[5], values[6], values[7];

    // 输入速度是 Local-VN、Local-VE、Local-VU，这里转换为 NED 的 vd。
    data.vel_n << values[8], values[9], -values[10];
    data.vel_std_ned << values[11], values[12], values[13];

    data.is_valid = true;
    return true;
}

bool GnssFileLoader::readSingle(GNSSData &data)
{
    std::string line;
    while (file.is_open() && std::getline(file, line))
    {
        if (parseLine(line, gnss_cur))
        {
            data = gnss_cur;
            return true;
        }
    }

    return false;
}

bool GnssFileLoader::readAll(std::vector<GNSSData> &datas)
{
    GNSSData data;
    while (readSingle(data))
    {
        datas.push_back(data);
    }

    return !datas.empty();
}

bool GnssFileLoader::isEndOfFile() const
{
    return file.eof();
}
