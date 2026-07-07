#pragma once

#include <fstream>
#include <string>
#include <vector>

#include "gnssData.h"

class GnssFileLoader
{
private:
    std::ifstream file;
    GNSSData gnss_cur; // 当前读取到的 GNSS 记录

    bool open(const char *filename);
    bool parseLine(const std::string &line, GNSSData &data) const;

public:
    GnssFileLoader() {}
    explicit GnssFileLoader(const char *filename);
    ~GnssFileLoader();

    bool reset(const char *filename);

    bool readSingle(GNSSData &data);
    bool readAll(std::vector<GNSSData> &datas);

    bool isEndOfFile() const;
};
