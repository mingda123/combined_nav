#pragma once

#include <fstream>

#include "inerNav.h"

class FileSaver
{
private:
    std::ofstream file;

    bool open(const char *filename);

public:
    FileSaver() {}
    FileSaver(const char *filename);
    ~FileSaver();

    bool reset(const char *filename);
    bool saveSingleInT(const PVA &pva_cur, const IMUData &imu_cur);
};
