#pragma once

struct GPSTime
{
    int week;   // GPS周
    double sow; // 周内秒
    GPSTime() { week = 0, sow = 0.0; }
};