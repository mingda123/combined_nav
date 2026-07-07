#include <iostream>
#include <string>
#include <vector>

#include "simpleini/SimpleIni.h"

#include "COMBINED/lci_config_helper.h"
#include "COMBINED/lci_runner.h"

int main()
{
    system("chcp 65001");

    CSimpleIniA ini_forward_lci;
    SI_Error rc = ini_forward_lci.LoadFile("./config/combined_nav.ini");
    if (rc < 0)
    {
        printf("load %s ini file failure\n", "./config/combined_nav.ini");
        return -1;
    }

    AlignConfig align_cfg = loadAlignConfig(ini_forward_lci);
    LciFilterConfig lci_cfg = loadLciFilterConfig(ini_forward_lci, "forward_lci", align_cfg);

    std::vector<IMUData> raw_imu_datas;
    std::vector<IMUData> align_imu_datas;
    std::vector<GNSSData> gnss_datas;
    if (!loadLciRawData(align_cfg, raw_imu_datas, align_imu_datas, gnss_datas))
    {
        return -1;
    }

    LciTrajectory forward_trajectory;
    if (!runForwardLci(align_cfg, lci_cfg, raw_imu_datas, align_imu_datas, gnss_datas, forward_trajectory))
    {
        std::cerr << "错误：前向松组合未生成有效结果。" << std::endl;
        return -1;
    }

    const char *output_file = ini_forward_lci.GetValue("forward_lci", "output_file", "./assets2/results/forward_lci_result.csv");
    saveLciResult(output_file, forward_trajectory.states, forward_trajectory.times);
    printLciSummary("前向松组合已完成", forward_trajectory);
    std::cout << "  输出文件: " << output_file << std::endl;
    return 0;
}
