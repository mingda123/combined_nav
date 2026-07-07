#include <iostream>
#include <string>
#include <vector>

#include "simpleini/SimpleIni.h"

#include "COMBINED/lci_config_helper.h"
#include "COMBINED/lci_runner.h"

int main()
{
    system("chcp 65001");

    CSimpleIniA ini_backward_lci;
    SI_Error rc = ini_backward_lci.LoadFile("./config/combined_nav.ini");
    if (rc < 0)
    {
        printf("load %s ini file failure\n", "./config/combined_nav.ini");
        return -1;
    }

    AlignConfig align_cfg = loadAlignConfig(ini_backward_lci);
    LciFilterConfig lci_cfg = loadLciFilterConfig(ini_backward_lci, "forward_lci", align_cfg);

    std::vector<IMUData> raw_imu_datas;
    std::vector<IMUData> align_imu_datas;
    std::vector<GNSSData> gnss_datas;
    if (!loadLciRawData(align_cfg, raw_imu_datas, align_imu_datas, gnss_datas))
    {
        return -1;
    }

    LciTrajectory backward_trajectory;
    if (!runBackwardLci(align_cfg, lci_cfg, raw_imu_datas, align_imu_datas, gnss_datas, nullptr, backward_trajectory))
    {
        std::cerr << "错误：后向松组合未生成有效结果。" << std::endl;
        return -1;
    }

    const char *output_file = ini_backward_lci.GetValue("backward_lci", "output_file", "./assets2/results/backward_lci_result.csv");
    saveLciResult(output_file, backward_trajectory.states, backward_trajectory.times);
    printLciSummary("后向松组合已完成", backward_trajectory);
    std::cout << "  输出文件: " << output_file << std::endl;
    return 0;
}
