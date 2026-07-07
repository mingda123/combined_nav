#include <iostream>
#include <string>
#include <vector>

#include "simpleini/SimpleIni.h"

#include "COMBINED/lci_config_helper.h"
#include "COMBINED/lci_runner.h"

int main()
{
    system("chcp 65001");

    CSimpleIniA ini_fused_lci;
    SI_Error rc = ini_fused_lci.LoadFile("./config/combined_nav.ini");
    if (rc < 0)
    {
        printf("load %s ini file failure\n", "./config/combined_nav.ini");
        return -1;
    }

    AlignConfig align_cfg = loadAlignConfig(ini_fused_lci);
    LciFilterConfig lci_cfg = loadLciFilterConfig(ini_fused_lci, "forward_lci", align_cfg);

    std::vector<IMUData> raw_imu_datas;
    std::vector<IMUData> align_imu_datas;
    std::vector<GNSSData> gnss_datas;
    if (!loadLciRawData(align_cfg, raw_imu_datas, align_imu_datas, gnss_datas))
    {
        return -1;
    }

    LciTrajectory forward_trajectory;
    LciTrajectory backward_trajectory;
    LciTrajectory fused_trajectory;

    if (!runForwardLci(align_cfg, lci_cfg, raw_imu_datas, align_imu_datas, gnss_datas, forward_trajectory))
    {
        std::cerr << "错误：前向松组合未生成有效结果。" << std::endl;
        return -1;
    }

    if (!runBackwardLci(align_cfg, lci_cfg, raw_imu_datas, align_imu_datas, gnss_datas, nullptr, backward_trajectory))
    {
        std::cerr << "错误：后向松组合未生成有效结果。" << std::endl;
        return -1;
    }

    if (!fuseForwardBackwardLci(forward_trajectory, backward_trajectory, fused_trajectory))
    {
        std::cerr << "错误：前后向融合未生成有效结果。" << std::endl;
        return -1;
    }

    const char *output_file = ini_fused_lci.GetValue("fused_lci", "output_file", "./assets2/results/fused_lci_result.csv");
    saveLciResult(output_file, fused_trajectory.states, fused_trajectory.times);
    printLciSummary("前后向融合已完成", fused_trajectory);
    std::cout << "  输出文件: " << output_file << std::endl;
    return 0;
}
