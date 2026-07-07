# combined_nav 组合导航程序说明

`combined_nav` 是 GNSS/IMU 松组合导航实验工程，当前程序包提供离线正向解算、离线反向解算、前后向融合解算、实时正向解算以及实时文件回放发送工具。本文档按自编程序说明文档要求编写，重点说明编译环境、文件结构、可执行程序接口、输入输出数据格式和配置文件使用方法。

> 运行提示：所有可执行程序都固定读取 `./config/combined_nav.ini`，请在项目根目录运行程序。

## 1. 编程环境

| 项目 | 当前工程使用情况 |
| --- | --- |
| 操作系统 | Windows 环境优先；实时通信模块在 Windows 下链接 `ws2_32`，非 Windows 下使用 POSIX socket |
| 编程语言 | C++11 |
| 构建工具 | CMake，工程要求 `cmake_minimum_required(VERSION 3.16)`；本机缓存记录为 CMake `4.2.0-rc1` |
| 编译器 | 本机缓存记录为 MinGW Makefiles，`g++.exe 14.2.0` |
| 第三方库 | Eigen 3.4.0 头文件库、`external/simpleini` 配置解析库 |
| 辅助工具 | MATLAB，使用 `readmatrix`、`readtable`、`detectImportOptions`、`interp1` 等函数进行结果对比和绘图 |

Eigen 路径通过 CMake 变量 `EIGEN3_INCLUDE_DIR` 指定，当前默认值为：

```powershell
D:/my_it/Eigeon/eigen-3.4.0
```

该目录下应能找到 `Eigen/Dense`。如果 Eigen 安装位置不同，需要在配置 CMake 时改为本机路径。

## 2. 文件结构

```text
combined_nav/
├─ CMakeLists.txt                  # 顶层 CMake 工程文件
├─ config/
│  └─ combined_nav.ini             # 组合导航统一配置文件
├─ src/
│  ├─ forward_LCI.cpp              # 离线正向松组合入口
│  ├─ backward_LCI.cpp             # 离线反向松组合入口
│  ├─ fused_LCI.cpp                # 前后向融合入口
│  ├─ realtime_forward_LCI.cpp     # 实时正向松组合接收端入口
│  ├─ realtime_file_sender.cpp     # 实时文件回放发送端入口
│  ├─ COMBINED/                    # 组合导航、对准、滤波、实时流模块
│  ├─ GNSS/                        # GNSS 数据结构与文件读取
│  ├─ IMU/                         # IMU 数据读取、惯导更新、结果保存
│  └─ COMMON/                      # 时间、姿态、常量、导航状态定义
├─ external/simpleini/             # INI 配置解析库
├─ plot-matlab/                    # 结果对比与绘图脚本
├─ assets/                         # 本地数据目录
├─ bin/                            # 编译后可执行程序输出目录
└─ lib/                            # 编译后静态库输出目录
```

`assets/`、`bin/`、`lib/`、`build/` 都属于本地数据或构建产物。

文件原始结构如下：

<img src=".\pic\文件结构.png" alt="文件结构" style="zoom:75%;" />

## 3. 编译方法

需要注意，项目需要防止在纯英文目录下。在项目根目录执行：

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DEIGEN3_INCLUDE_DIR=D:/my_it/Eigeon/eigen-3.4.0
cmake --build build
```

编译完成后，可执行程序输出到 `bin/`：

```text
bin/forward_LCI.exe
bin/backward_LCI.exe
bin/fused_LCI.exe
bin/realtime_forward_LCI.exe
bin/realtime_file_sender.exe
```

如果使用 Visual Studio 或其他编译器，可以省略 `-G "MinGW Makefiles"`，但仍需保证 `EIGEN3_INCLUDE_DIR` 指向正确的 Eigen 头文件目录。

<img src=".\pic\编译截图.png" alt="编译截图" style="zoom:50%;" />

## 4. 可执行程序接口

所有程序当前都没有命令行参数，统一从 `config/combined_nav.ini` 读取输入文件、输出文件和滤波参数。

| 可执行程序 | 运行命令 | 功能 | 主要输入配置 | 主要输出 |
| --- | --- | --- | --- | --- |
| `forward_LCI.exe` | `.\bin\forward_LCI.exe` | 离线正向松组合解算 | `[imu]`、`[gnss]`、`[alignment]`、`[forward_lci]` | `[forward_lci] output_file` |
| `backward_LCI.exe` | `.\bin\backward_LCI.exe` | 离线反向松组合解算 | 同上，并要求数据末端存在可用静止对准窗口 | `[backward_lci] output_file` |
| `fused_LCI.exe` | `.\bin\fused_LCI.exe` | 先运行正向和反向，再进行前后向融合 | 同上 | `[fused_lci] output_file` |
| `realtime_forward_LCI.exe` | `.\bin\realtime_forward_LCI.exe` | 启动 UDP/TCP 接收端，实时正向解算 | `[realtime_stream]`、`[realtime_forward_lci]`，以及 IMU/GNSS/对准/滤波配置 | `[realtime_forward_lci] output_file` |
| `realtime_file_sender.exe` | `.\bin\realtime_file_sender.exe` | 将文件中的 IMU/GNSS 数据按时间顺序回放到实时接收端 | `[realtime_stream]`、`[realtime_file_sender]` | 网络发送，无结果文件 |

离线解算的典型运行顺序：

```powershell
.\bin\forward_LCI.exe
.\bin\backward_LCI.exe
.\bin\fused_LCI.exe
```

实时回放的典型运行顺序：

```powershell
# 终端 1：先启动接收端
.\bin\realtime_forward_LCI.exe

# 终端 2：再启动文件发送端
.\bin\realtime_file_sender.exe
```

程序运行成功时会在终端输出起止 `sow`、IMU 历元数、GNSS 更新次数、最终位置/姿态/零偏/比例因子以及输出文件路径。实时接收端初始化成功后会输出 `Realtime forward LCI initialized` 和 `switched to live mode` 等信息。示例如下：

![运行结果](.\pic\运行结果.png)

## 5. 配置文件说明

示例配置文件为：

```text
config/combined_nav.ini
```

路径均按项目根目录解析。修改输入输出位置时，建议继续使用相对路径，例如 `./assets/...`。向量参数统一写成 `[x, y, z]`。

### 5.1 数据与传感器配置

| 配置段 | 参数 | 含义 |
| --- | --- | --- |
| `[imu]` | `input_file` | 离线 IMU 输入文件 |
| `[imu]` | `sample_rate` | IMU 采样率，单位 Hz |
| `[imu]` | `need_trans` | 是否执行坐标轴转换，`true` 或 `false` |
| `[imu]` | `acc_noise` | 加计白噪声，三轴向量 |
| `[imu]` | `gyro_noise` | 陀螺白噪声，配置中按 deg 相关单位填写，程序内部转为 rad |
| `[imu]` | `acc_random_walk` | 加计零偏随机游走 |
| `[imu]` | `gyro_random_walk` | 陀螺零偏随机游走，配置中按 deg 相关单位填写，程序内部转为 rad |
| `[gnss]` | `input_file` | 离线 GNSS 输入文件 |
| `[gnss]` | `lever_arm` | GNSS 天线相对 IMU 的杆臂，机体系三轴，单位 m |

### 5.2 对准与滤波配置

| 配置段          | 参数                                                         | 含义                                             |
| --------------- | ------------------------------------------------------------ | :----------------------------------------------- |
| `[alignment]`   | `start_search_length`                                        | 起始静止窗口搜索时长，单位 s                     |
| `[alignment]`   | `end_search_length`                                          | 末端静止窗口搜索时长，单位 s                     |
| `[alignment]`   | `min_static_duration`                                        | 静止窗口最短持续时间，单位 s                     |
| `[alignment]`   | `start_align_acc_bias`、`start_align_gyro_bias`              | 起始对准使用的加计/陀螺零偏                      |
| `[alignment]`   | `end_align_acc_bias`、`end_align_gyro_bias`                  | 反向解算末端对准使用的加计/陀螺零偏              |
| `[forward_lci]` | `gnss_time_tolerance`                                        | GNSS 与 IMU 时间匹配容差，单位 s                 |
| `[forward_lci]` | `use_gnss_velocity`                                          | 是否使用 GNSS 速度更新                           |
| `[forward_lci]` | `use_zupt`                                                   | 是否启用零速更新                                 |
| `[forward_lci]` | `use_nhc`                                                    | 是否启用非完整性约束                             |
| `[forward_lci]` | `corr_time`                                                  | IMU 误差相关时间，单位 s                         |
| `[forward_lci]` | `init_pos_std`、`init_vel_std`、`init_att_std_deg`           | 初始位置、速度、姿态标准差                       |
| `[forward_lci]` | `init_gyro_bias_std`、`init_acc_bias_std`、`init_gyro_scale_std`、`init_acc_scale_std` | 初始误差状态标准差；配置中未填写时使用程序默认值 |
| `[forward_lci]` | `zupt_std`、`zupt_acc_threshold`、`zupt_gyro_threshold`、`zupt_min_duration` | 零速更新参数                                     |
| `[forward_lci]` | `nhc_std`                                                    | 非完整性约束标准差，按车体系 x/y/z 给定          |

`backward_LCI.exe` 和 `fused_LCI.exe` 当前复用 `[forward_lci]` 中的滤波参数；`[backward_lci]` 和 `[fused_lci]` 只配置各自结果文件路径。

### 5.3 实时配置

| 配置段 | 参数 | 含义 |
| --- | --- | --- |
| `[realtime_stream]` | `protocol` | `udp` 或 `tcp` |
| `[realtime_stream]` | `bind_host`、`port` | 实时接收端绑定地址和端口 |
| `[realtime_stream]` | `target_host`、`target_port` | 文件发送端目标地址和端口 |
| `[realtime_stream]` | `tcp_backlog` | TCP 监听队列长度 |
| `[realtime_stream]` | `poll_timeout_ms` | 接收轮询超时时间，单位 ms |
| `[realtime_stream]` | `max_packet_size` | 单次接收最大报文长度，单位 byte |
| `[realtime_stream]` | `default_gps_week` | 实时记录缺省 GPS week 配置项；建议实时 IMU/GNSS 报文显式提供 week |
| `[realtime_forward_lci]` | `output_file` | 实时正向解算结果文件 |
| `[realtime_forward_lci]` | `print_interval` | 每输出多少个历元打印一次状态 |
| `[realtime_forward_lci]` | `alignment_check_interval` | 未初始化阶段每多少个 IMU 历元尝试一次静止对准 |
| `[realtime_file_sender]` | `imu_file`、`gnss_file` | 文件回放使用的 IMU/GNSS 输入文件 |
| `[realtime_file_sender]` | `speed` | 回放倍速，`1.0` 表示按原始时间间隔发送 |
| `[realtime_file_sender]` | `loop` | 是否循环发送 |
| `[realtime_file_sender]` | `send_imu`、`send_gnss` | 是否发送 IMU/GNSS 记录 |
| `[realtime_file_sender]` | `startup_wait_ms` | 打开发送端后的等待时间，单位 ms |
| `[realtime_file_sender]` | `log_interval` | 每发送多少条记录打印一次日志 |

## 6. 输入数据格式

### 6.1 离线 IMU 输入文件

IMU 输入为普通文本，每条有效数据 7 列，空白分隔：

```text
sow gx_deg_s gy_deg_s gz_deg_s ax_mps2 ay_mps2 az_mps2
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `sow` | GPS 周内秒 |
| `gx_deg_s gy_deg_s gz_deg_s` | 三轴角速度，单位 deg/s |
| `ax_mps2 ay_mps2 az_mps2` | 三轴加速度/比力，单位 m/s^2 |

程序根据 `[imu] sample_rate` 计算 `dt = 1 / sample_rate`，并把角速度和加速度转换为角增量、速度增量。包含 `Time` 的表头行会被跳过。离线 IMU 文件不提供 GPS week 时，输出中的 `week` 默认为 0。

### 6.2 离线 GNSS 输入文件

GNSS 输入为普通文本，至少需要前 14 列数字，空白分隔。程序读取前 14 列：

```text
week sow lat_deg lon_deg h_m pos_std_n pos_std_e pos_std_d vn ve vu vel_std_n vel_std_e vel_std_u
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `week`、`sow` | GPS 周和周内秒 |
| `lat_deg lon_deg h_m` | 纬度、经度、高程 |
| `pos_std_n pos_std_e pos_std_d` | 位置标准差，单位 m |
| `vn ve vu` | 北、东、天速度，单位 m/s |
| `vel_std_n vel_std_e vel_std_u` | 速度标准差，单位 m/s |

程序内部使用 NED 速度，因此读取 GNSS 时会将 `vu` 转换为 `vd = -vu`。

### 6.3 实时流输入

实时接收端按行解析文本报文，每行以 `\n` 结束。字段可以用空格或逗号分隔，记录类型前缀 `IMU`、`GNSS` 建议保留。

IMU 报文格式：

```text
IMU week sow gx_deg_s gy_deg_s gz_deg_s ax_mps2 ay_mps2 az_mps2
```

也支持省略 `week`：

```text
IMU sow gx_deg_s gy_deg_s gz_deg_s ax_mps2 ay_mps2 az_mps2
```

GNSS 报文格式：

```text
GNSS week sow lat_deg lon_deg h_m pos_std_n pos_std_e pos_std_d vn ve vu vel_std_n vel_std_e vel_std_u
```

文件发送端 `realtime_file_sender.exe` 会自动把 `[realtime_file_sender] imu_file` 和 `gnss_file` 转成上述报文格式，并按 `sow` 排序发送。

## 7. 输出结果格式

离线和实时组合导航结果均为 CSV，首行为表头：

```text
week,sow,lat_deg,lon_deg,h_m,vn,ve,vd,roll_deg,pitch_deg,yaw_deg,bgx,bgy,bgz,bax,bay,baz,sgx,sgy,sgz,sax,say,saz
```

字段说明：

| 字段 | 含义 |
| --- | --- |
| `week`、`sow` | GPS 周和周内秒 |
| `lat_deg lon_deg h_m` | 纬度、经度、高程 |
| `vn ve vd` | NED 速度，单位 m/s，其中 `vd` 向下为正 |
| `roll_deg pitch_deg yaw_deg` | 姿态角，单位 deg |
| `bgx bgy bgz` | 陀螺零偏估计，单位 rad/s |
| `bax bay baz` | 加计零偏估计，单位 m/s^2 |
| `sgx sgy sgz` | 陀螺比例因子估计，无量纲 |
| `sax say saz` | 加计比例因子估计，无量纲 |

默认输出路径在 `config/combined_nav.ini` 中配置，例如：

```text
./assets/results/forward_lci_result.csv
./assets/results/backward_lci_result.csv
./assets/results/fused_lci_result.csv
./assets/results/realtime_forward_lci_result.csv
```

运行前请确保 `assets/results/` 目录存在，否则程序无法创建结果文件。

## 8. 辅助 MATLAB 脚本

辅助脚本位于 `plot-matlab/`，用于复现实习报告中的结果对比和绘图。脚本中的文件路径是绝对路径，换机器或换目录后需要先修改脚本开头的文件变量。

| 脚本 | 用途 | 需要的主要文件 |
| --- | --- | --- |
| `myres_comp_with_ref.m` | 将本程序输出结果与参考组合导航结果对比，绘制位置、速度、姿态和零偏曲线 | `assets/ref.pos`、`assets/results/fused_lci_result.csv` |
| `compare_with_rtk.m` | 对比参考结果和 RTK/GNSS 结果，输出位置速度统计并绘图 | `assets/ref.pos`、`assets/gnss_20260602_110011_682828.pos` |
| `posming_compare.m` | 对比 POSMind 松组合/紧组合结果，含姿态与 IMU 误差项分析 | `assets/LCI.pos`、`assets/TCI.pos` |

在 MATLAB 中进入项目目录修改对应文件路径运行对应脚本即可。

## 9. 数据与打包说明

- `assets/activate_data.txt`：IMU 原始文本数据，供离线解算和实时回放使用。
- `assets/gnss_20260602_110011_682828.pos`：GNSS 输入数据。
- `assets/ref.pos`、`assets/LCI.pos`、`assets/TCI.pos`：辅助对比绘图使用的参考结果文件。
- `assets/results/*.csv`：程序输出结果，可用于复现实习报告中的图表。

## 10. 常见问题

- 程序提示无法读取 `./config/combined_nav.ini`：请确认从项目根目录运行，而不是从 `bin/` 目录运行。
- 程序提示无法写结果文件：请确认 `assets/results/` 已创建，且配置中的输出路径有效。
- 编译时报找不到 `Eigen/Dense`：请用 `-DEIGEN3_INCLUDE_DIR=...` 指定 Eigen 头文件目录。
- 反向或融合解算失败：请检查数据末端是否存在满足 `[alignment]` 配置要求的静止窗口。
- 实时接收不到数据：请确认接收端先启动，`protocol`、`bind_host`、`port` 与发送端 `target_host`、`target_port` 一致。
