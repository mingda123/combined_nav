%% 对比分析POSMIND松组合（LCI）与紧组合（TCI）解算结果（含姿态角）
% 数据格式：17列，前两行为标题（跳过）
% 列顺序：week, sec, Lat, Lon, EllipHgt, VN, VE, VU,
%         Roll, Pitch, Heading, AccelBiasX, AccelBiasY, AccelBiasZ,
%         GyroBiasX, GyroBiasY, GyroBiasZ
% 单位：位置(deg, deg, m)，速度(m/s)，姿态角(deg)，
%       加速度偏置(mg)，陀螺偏置(deg/h)

clear; clc; close all;

%% ==================== 1. 读取数据 ====================
file1 = 'D:\my_project\combined_nav\assets2\LCI.pos';   % 松组合结果（参考）
file2 = 'D:\my_project\combined_nav\assets2\TCI.pos';   % 紧组合结果（待比较）

% 定义变量名（17列）
varNames = {'week','sec','Lat','Lon','Hgt','VN','VE','VU', ...
            'Roll','Pitch','Heading','AccX','AccY','AccZ', ...
            'GyrX','GyrY','GyrZ'};

% ---- 读取文件1 ----
opts1 = detectImportOptions(file1, 'FileType', 'text');
opts1.DataLines = [3, Inf];                     % 跳过前两行标题
opts1.VariableNames = varNames;                 % 手动指定列名
opts1.VariableNamingRule = 'preserve';          % 保持名称不变
data1 = readtable(file1, opts1);

% ---- 读取文件2 ----
opts2 = detectImportOptions(file2, 'FileType', 'text');
opts2.DataLines = [3, Inf];
opts2.VariableNames = varNames;
opts2.VariableNamingRule = 'preserve';
data2 = readtable(file2, opts2);

fprintf('数据读取完成：松组合 %d 行，紧组合 %d 行\n', height(data1), height(data2));

%% ==================== 2. 时间对齐 ====================
% 构建连续时间（相对秒数）
t1 = data1.week * 7*86400 + data1.sec;
t2 = data2.week * 7*86400 + data2.sec;

% 公共时间轴（交集）
t_start = max(min(t1), min(t2));
t_end   = min(max(t1), max(t2));
t_common = unique([t1; t2]);
t_common = t_common(t_common >= t_start & t_common <= t_end);
fprintf('公共时间范围：%.2f ~ %.2f 秒，共 %d 个插值点\n', t_start, t_end, length(t_common));

% 需要插值的变量（除week和sec外）
vars = {'Lat','Lon','Hgt','VN','VE','VU','Roll','Pitch','Heading', ...
        'AccX','AccY','AccZ','GyrX','GyrY','GyrZ'};
nVars = length(vars);

% 线性插值对齐
interp1_data = zeros(length(t_common), nVars);
interp2_data = zeros(length(t_common), nVars);
for i = 1:nVars
    interp1_data(:,i) = interp1(t1, data1.(vars{i}), t_common, 'linear', 'extrap');
    interp2_data(:,i) = interp1(t2, data2.(vars{i}), t_common, 'linear', 'extrap');
end

data1_aligned = array2table(interp1_data, 'VariableNames', vars);
data2_aligned = array2table(interp2_data, 'VariableNames', vars);
time_sec = t_common - t_start;          % 相对时间（秒）
time_min = time_sec / 60;               % 相对时间（分钟）

%% ==================== 3. 计算误差（紧组合 - 松组合） ====================
error = data2_aligned{:,:} - data1_aligned{:,:};
error_table = array2table(error, 'VariableNames', vars);

% 提取各类误差
pos_err  = error_table{:, {'Lat','Lon','Hgt'}};       % 位置误差 [deg, deg, m]
vel_err  = error_table{:, {'VN','VE','VU'}};          % 速度误差 [m/s]
att_err  = error_table{:, {'Roll','Pitch','Heading'}}; % 姿态误差 [deg, deg, deg]
acc_err  = error_table{:, {'AccX','AccY','AccZ'}};     % 加速度偏置误差 [mg]
gyr_err  = error_table{:, {'GyrX','GyrY','GyrZ'}};     % 陀螺偏置误差 [deg/h]

%% ==================== 4. 统计误差指标 ====================
fprintf('\n========== 误差统计（紧组合 - 松组合） ==========\n');

% 位置误差
mean_pos = mean(pos_err);
std_pos  = std(pos_err);
rms_pos  = sqrt(mean(pos_err.^2));
fprintf('位置误差 (Lat, Lon, Hgt):\n');
fprintf('  均值 = [%.4e, %.4e, %.2f] (deg, deg, m)\n', mean_pos);
fprintf('  标准差 = [%.4e, %.4e, %.2f]\n', std_pos);
fprintf('  RMS   = [%.4e, %.4e, %.2f]\n', rms_pos);

% 速度误差
mean_vel = mean(vel_err);
std_vel  = std(vel_err);
rms_vel  = sqrt(mean(vel_err.^2));
fprintf('\n速度误差 (VN, VE, VU):\n');
fprintf('  均值 = [%.3f, %.3f, %.3f] m/s\n', mean_vel);
fprintf('  标准差 = [%.3f, %.3f, %.3f] m/s\n', std_vel);
fprintf('  RMS   = [%.3f, %.3f, %.3f] m/s\n', rms_vel);

% 姿态误差（新增重点）
mean_att = mean(att_err);
std_att  = std(att_err);
rms_att  = sqrt(mean(att_err.^2));
fprintf('\n姿态误差 (Roll, Pitch, Heading):\n');
fprintf('  均值 = [%.3f, %.3f, %.3f] deg\n', mean_att);
fprintf('  标准差 = [%.3f, %.3f, %.3f] deg\n', std_att);
fprintf('  RMS   = [%.3f, %.3f, %.3f] deg\n', rms_att);

% 加速度偏置误差
mean_acc = mean(acc_err);
std_acc  = std(acc_err);
rms_acc  = sqrt(mean(acc_err.^2));
fprintf('\n加速度偏置误差 (X, Y, Z):\n');
fprintf('  均值 = [%.3f, %.3f, %.3f] mg\n', mean_acc);
fprintf('  标准差 = [%.3f, %.3f, %.3f] mg\n', std_acc);
fprintf('  RMS   = [%.3f, %.3f, %.3f] mg\n', rms_acc);

% 陀螺偏置误差
mean_gyr = mean(gyr_err);
std_gyr  = std(gyr_err);
rms_gyr  = sqrt(mean(gyr_err.^2));
fprintf('\n陀螺偏置误差 (X, Y, Z):\n');
fprintf('  均值 = [%.3f, %.3f, %.3f] deg/h\n', mean_gyr);
fprintf('  标准差 = [%.3f, %.3f, %.3f] deg/h\n', std_gyr);
fprintf('  RMS   = [%.3f, %.3f, %.3f] deg/h\n', rms_gyr);

% 最大绝对误差
fprintf('\n最大绝对误差：\n');
fprintf('  位置 (Lat,Lon,Hgt) = [%.4e, %.4e, %.2f]\n', max(abs(pos_err)));
fprintf('  速度 (VN,VE,VU)    = [%.3f, %.3f, %.3f] m/s\n', max(abs(vel_err)));
fprintf('  姿态 (Roll,Pitch,Heading) = [%.3f, %.3f, %.3f] deg\n', max(abs(att_err)));
fprintf('  陀螺偏置 (X,Y,Z)   = [%.3f, %.3f, %.3f] deg/h\n', max(abs(gyr_err)));

%% ==================== 5. 绘图 ====================

% ---------- 5.1 平面轨迹对比 ----------
figure('Name', '平面轨迹对比', 'Position', [100,100,800,600]);
plot(data1.Lon, data1.Lat, 'b-', 'LineWidth', 1.5); hold on;
plot(data2.Lon, data2.Lat, 'r--', 'LineWidth', 1.5);
plot(data1_aligned.Lon, data1_aligned.Lat, 'b.', 'MarkerSize', 3);
plot(data2_aligned.Lon, data2_aligned.Lat, 'r.', 'MarkerSize', 3);
xlabel('经度 (deg)'); ylabel('纬度 (deg)');
legend('松组合原始', '紧组合原始', '松组合对齐', '紧组合对齐', 'Location','best');
title('平面轨迹对比'); axis equal; grid on;

% ---------- 5.2 位置误差时间序列 ----------
figure('Name', '位置误差', 'Position', [100,100,800,600]);
subplot(3,1,1);
plot(time_min, pos_err(:,1)*1e3, 'b-', 'LineWidth', 1);
ylabel('\Delta 纬度 (mdeg)'); grid on; title('纬度误差');
subplot(3,1,2);
plot(time_min, pos_err(:,2)*1e3, 'r-', 'LineWidth', 1);
ylabel('\Delta 经度 (mdeg)'); grid on; title('经度误差');
subplot(3,1,3);
plot(time_min, pos_err(:,3), 'k-', 'LineWidth', 1);
ylabel('\Delta 高程 (m)'); grid on; title('高程误差');
xlabel('时间 (min)'); sgtitle('位置误差随时间变化');

% ---------- 5.3 速度误差时间序列 ----------
figure('Name', '速度误差', 'Position', [100,100,800,600]);
subplot(3,1,1);
plot(time_min, vel_err(:,1), 'b-', 'LineWidth', 1);
ylabel('\Delta VN (m/s)'); grid on; title('北向速度误差');
subplot(3,1,2);
plot(time_min, vel_err(:,2), 'r-', 'LineWidth', 1);
ylabel('\Delta VE (m/s)'); grid on; title('东向速度误差');
subplot(3,1,3);
plot(time_min, vel_err(:,3), 'k-', 'LineWidth', 1);
ylabel('\Delta VU (m/s)'); grid on; title('天向速度误差');
xlabel('时间 (min)'); sgtitle('速度误差随时间变化');

% ---------- 5.4 姿态误差时间序列（新增重点） ----------
figure('Name', '姿态误差', 'Position', [100,100,800,600]);
subplot(3,1,1);
plot(time_min, att_err(:,1), 'b-', 'LineWidth', 1);
ylabel('\Delta Roll (deg)'); grid on; title('横滚角误差');
subplot(3,1,2);
plot(time_min, att_err(:,2), 'r-', 'LineWidth', 1);
ylabel('\Delta Pitch (deg)'); grid on; title('俯仰角误差');
subplot(3,1,3);
plot(time_min, att_err(:,3), 'k-', 'LineWidth', 1);
ylabel('\Delta Heading (deg)'); grid on; title('航向角误差');
xlabel('时间 (min)'); sgtitle('姿态误差随时间变化');

% ---------- 5.5 陀螺偏置误差（辅助分析） ----------
figure('Name', '陀螺偏置误差', 'Position', [100,100,800,600]);
subplot(3,1,1);
plot(time_min, gyr_err(:,1), 'b-', 'LineWidth', 1);
ylabel('\Delta GyrX (deg/h)'); grid on; title('X轴陀螺偏置误差');
subplot(3,1,2);
plot(time_min, gyr_err(:,2), 'r-', 'LineWidth', 1);
ylabel('\Delta GyrY (deg/h)'); grid on; title('Y轴陀螺偏置误差');
subplot(3,1,3);
plot(time_min, gyr_err(:,3), 'k-', 'LineWidth', 1);
ylabel('\Delta GyrZ (deg/h)'); grid on; title('Z轴陀螺偏置误差');
xlabel('时间 (min)'); sgtitle('陀螺偏置误差');

% ---------- 5.6 加速度计偏置误差（可选） ----------
figure('Name', '加速度计偏置误差', 'Position', [100,100,800,600]);
subplot(3,1,1);
plot(time_min, acc_err(:,1), 'b-', 'LineWidth', 1);
ylabel('\Delta AccX (mg)'); grid on; title('X轴加速度偏置误差');
subplot(3,1,2);
plot(time_min, acc_err(:,2), 'r-', 'LineWidth', 1);
ylabel('\Delta AccY (mg)'); grid on; title('Y轴加速度偏置误差');
subplot(3,1,3);
plot(time_min, acc_err(:,3), 'k-', 'LineWidth', 1);
ylabel('\Delta AccZ (mg)'); grid on; title('Z轴加速度偏置误差');
xlabel('时间 (min)'); sgtitle('加速度计偏置误差');

%% ==================== 6. 误差分布直方图（可选） ====================
% 取消注释以查看误差分布
% figure('Name', '误差分布直方图', 'Position', [100,100,1200,900]);
% for i = 1:3
%     subplot(3,4,i); histogram(pos_err(:,i), 50); title(sprintf('位置误差%d',i));
%     subplot(3,4,3+i); histogram(vel_err(:,i), 50); title(sprintf('速度误差%d',i));
%     subplot(3,4,6+i); histogram(att_err(:,i), 50); title(sprintf('姿态误差%d',i));
%     subplot(3,4,9+i); histogram(gyr_err(:,i), 50); title(sprintf('陀螺偏置%d',i));
% end
% sgtitle('误差分布直方图');

fprintf('\n分析完成！所有图形已生成。\n');