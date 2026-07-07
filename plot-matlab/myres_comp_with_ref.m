%% 将我的解算结果与组合导航参考结果比较
clc;
clear;
close all;

%% =========================
% 1. 文件路径：只改这里
% ==========================
ref_file = 'D:\my_project\combined_nav\assets\ref.pos';
cur_file = 'D:\my_project\combined_nav\assets\results\fused_lci_result.csv';

%% =========================
% 2. 读取文件
% ==========================
% 参考结果：前两行为表头
ref_raw = readmatrix(ref_file, 'FileType', 'text', 'NumHeaderLines', 2);

% 当前结果：csv，第一行为表头（修改点：跳过表头）
cur_raw = readmatrix(cur_file, 'NumHeaderLines', 1);

if size(ref_raw, 2) < 14
    error('参考结果文件列数不足，至少应为 14 列。');
end

if size(cur_raw, 2) < 17
    error('当前结果文件列数不足，至少应为 17 列。');
end

% 去掉可能的全 NaN 行
ref_raw = ref_raw(~all(isnan(ref_raw), 2), :);
cur_raw = cur_raw(~all(isnan(cur_raw), 2), :);

%% =========================
% 3. 提取字段
% ==========================
% 参考结果
ref_week = ref_raw(:, 1);
ref_sow  = ref_raw(:, 2);
ref_lat  = ref_raw(:, 3);
ref_lon  = ref_raw(:, 4);
ref_h    = ref_raw(:, 5);
ref_vn   = ref_raw(:, 6);
ref_ve   = ref_raw(:, 7);
ref_vu   = ref_raw(:, 8);

% 当前结果
cur_week = cur_raw(:, 1);
cur_sow  = cur_raw(:, 2);
cur_lat  = cur_raw(:, 3);
cur_lon  = cur_raw(:, 4);
cur_h    = cur_raw(:, 5);
cur_vn   = cur_raw(:, 6);
cur_ve   = cur_raw(:, 7);
cur_vd   = cur_raw(:, 8);
% 新增：提取姿态角（第9~11列）
cur_roll  = cur_raw(:, 9);
cur_pitch = cur_raw(:, 10);
cur_yaw   = cur_raw(:, 11);

%% ===== 新增：提取陀螺零偏（12~14）和加计零偏（15~17） =====
if size(cur_raw, 2) >= 17
    cur_bgx = cur_raw(:, 12);
    cur_bgy = cur_raw(:, 13);
    cur_bgz = cur_raw(:, 14);
    cur_bax = cur_raw(:, 15);
    cur_bay = cur_raw(:, 16);
    cur_baz = cur_raw(:, 17);
else
    error('当前结果文件缺少陀螺/加计零偏列（至少需要17列）。');
end

% 关键：统一垂向速度符号
cur_vu = -cur_vd;

%% =========================
% 4. 按 sow 排序
% ==========================
[ref_sow, idx] = sort(ref_sow);
ref_week = ref_week(idx);
ref_lat  = ref_lat(idx);
ref_lon  = ref_lon(idx);
ref_h    = ref_h(idx);
ref_vn   = ref_vn(idx);
ref_ve   = ref_ve(idx);
ref_vu   = ref_vu(idx);

[cur_sow, idx] = sort(cur_sow);
cur_week = cur_week(idx);
cur_lat  = cur_lat(idx);
cur_lon  = cur_lon(idx);
cur_h    = cur_h(idx);
cur_vn   = cur_vn(idx);
cur_ve   = cur_ve(idx);
cur_vd   = cur_vd(idx);
cur_vu   = cur_vu(idx);          % 注意顺序，cur_vu已由-cur_vd得到
% 新增姿态角排序
cur_roll  = cur_roll(idx);
cur_pitch = cur_pitch(idx);
cur_yaw   = cur_yaw(idx);
% ===== 新增：零偏排序 =====
cur_bgx = cur_bgx(idx);
cur_bgy = cur_bgy(idx);
cur_bgz = cur_bgz(idx);
cur_bax = cur_bax(idx);
cur_bay = cur_bay(idx);
cur_baz = cur_baz(idx);

%% =========================
% 5. 去除重复 sow
% ==========================
[ref_sow, ia] = unique(ref_sow, 'stable');
ref_week = ref_week(ia);
ref_lat  = ref_lat(ia);
ref_lon  = ref_lon(ia);
ref_h    = ref_h(ia);
ref_vn   = ref_vn(ia);
ref_ve   = ref_ve(ia);
ref_vu   = ref_vu(ia);

[cur_sow, ia] = unique(cur_sow, 'stable');
cur_week = cur_week(ia);
cur_lat  = cur_lat(ia);
cur_lon  = cur_lon(ia);
cur_h    = cur_h(ia);
cur_vn   = cur_vn(ia);
cur_ve   = cur_ve(ia);
cur_vd   = cur_vd(ia);
cur_vu   = cur_vu(ia);
% 新增姿态角去重
cur_roll  = cur_roll(ia);
cur_pitch = cur_pitch(ia);
cur_yaw   = cur_yaw(ia);
% ===== 新增：零偏去重 =====
cur_bgx = cur_bgx(ia);
cur_bgy = cur_bgy(ia);
cur_bgz = cur_bgz(ia);
cur_bax = cur_bax(ia);
cur_bay = cur_bay(ia);
cur_baz = cur_baz(ia);

%% =========================
% 6. 只按 sow 取重叠时间段
%    不使用 GPS week
% ==========================
start_sow = max(min(ref_sow), min(cur_sow));
end_sow   = min(max(ref_sow), max(cur_sow));

if end_sow <= start_sow
    error('两个结果文件在 sow 上没有重叠时间段。');
end

ref_mask = (ref_sow >= start_sow) & (ref_sow <= end_sow);
cur_mask = (cur_sow >= start_sow) & (cur_sow <= end_sow);

ref_sow2 = ref_sow(ref_mask);
ref_lat2 = ref_lat(ref_mask);
ref_lon2 = ref_lon(ref_mask);
ref_h2   = ref_h(ref_mask);
ref_vn2  = ref_vn(ref_mask);
ref_ve2  = ref_ve(ref_mask);
ref_vu2  = ref_vu(ref_mask);

cur_sow2 = cur_sow(cur_mask);
cur_lat2 = cur_lat(cur_mask);
cur_lon2 = cur_lon(cur_mask);
cur_h2   = cur_h(cur_mask);
cur_vn2  = cur_vn(cur_mask);
cur_ve2  = cur_ve(cur_mask);
cur_vu2  = cur_vu(cur_mask);
% 新增姿态角重叠筛选
cur_roll2  = cur_roll(cur_mask);
cur_pitch2 = cur_pitch(cur_mask);
cur_yaw2   = cur_yaw(cur_mask);
% ===== 新增：零偏重叠筛选 =====
cur_bgx2 = cur_bgx(cur_mask);
cur_bgy2 = cur_bgy(cur_mask);
cur_bgz2 = cur_bgz(cur_mask);
cur_bax2 = cur_bax(cur_mask);
cur_bay2 = cur_bay(cur_mask);
cur_baz2 = cur_baz(cur_mask);

%% =========================
% 7. 当前结果插值到参考结果时刻
%    仍然只使用 sow
% ==========================
t_all = ref_sow2;

% 再用当前结果的实际插值区间裁一遍，避免 interp1 边界出 NaN
valid_t = (t_all >= cur_sow2(1)) & (t_all <= cur_sow2(end));

t       = t_all(valid_t);
ref_lat2 = ref_lat2(valid_t);
ref_lon2 = ref_lon2(valid_t);
ref_h2   = ref_h2(valid_t);
ref_vn2  = ref_vn2(valid_t);
ref_ve2  = ref_ve2(valid_t);
ref_vu2  = ref_vu2(valid_t);

cur_lat_i = interp1(cur_sow2, cur_lat2, t, 'linear');
cur_lon_i = interp1(cur_sow2, cur_lon2, t, 'linear');
cur_h_i   = interp1(cur_sow2, cur_h2,   t, 'linear');
cur_vn_i  = interp1(cur_sow2, cur_vn2,  t, 'linear');
cur_ve_i  = interp1(cur_sow2, cur_ve2,  t, 'linear');
cur_vu_i  = interp1(cur_sow2, cur_vu2,  t, 'linear');
% 新增姿态角插值
cur_roll_i  = interp1(cur_sow2, cur_roll2,  t, 'linear');
cur_pitch_i = interp1(cur_sow2, cur_pitch2, t, 'linear');
cur_yaw_i   = interp1(cur_sow2, cur_yaw2,   t, 'linear');
% ===== 新增：零偏插值 =====
cur_bgx_i = interp1(cur_sow2, cur_bgx2, t, 'linear');
cur_bgy_i = interp1(cur_sow2, cur_bgy2, t, 'linear');
cur_bgz_i = interp1(cur_sow2, cur_bgz2, t, 'linear');
cur_bax_i = interp1(cur_sow2, cur_bax2, t, 'linear');
cur_bay_i = interp1(cur_sow2, cur_bay2, t, 'linear');
cur_baz_i = interp1(cur_sow2, cur_baz2, t, 'linear');

% 如果插值后仍有非法值，再统一清一次
valid = isfinite(cur_lat_i) & isfinite(cur_lon_i) & isfinite(cur_h_i) & ...
        isfinite(cur_vn_i)  & isfinite(cur_ve_i)  & isfinite(cur_vu_i) & ...
        isfinite(ref_lat2)  & isfinite(ref_lon2)  & isfinite(ref_h2)   & ...
        isfinite(ref_vn2)   & isfinite(ref_ve2)   & isfinite(ref_vu2) & ...
        isfinite(cur_roll_i) & isfinite(cur_pitch_i) & isfinite(cur_yaw_i);   % 新增姿态角有效性

t        = t(valid);
ref_lat2 = ref_lat2(valid);
ref_lon2 = ref_lon2(valid);
ref_h2   = ref_h2(valid);
ref_vn2  = ref_vn2(valid);
ref_ve2  = ref_ve2(valid);
ref_vu2  = ref_vu2(valid);

cur_lat_i = cur_lat_i(valid);
cur_lon_i = cur_lon_i(valid);
cur_h_i   = cur_h_i(valid);
cur_vn_i  = cur_vn_i(valid);
cur_ve_i  = cur_ve_i(valid);
cur_vu_i  = cur_vu_i(valid);
% 新增姿态角有效性筛选
cur_roll_i  = cur_roll_i(valid);
cur_pitch_i = cur_pitch_i(valid);
cur_yaw_i   = cur_yaw_i(valid);
% ===== 新增：零偏也同步进行有效性筛选，保持与位置速度相同的时间点 =====
cur_bgx_i = cur_bgx_i(valid);
cur_bgy_i = cur_bgy_i(valid);
cur_bgz_i = cur_bgz_i(valid);
cur_bax_i = cur_bax_i(valid);
cur_bay_i = cur_bay_i(valid);
cur_baz_i = cur_baz_i(valid);

%% =========================
% 8. BLH -> 局部 NEU
%    用参考轨迹起点作原点
% ==========================
a  = 6378137.0;
f  = 1 / 298.257223563;
e2 = f * (2 - f);

lat0_deg = ref_lat2(1);
lon0_deg = ref_lon2(1);
h0       = ref_h2(1);

lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

ref_lat_rad = deg2rad(ref_lat2);
ref_lon_rad = deg2rad(ref_lon2);

sin_lat0 = sin(lat0);
den0 = sqrt(1 - e2 * sin_lat0^2);
Rm0 = a * (1 - e2) / den0^3;
Rn0 = a / den0;

ref_n = (ref_lat_rad - lat0) .* (Rm0 + h0);
ref_e = (ref_lon_rad - lon0) .* (Rn0 + h0) .* cos(lat0);
ref_u = ref_h2 - h0;

cur_lat_rad = deg2rad(cur_lat_i);
cur_lon_rad = deg2rad(cur_lon_i);

cur_n = (cur_lat_rad - lat0) .* (Rm0 + h0);
cur_e = (cur_lon_rad - lon0) .* (Rn0 + h0) .* cos(lat0);
cur_u = cur_h_i - h0;

%% =========================
% 9. 相对参考轨迹的 NEU 位置误差
% ==========================
sin_ref_lat = sin(ref_lat_rad);
den = sqrt(1 - e2 .* sin_ref_lat.^2);
Rm = a * (1 - e2) ./ (den.^3);
Rn = a ./ den;

pos_err_n = (cur_lat_rad - ref_lat_rad) .* (Rm + ref_h2);
pos_err_e = (cur_lon_rad - ref_lon_rad) .* (Rn + ref_h2) .* cos(ref_lat_rad);
pos_err_u = cur_h_i - ref_h2;

%% =========================
% 10. 速度误差
% ==========================
vel_err_n = cur_vn_i - ref_vn2;
vel_err_e = cur_ve_i - ref_ve2;
vel_err_u = cur_vu_i - ref_vu2;

pos_err_h  = sqrt(pos_err_n.^2 + pos_err_e.^2);
pos_err_3d = sqrt(pos_err_n.^2 + pos_err_e.^2 + pos_err_u.^2);

vel_err_h  = sqrt(vel_err_n.^2 + vel_err_e.^2);
vel_err_3d = sqrt(vel_err_n.^2 + vel_err_e.^2 + vel_err_u.^2);

%% =========================
% 11. 输出统计量
% ==========================
rmse = @(x) sqrt(mean(x.^2));

fprintf('\n=============== 位置误差统计（NEU, m） ===============\n');
fprintf('North      : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_n), rmse(pos_err_n), max(abs(pos_err_n)));
fprintf('East       : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_e), rmse(pos_err_e), max(abs(pos_err_e)));
fprintf('Up         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_u), rmse(pos_err_u), max(abs(pos_err_u)));
fprintf('Horizontal : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_h), rmse(pos_err_h), max(abs(pos_err_h)));
fprintf('3D         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_3d), rmse(pos_err_3d), max(abs(pos_err_3d)));

fprintf('\n=============== 速度误差统计（NVU, m/s） ===============\n');
fprintf('Vn         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_n), rmse(vel_err_n), max(abs(vel_err_n)));
fprintf('Ve         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_e), rmse(vel_err_e), max(abs(vel_err_e)));
fprintf('Vu         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_u), rmse(vel_err_u), max(abs(vel_err_u)));
fprintf('Horizontal : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_h), rmse(vel_err_h), max(abs(vel_err_h)));
fprintf('3D         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_3d), rmse(vel_err_3d), max(abs(vel_err_3d)));

fprintf('\n说明：本脚本时间对齐只使用 sow，不使用 GPS week；垂向速度比较采用 vu_current = -vd。\n');

%% =========================
% 12. 三轴位置对比图
% ==========================
figure('Name', 'Position Compare', 'Color', 'w');

subplot(3,1,1);
plot(t, ref_n, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_n, 'r', 'LineWidth', 1.2);
grid on;
ylabel('North (m)');
legend('Reference', 'Current');
title('Position Compare');

subplot(3,1,2);
plot(t, ref_e, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_e, 'r', 'LineWidth', 1.2);
grid on;
ylabel('East (m)');
legend('Reference', 'Current');

subplot(3,1,3);
plot(t, ref_u, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_u, 'r', 'LineWidth', 1.2);
grid on;
ylabel('Up (m)');
xlabel('sow (s)');
legend('Reference', 'Current');

%% =========================
% 13. 三轴速度对比图
% ==========================
figure('Name', 'Velocity Compare', 'Color', 'w');

subplot(3,1,1);
plot(t, ref_vn2, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_vn_i, 'r', 'LineWidth', 1.2);
grid on;
ylabel('Vn (m/s)');
legend('Reference', 'Current');
title('Velocity Compare');

subplot(3,1,2);
plot(t, ref_ve2, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_ve_i, 'r', 'LineWidth', 1.2);
grid on;
ylabel('Ve (m/s)');
legend('Reference', 'Current');

subplot(3,1,3);
plot(t, ref_vu2, 'b', 'LineWidth', 1.5); hold on;
plot(t, cur_vu_i, 'r', 'LineWidth', 1.2);
grid on;
ylabel('Vu (m/s)');
xlabel('sow (s)');
legend('Reference', 'Current');

%% =========================
% 14. 位置误差图
% ==========================
figure('Name', 'Position Error', 'Color', 'w');

subplot(4,1,1);
plot(t, pos_err_n, 'k', 'LineWidth', 1.2);
grid on;
ylabel('N err (m)');
title('Position Error');

subplot(4,1,2);
plot(t, pos_err_e, 'k', 'LineWidth', 1.2);
grid on;
ylabel('E err (m)');

subplot(4,1,3);
plot(t, pos_err_u, 'k', 'LineWidth', 1.2);
grid on;
ylabel('U err (m)');

subplot(4,1,4);
plot(t, pos_err_h, 'm', 'LineWidth', 1.2);
grid on;
ylabel('H err (m)');
xlabel('sow (s)');

%% =========================
% 15. 速度误差图
% ==========================
figure('Name', 'Velocity Error', 'Color', 'w');

subplot(4,1,1);
plot(t, vel_err_n, 'k', 'LineWidth', 1.2);
grid on;
ylabel('Vn err (m/s)');
title('Velocity Error');

subplot(4,1,2);
plot(t, vel_err_e, 'k', 'LineWidth', 1.2);
grid on;
ylabel('Ve err (m/s)');

subplot(4,1,3);
plot(t, vel_err_u, 'k', 'LineWidth', 1.2);
grid on;
ylabel('Vu err (m/s)');

subplot(4,1,4);
plot(t, vel_err_h, 'm', 'LineWidth', 1.2);
grid on;
ylabel('H err (m/s)');
xlabel('sow (s)');

%% =========================
% 16. 轨迹平面图
% ==========================
figure('Name', 'Trajectory Plane', 'Color', 'w');
plot(ref_e, ref_n, 'b', 'LineWidth', 1.6); hold on;
plot(cur_e, cur_n, 'r', 'LineWidth', 1.4);
plot(ref_e(1), ref_n(1), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(ref_e(end), ref_n(end), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
title('Trajectory Plane');
legend('Reference', 'Current', 'Start', 'End');

%% =========================
% 17. 排除大误差区域后的结果分析（可调阈值）
% ==========================
% 定义误差阈值（米），基于三维位置误差进行筛选
err_threshold = 5;  % 您可按需修改此值

% 生成有效数据掩码
good_mask = pos_err_3d < err_threshold;

% 检查是否有剩余点
if sum(good_mask) == 0
    warning('筛选后没有有效数据点，请增大阈值。');
else
    % 提取有效数据
    t_good       = t(good_mask);
    ref_n_good   = ref_n(good_mask);
    ref_e_good   = ref_e(good_mask);
    ref_u_good   = ref_u(good_mask);
    cur_n_good   = cur_n(good_mask);
    cur_e_good   = cur_e(good_mask);
    cur_u_good   = cur_u(good_mask);
    
    ref_vn_good  = ref_vn2(good_mask);
    ref_ve_good  = ref_ve2(good_mask);
    ref_vu_good  = ref_vu2(good_mask);
    cur_vn_good  = cur_vn_i(good_mask);
    cur_ve_good  = cur_ve_i(good_mask);
    cur_vu_good  = cur_vu_i(good_mask);
    
    pos_err_n_good = pos_err_n(good_mask);
    pos_err_e_good = pos_err_e(good_mask);
    pos_err_u_good = pos_err_u(good_mask);
    pos_err_h_good = pos_err_h(good_mask);
    pos_err_3d_good = pos_err_3d(good_mask);
    
    vel_err_n_good = vel_err_n(good_mask);
    vel_err_e_good = vel_err_e(good_mask);
    vel_err_u_good = vel_err_u(good_mask);
    vel_err_h_good = vel_err_h(good_mask);
    vel_err_3d_good = vel_err_3d(good_mask);
    
    % 输出过滤后的统计量
    rmse = @(x) sqrt(mean(x.^2));
    fprintf('\n======== 过滤后（3D位置误差 < %.1f m）位置误差统计 ========\n', err_threshold);
    fprintf('North      : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_n_good), rmse(pos_err_n_good), max(abs(pos_err_n_good)));
    fprintf('East       : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_e_good), rmse(pos_err_e_good), max(abs(pos_err_e_good)));
    fprintf('Up         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_u_good), rmse(pos_err_u_good), max(abs(pos_err_u_good)));
    fprintf('Horizontal : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_h_good), rmse(pos_err_h_good), max(abs(pos_err_h_good)));
    fprintf('3D         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(pos_err_3d_good), rmse(pos_err_3d_good), max(abs(pos_err_3d_good)));
    
    fprintf('\n======== 过滤后速度误差统计（NVU, m/s） ========\n');
    fprintf('Vn         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_n_good), rmse(vel_err_n_good), max(abs(vel_err_n_good)));
    fprintf('Ve         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_e_good), rmse(vel_err_e_good), max(abs(vel_err_e_good)));
    fprintf('Vu         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_u_good), rmse(vel_err_u_good), max(abs(vel_err_u_good)));
    fprintf('Horizontal : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_h_good), rmse(vel_err_h_good), max(abs(vel_err_h_good)));
    fprintf('3D         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(vel_err_3d_good), rmse(vel_err_3d_good), max(abs(vel_err_3d_good)));
    
    % ---- 过滤后的位置对比图 ----
    figure('Name', 'Position Compare (Filtered)', 'Color', 'w');
    subplot(3,1,1);
    plot(t_good, ref_n_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_n_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('North (m)'); legend('Reference', 'Current');
    title(sprintf('Position Compare (Filtered, 3D error < %.1f m)', err_threshold));
    
    subplot(3,1,2);
    plot(t_good, ref_e_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_e_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('East (m)'); legend('Reference', 'Current');
    
    subplot(3,1,3);
    plot(t_good, ref_u_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_u_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('Up (m)'); xlabel('sow (s)'); legend('Reference', 'Current');
    
    % ---- 过滤后的速度对比图 ----
    figure('Name', 'Velocity Compare (Filtered)', 'Color', 'w');
    subplot(3,1,1);
    plot(t_good, ref_vn_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_vn_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('Vn (m/s)'); legend('Reference', 'Current');
    title(sprintf('Velocity Compare (Filtered, 3D error < %.1f m)', err_threshold));
    
    subplot(3,1,2);
    plot(t_good, ref_ve_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_ve_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('Ve (m/s)'); legend('Reference', 'Current');
    
    subplot(3,1,3);
    plot(t_good, ref_vu_good, 'b', 'LineWidth', 1.5); hold on;
    plot(t_good, cur_vu_good, 'r', 'LineWidth', 1.2);
    grid on; ylabel('Vu (m/s)'); xlabel('sow (s)'); legend('Reference', 'Current');
    
    % ---- 过滤后的位置误差图 ----
    figure('Name', 'Position Error (Filtered)', 'Color', 'w');
    subplot(4,1,1);
    plot(t_good, pos_err_n_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('N err (m)'); title(sprintf('Position Error (Filtered, 3D error < %.1f m)', err_threshold));
    
    subplot(4,1,2);
    plot(t_good, pos_err_e_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('E err (m)');
    
    subplot(4,1,3);
    plot(t_good, pos_err_u_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('U err (m)');
    
    subplot(4,1,4);
    plot(t_good, pos_err_h_good, 'm', 'LineWidth', 1.2);
    grid on; ylabel('H err (m)'); xlabel('sow (s)');
    
    % ---- 过滤后的速度误差图 ----
    figure('Name', 'Velocity Error (Filtered)', 'Color', 'w');
    subplot(4,1,1);
    plot(t_good, vel_err_n_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('Vn err (m/s)'); title(sprintf('Velocity Error (Filtered, 3D error < %.1f m)', err_threshold));
    
    subplot(4,1,2);
    plot(t_good, vel_err_e_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('Ve err (m/s)');
    
    subplot(4,1,3);
    plot(t_good, vel_err_u_good, 'k', 'LineWidth', 1.2);
    grid on; ylabel('Vu err (m/s)');
    
    subplot(4,1,4);
    plot(t_good, vel_err_h_good, 'm', 'LineWidth', 1.2);
    grid on; ylabel('H err (m/s)'); xlabel('sow (s)');
    
    % ---- 过滤后的轨迹平面图 ----
    figure('Name', 'Trajectory Plane (Filtered)', 'Color', 'w');
    plot(ref_e_good, ref_n_good, 'b', 'LineWidth', 1.6); hold on;
    plot(cur_e_good, cur_n_good, 'r', 'LineWidth', 1.4);
    plot(ref_e_good(1), ref_n_good(1), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
    plot(ref_e_good(end), ref_n_good(end), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5);
    grid on; axis equal;
    xlabel('East (m)'); ylabel('North (m)');
    title(sprintf('Trajectory Plane (Filtered, 3D error < %.1f m)', err_threshold));
    legend('Reference', 'Current', 'Start', 'End');
    
    fprintf('\n过滤后剩余点数：%d（总点数 %d，剔除 %d 个）\n', sum(good_mask), length(t), sum(~good_mask));
end

%% =========================
% 18. 绘制姿态角（roll, pitch, yaw）——当前结果（三轴子图）
% ==========================
figure('Name', 'Attitude Angles (Overlay)', 'Color', 'w');
plot(t, cur_roll_i, 'r', 'LineWidth', 1.2); hold on;
plot(t, cur_pitch_i, 'g', 'LineWidth', 1.2);
plot(t, cur_yaw_i, 'b', 'LineWidth', 1.2);
grid on;
legend('Roll', 'Pitch', 'Yaw');
xlabel('sow (s)'); ylabel('Angle (deg)');
title('Attitude Angles from Current Result');

fprintf('\n姿态角绘制完成。\n');

%% =========================
% 19. 绘制陀螺零偏（三轴同一图）  <-- 新增
% ==========================
figure('Name', 'Gyroscope Bias', 'Color', 'w');
plot(t, cur_bgx_i, 'r', 'LineWidth', 1.2); hold on;
plot(t, cur_bgy_i, 'g', 'LineWidth', 1.2);
plot(t, cur_bgz_i, 'b', 'LineWidth', 1.2);
grid on;
legend('bgx', 'bgy', 'bgz');
xlabel('sow (s)'); ylabel('Gyro Bias (rad/s)');
title('Gyroscope Bias Estimates');

fprintf('陀螺零偏图绘制完成。\n');

%% =========================
% 20. 绘制加计零偏（三轴同一图）  <-- 新增
% ==========================
figure('Name', 'Accelerometer Bias', 'Color', 'w');
plot(t, cur_bax_i, 'r', 'LineWidth', 1.2); hold on;
plot(t, cur_bay_i, 'g', 'LineWidth', 1.2);
plot(t, cur_baz_i, 'b', 'LineWidth', 1.2);
grid on;
legend('bax', 'bay', 'baz');
xlabel('sow (s)'); ylabel('Accel Bias (m/s^2)');
title('Accelerometer Bias Estimates');

fprintf('加计零偏图绘制完成。\n');