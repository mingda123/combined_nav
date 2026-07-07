%% 将组合参考结果与rtk结果比较
clc;
clear;
close all;

%% =========================
% 1. 文件路径：只改这里
% ==========================
file_ref1 = 'D:\my_project\combined_nav\assets2\ref.pos';
file_ref2 = 'D:\my_project\combined_nav\assets2\gnss_20260602_110011_682828.pos';

%% =========================
% 2. 读取两份参考值文件
%    默认前两行为表头
% ==========================
raw1 = readmatrix(file_ref1, 'FileType', 'text', 'NumHeaderLines', 2);
raw2 = readmatrix(file_ref2, 'FileType', 'text', 'NumHeaderLines', 2);

raw1 = raw1(~all(isnan(raw1), 2), :);
raw2 = raw2(~all(isnan(raw2), 2), :);

if size(raw1, 2) < 8
    error('file_ref1 列数不足，至少需要前 8 列。');
end
if size(raw2, 2) < 8
    error('file_ref2 列数不足，至少需要前 8 列。');
end

%% =========================
% 3. 提取字段
%    这里只按 sow 对齐，不使用 GPS week
% ==========================
sow1 = raw1(:, 2);
lat1 = raw1(:, 3);
lon1 = raw1(:, 4);
h1   = raw1(:, 5);
vn1  = raw1(:, 6);
ve1  = raw1(:, 7);
vu1  = raw1(:, 8);

sow2 = raw2(:, 2);
lat2 = raw2(:, 3);
lon2 = raw2(:, 4);
h2   = raw2(:, 5);
vn2  = raw2(:, 6);
ve2  = raw2(:, 7);
vu2  = raw2(:, 8);

%% =========================
% 4. 排序并去除重复 sow
% ==========================
[sow1, idx1] = sort(sow1);
lat1 = lat1(idx1);
lon1 = lon1(idx1);
h1   = h1(idx1);
vn1  = vn1(idx1);
ve1  = ve1(idx1);
vu1  = vu1(idx1);

[sow2, idx2] = sort(sow2);
lat2 = lat2(idx2);
lon2 = lon2(idx2);
h2   = h2(idx2);
vn2  = vn2(idx2);
ve2  = ve2(idx2);
vu2  = vu2(idx2);

[sow1, ia1] = unique(sow1, 'stable');
lat1 = lat1(ia1);
lon1 = lon1(ia1);
h1   = h1(ia1);
vn1  = vn1(ia1);
ve1  = ve1(ia1);
vu1  = vu1(ia1);

[sow2, ia2] = unique(sow2, 'stable');
lat2 = lat2(ia2);
lon2 = lon2(ia2);
h2   = h2(ia2);
vn2  = vn2(ia2);
ve2  = ve2(ia2);
vu2  = vu2(ia2);

%% =========================
% 5. 取重叠时间段
% ==========================
start_sow = max(min(sow1), min(sow2));
end_sow   = min(max(sow1), max(sow2));

if end_sow <= start_sow
    error('两份数据在 sow 上没有重叠时间段。');
end

mask1 = (sow1 >= start_sow) & (sow1 <= end_sow);
mask2 = (sow2 >= start_sow) & (sow2 <= end_sow);

sow1c = sow1(mask1);
lat1c = lat1(mask1);
lon1c = lon1(mask1);
h1c   = h1(mask1);
vn1c  = vn1(mask1);
ve1c  = ve1(mask1);
vu1c  = vu1(mask1);

sow2c = sow2(mask2);
lat2c = lat2(mask2);
lon2c = lon2(mask2);
h2c   = h2(mask2);
vn2c  = vn2(mask2);
ve2c  = ve2(mask2);
vu2c  = vu2(mask2);

%% =========================
% 6. 将第二份数据插值到第一份数据时刻
% ==========================
t_all = sow1c;
valid_t = (t_all >= sow2c(1)) & (t_all <= sow2c(end));

t    = t_all(valid_t);
lat1c = lat1c(valid_t);
lon1c = lon1c(valid_t);
h1c   = h1c(valid_t);
vn1c  = vn1c(valid_t);
ve1c  = ve1c(valid_t);
vu1c  = vu1c(valid_t);

lat2i = interp1(sow2c, lat2c, t, 'linear');
lon2i = interp1(sow2c, lon2c, t, 'linear');
h2i   = interp1(sow2c, h2c,   t, 'linear');
vn2i  = interp1(sow2c, vn2c,  t, 'linear');
ve2i  = interp1(sow2c, ve2c,  t, 'linear');
vu2i  = interp1(sow2c, vu2c,  t, 'linear');

valid = isfinite(lat2i) & isfinite(lon2i) & isfinite(h2i) & ...
        isfinite(vn2i)  & isfinite(ve2i)  & isfinite(vu2i);

t    = t(valid);
lat1c = lat1c(valid);
lon1c = lon1c(valid);
h1c   = h1c(valid);
vn1c  = vn1c(valid);
ve1c  = ve1c(valid);
vu1c  = vu1c(valid);

lat2i = lat2i(valid);
lon2i = lon2i(valid);
h2i   = h2i(valid);
vn2i  = vn2i(valid);
ve2i  = ve2i(valid);
vu2i  = vu2i(valid);

%% =========================
% 7. 建立统一局部坐标系
%    用第一份轨迹起点作为原点
% ==========================
a  = 6378137.0;
f  = 1 / 298.257223563;
e2 = f * (2 - f);

lat0_deg = lat1c(1);
lon0_deg = lon1c(1);
h0       = h1c(1);

lat0 = deg2rad(lat0_deg);
lon0 = deg2rad(lon0_deg);

sin_lat0 = sin(lat0);
den0 = sqrt(1 - e2 * sin_lat0^2);
Rm0 = a * (1 - e2) / den0^3;
Rn0 = a / den0;

lat1_rad = deg2rad(lat1c);
lon1_rad = deg2rad(lon1c);
lat2_rad = deg2rad(lat2i);
lon2_rad = deg2rad(lon2i);

n1 = (lat1_rad - lat0) .* (Rm0 + h0);
e1 = (lon1_rad - lon0) .* (Rn0 + h0) .* cos(lat0);
u1 = h1c - h0;

n2 = (lat2_rad - lat0) .* (Rm0 + h0);
e2p = (lon2_rad - lon0) .* (Rn0 + h0) .* cos(lat0);
u2 = h2i - h0;

%% =========================
% 8. 逐时刻计算相对第一份数据的 NEU 差异
% ==========================
sin_lat_ref = sin(lat1_rad);
den = sqrt(1 - e2 .* sin_lat_ref.^2);
Rm = a * (1 - e2) ./ (den.^3);
Rn = a ./ den;

diff_n = (lat2_rad - lat1_rad) .* (Rm + h1c);
diff_e = (lon2_rad - lon1_rad) .* (Rn + h1c) .* cos(lat1_rad);
diff_u = h2i - h1c;

diff_h  = sqrt(diff_n.^2 + diff_e.^2);
diff_3d = sqrt(diff_n.^2 + diff_e.^2 + diff_u.^2);

%% =========================
% 9. 速度三轴差异
% ==========================
diff_vn = vn2i - vn1c;
diff_ve = ve2i - ve1c;
diff_vu = vu2i - vu1c;

%% =========================
% 10. 统计输出
% ==========================
rmse = @(x) sqrt(mean(x.^2));

fprintf('\n=============== 三轴位置差异（NEU, m） ===============\n');
fprintf('North      : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_n), rmse(diff_n), max(abs(diff_n)));
fprintf('East       : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_e), rmse(diff_e), max(abs(diff_e)));
fprintf('Up         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_u), rmse(diff_u), max(abs(diff_u)));
fprintf('Horizontal : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_h), rmse(diff_h), max(abs(diff_h)));
fprintf('3D         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_3d), rmse(diff_3d), max(abs(diff_3d)));

fprintf('\n=============== 三轴速度差异（m/s） ===============\n');
fprintf('Vn         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_vn), rmse(diff_vn), max(abs(diff_vn)));
fprintf('Ve         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_ve), rmse(diff_ve), max(abs(diff_ve)));
fprintf('Vu         : mean = % .4f, rmse = %.4f, max_abs = %.4f\n', mean(diff_vu), rmse(diff_vu), max(abs(diff_vu)));

%% =========================
% 11. 平面轨迹图
% ==========================
figure('Name', 'Plane Trajectory', 'Color', 'w');
plot(e1, n1, 'b', 'LineWidth', 1.5); hold on;
plot(e2p, n2, 'r', 'LineWidth', 1.2);
plot(e1(1), n1(1), 'go', 'MarkerSize', 8, 'LineWidth', 1.5);
plot(e1(end), n1(end), 'ks', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on;
axis equal;
xlabel('East (m)');
ylabel('North (m)');
title('Plane Trajectory');
legend('Data 1', 'Data 2', 'Start', 'End');

%% =========================
% 12. 三轴位置差异图
% ==========================
figure('Name', 'NEU Difference', 'Color', 'w');

subplot(4,1,1);
plot(t, diff_n, 'b', 'LineWidth', 1.2);
grid on;
ylabel('North (m)');
title('NEU Difference');

subplot(4,1,2);
plot(t, diff_e, 'r', 'LineWidth', 1.2);
grid on;
ylabel('East (m)');

subplot(4,1,3);
plot(t, diff_u, 'k', 'LineWidth', 1.2);
grid on;
ylabel('Up (m)');

subplot(4,1,4);
plot(t, diff_h, 'm', 'LineWidth', 1.2);
grid on;
ylabel('Horizontal (m)');
xlabel('sow (s)');

%% =========================
% 13. 三轴速度差异图
% ==========================
figure('Name', 'Velocity Difference', 'Color', 'w');

subplot(3,1,1);
plot(t, diff_vn, 'b', 'LineWidth', 1.2);
grid on;
ylabel('Vn (m/s)');
title('Velocity Difference');

subplot(3,1,2);
plot(t, diff_ve, 'r', 'LineWidth', 1.2);
grid on;
ylabel('Ve (m/s)');

subplot(3,1,3);
plot(t, diff_vu, 'k', 'LineWidth', 1.2);
grid on;
ylabel('Vu (m/s)');
xlabel('sow (s)');