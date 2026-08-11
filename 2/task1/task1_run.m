%% task1_run.m
% 实验2 Task1：俯仰角稳定回路前置运行代码
% 作用：
%   1. 定义 task1.slx 所需工作区参数
%   2. 运行 task1.slx
%   3. 读取 out.simout
%   4. 绘制 vartheta_cmd、vartheta、delta_z、vartheta_dot
%
% 固定变量名：
%   vartheta_cmd  : 俯仰角指令
%   vartheta      : 实际俯仰角
%   delta_z       : 舵偏输入
%   vartheta_dot  : 俯仰角速度

clear; clc; close all;

%% 1. 导弹特征点参数
Kd   = 0.710;
Td   = 0.16;
T1d  = 1.508;
xid  = 0.084;

%% 2. 控制器参数
K_th = 3.0;     % 俯仰角比例控制增益
K_om = 0.20;    % 角速率反馈增益

%% 3. 俯仰角指令
vartheta_cmd = deg2rad(5);   % 5 deg 阶跃指令

%% 4. 传递函数参数
% 导弹俯仰通道：
% vartheta(s) / delta_z(s)
% = Kd*(T1d*s + 1) / [s*(Td^2*s^2 + 2*xid*Td*s + 1)]
num_vartheta = [Kd*T1d, Kd];
den_vartheta = [Td^2, 2*xid*Td, 1, 0];

% 微分滤波模块：
% vartheta_dot / vartheta = s / (0.005s + 1)
num_diff = [1 0];
den_diff = [0.005 1];

%% 5. 检查模型是否存在
model = 'task1';

if ~isfile([model '.slx'])
    error('未找到 task1.slx。请先运行 build_task1_slx.m 生成模型。');
end

%% 6. 运行仿真
out = sim(model);

%% 7. 读取仿真数据
t = out.tout;
data = out.simout;

% simout 列顺序固定：
% 1: vartheta_cmd
% 2: vartheta
% 3: delta_z
% 4: vartheta_dot
vartheta_cmd_out = data(:,1);
vartheta_out     = data(:,2);
delta_z_out      = data(:,3);
vartheta_dot_out = data(:,4);

%% 8. 绘图：俯仰角响应
figure;
plot(t, rad2deg(vartheta_cmd_out), '--', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(vartheta_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('俯仰角 / deg');
legend('vartheta\_cmd', 'vartheta', 'Location', 'best');
title('俯仰角稳定回路响应');
xlim([0 6]);
ylim([0 6]);

%% 9. 绘图：舵偏输入
figure('Name','舵偏输入');
plot(t, rad2deg(delta_z_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('delta\_z / deg');
title('舵偏输入 delta\_z');

%% 10. 绘图：俯仰角速度
figure('Name','俯仰角速度');
plot(t, rad2deg(vartheta_dot_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('vartheta\_dot / deg/s');
title('俯仰角速度 vartheta\_dot');

%% 11. 性能指标
info = stepinfo(vartheta_out, t, vartheta_cmd);

fprintf('\n===== 实验2 Task1：俯仰角稳定回路性能指标 =====\n');
fprintf('俯仰角指令 vartheta_cmd = %.3f deg\n', rad2deg(vartheta_cmd));
fprintf('K_th = %.3f\n', K_th);
fprintf('K_om = %.3f\n', K_om);
fprintf('上升时间 RiseTime       = %.4f s\n', info.RiseTime);
fprintf('调节时间 SettlingTime   = %.4f s\n', info.SettlingTime);
fprintf('超调量 Overshoot        = %.4f %%\n', info.Overshoot);
fprintf('峰值 Peak               = %.4f deg\n', rad2deg(info.Peak));
fprintf('稳态值 Final Value      = %.4f deg\n', rad2deg(vartheta_out(end)));
fprintf('稳态误差 ess            = %.6f deg\n', rad2deg(abs(vartheta_cmd - vartheta_out(end))));
fprintf('=================================================\n');
