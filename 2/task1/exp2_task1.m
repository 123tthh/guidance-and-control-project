function exp2_task1
% =========================================================================
% 实验二·任务 1：导弹俯仰角稳定回路控制器设计与性能分析
% -------------------------------------------------------------------------
%
% 固定命名：
%   vartheta_cmd : 俯仰角指令
%   vartheta     : 实际俯仰角
%   delta_z      : 舵偏输入
%   vartheta_dot : 俯仰角速度
%
% 注意：
%   1. 本脚本不使用 theta 表示俯仰角；
%   2. theta 在导弹运动学中通常保留给航迹倾角；
%   3. 本脚本采用与 Simulink 模型一致的微分滤波 s/(0.005s+1)。
%
% 用法：
%   >> exp2_task1
% =========================================================================

clc; close all;

%% 1. 导弹特征点参数
Kd   = 0.710;
Td   = 0.16;
T1d  = 1.508;
xid  = 0.084;
v    = 306; %#ok<NASGU>  % 本任务选俯仰角稳定回路，v 不直接进入 vartheta/delta_z 通道

%% 2. 控制器与指令参数
K_th = 3.0;       % 俯仰角比例控制增益
K_om = 0.20;      % 角速率反馈增益

vartheta_cmd = deg2rad(5);   % 与 task1_run.m / task1.slx 一致：5 deg 俯仰角阶跃指令

%% 3. 被控对象与微分滤波模块
% 导弹俯仰角通道：
% vartheta(s) / delta_z(s)
% = Kd*(T1d*s + 1) / [s*(Td^2*s^2 + 2*xid*Td*s + 1)]
num_vartheta = [Kd*T1d, Kd];
den_vartheta = [Td^2, 2*xid*Td, 1, 0];

% 微分滤波模块：
% vartheta_dot / vartheta = s / (0.005s + 1)
num_diff = [1 0];
den_diff = [0.005 1];

s = tf('s');
P = tf(num_vartheta, den_vartheta);     % vartheta / delta_z
D = tf(num_diff, den_diff);             % filtered derivative

%% 4. 开环弹体特性
wn_d = 1 / Td;
open_pair_real = -xid * wn_d;
open_pair_imag = wn_d * sqrt(1 - xid^2);
open_zero = -1 / T1d;

fprintf('弹体自然频率  wn_d = %.3f rad/s\n', wn_d);
fprintf('弹体开环极点  = 0.000  (积分极点)\n');
fprintf('弹体短周期极点 = %+.3f +/- %.3fj  (xid=%.3f, 极轻阻尼)\n', ...
    open_pair_real, open_pair_imag, xid);
fprintf('弹体开环零点  = %+.3f\n', open_zero);

figure('Color','w','Name','开环弹体分析','Position',[80 80 1000 380]);
subplot(1,2,1);
step(P, 8);
grid on;
title('开环弹体阶跃响应（积分性、振荡）');
xlabel('时间 / s');
ylabel('幅值');

subplot(1,2,2);
bode(P);
grid on;
title('开环弹体 Bode 图');

%% 5. 内环：角速率反馈
% 控制律内环部分：
% delta_z = u_outer - K_om * vartheta_dot
% vartheta_dot = D(s) * vartheta
%
% 因此内环闭环对象：
% H_in(s) = vartheta(s) / u_outer(s)
%         = P(s) / [1 + K_om*D(s)*P(s)]
H_in = feedback(P, K_om * D);

fprintf('\n内环 K_om = %.2f\n', K_om);
fprintf('内环闭环极点:\n');
disp(pole(H_in));

inner_poles = pole(H_in);
cpx_inner = inner_poles(abs(imag(inner_poles)) > 1e-3);
if ~isempty(cpx_inner)
    p = cpx_inner(1);
    wn_in = abs(p);
    zeta_in = -real(p) / wn_in;
    fprintf('   主导复极点: wn = %.3f rad/s, zeta = %.3f\n', wn_in, zeta_in);
end

%% 6. 外环：俯仰角比例反馈
% 外环控制律：
% delta_z = K_th*(vartheta_cmd - vartheta) - K_om*vartheta_dot
%
% 外环闭环：
% T_cl(s) = vartheta(s) / vartheta_cmd(s)
%         = K_th*H_in(s) / [1 + K_th*H_in(s)]
L_ol = K_th * H_in;
T_cl = feedback(L_ol, 1);

fprintf('\n外环 K_th = %.2f\n', K_th);
fprintf('闭环极点:\n');
disp(pole(T_cl));

fprintf('闭环静态增益 dcgain(T_cl) = %.6f\n', dcgain(T_cl));

%% 7. 闭环响应、舵偏输入和俯仰角速度
% 解析得到：
% vartheta     = T_cl * vartheta_cmd
% vartheta_dot = D * T_cl * vartheta_cmd
% delta_z      = K_th*(vartheta_cmd - vartheta) - K_om*vartheta_dot
%
% 即从 vartheta_cmd 到 delta_z 的传递函数：
T_vartheta     = T_cl;
T_vartheta_dot = minreal(D * T_cl);
T_delta_z      = minreal(K_th * (1 - T_cl) - K_om * D * T_cl);

t = 0:0.001:6;

[vartheta_out, t]     = step(vartheta_cmd * T_vartheta, t);
vartheta_dot_out      = step(vartheta_cmd * T_vartheta_dot, t);
delta_z_out           = step(vartheta_cmd * T_delta_z, t);
vartheta_cmd_out      = vartheta_cmd * ones(size(t));

figure('Color','w','Name','俯仰角稳定回路响应','Position',[80 80 760 480]);
plot(t, rad2deg(vartheta_cmd_out), '--', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(vartheta_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('俯仰角 / deg');
legend('vartheta\_cmd', 'vartheta', 'Location', 'best');
title('俯仰角稳定回路响应');
xlim([0 6]);
ylim([0 6]);

figure('Color','w','Name','舵偏输入','Position',[100 100 760 420]);
plot(t, rad2deg(delta_z_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('delta\_z / deg');
title('舵偏输入 delta\_z');
xlim([0 6]);

figure('Color','w','Name','俯仰角速度','Position',[120 120 760 420]);
plot(t, rad2deg(vartheta_dot_out), 'LineWidth', 1.5);
grid on;
xlabel('时间 / s');
ylabel('vartheta\_dot / deg/s');
title('俯仰角速度 vartheta\_dot');
xlim([0 6]);

%% 8. 补偿前后对比与稳定裕度
T_no_comp = feedback(P, 1);   % 无内环、无 K_th 放大，仅单位姿态反馈对比

figure('Color','w','Name','补偿前后对比与稳定裕度','Position',[80 80 1100 430]);

subplot(1,2,1);
step(T_cl, 15); hold on;
step(T_no_comp, 15);
grid on;
legend({'两环反馈：K\_om=0.20, K\_th=3.00', ...
        '无角速率反馈：K\_om=0, K\_th=1'}, ...
        'Location','best');
title('单位阶跃响应对比');
xlabel('时间 / s');
ylabel('幅值');

subplot(1,2,2);
margin(L_ol);
grid on;
title('外环开环 Bode 与稳定裕度');

%% 9. 闭环零极点图
figure('Color','w','Name','闭环零极点');
pzmap(T_cl);
grid on;
title('闭环零极点');

%% 10. 性能指标
info = stepinfo(vartheta_out, t, vartheta_cmd);
[Gm, Pm, Wcg, Wcp] = margin(L_ol);

fprintf('\n=== 控制器性能（vartheta_cmd = 5 deg）===\n');
fprintf('  上升时间 t_r          = %.4f s\n', info.RiseTime);
fprintf('  调节时间 t_s          = %.4f s\n', info.SettlingTime);
fprintf('  超调量 Mp             = %.4f %%\n', info.Overshoot);
fprintf('  峰值 Peak             = %.4f deg\n', rad2deg(info.Peak));
fprintf('  峰值时间 PeakTime     = %.4f s\n', info.PeakTime);
fprintf('  理论稳态值            = %.4f deg\n', rad2deg(dcgain(T_cl) * vartheta_cmd));
fprintf('  6 s 时实际值           = %.4f deg\n', rad2deg(vartheta_out(end)));
fprintf('  6 s 时误差             = %.6f deg\n', rad2deg(abs(vartheta_cmd - vartheta_out(end))));
fprintf('  最大舵偏输入 max(delta_z)        = %.4f deg\n', rad2deg(max(delta_z_out)));
fprintf('  最小舵偏输入 min(delta_z)        = %.4f deg\n', rad2deg(min(delta_z_out)));
fprintf('  最大俯仰角速度 max(vartheta_dot) = %.4f deg/s\n', rad2deg(max(vartheta_dot_out)));

if isinf(Gm)
    fprintf('  幅值裕度 GM            = Inf dB  (omega = Inf rad/s)\n');
else
    fprintf('  幅值裕度 GM            = %.2f dB  (omega = %.4f rad/s)\n', 20*log10(Gm), Wcg);
end
fprintf('  相位裕度 PM            = %.2f deg (穿越 omega_c = %.4f rad/s)\n', Pm, Wcp);

end
