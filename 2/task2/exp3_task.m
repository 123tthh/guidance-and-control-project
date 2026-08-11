function OUT = exp3_task(action, methodName)
%EXP3_TASK 实验二任务二：三种导引律 MATLAB 仿真与 Simulink 初始化。
%
% 入口：
%   exp3_task
%   P = exp3_task('params')
%   S = exp3_task('init','三点法')
%   S = exp3_task('init','追踪法')
%   S = exp3_task('init','比例引导法')
%
% Simulink 统一参数向量 par 的索引：
%   par(1)  = v        导弹速度 / (m/s)
%   par(2)  = vT       目标速度 / (m/s)
%   par(3)  = r0       目标初始距离 / m
%   par(4)  = q0       目标初始视线角 / rad
%   par(5)  = xT0      目标初始 x 坐标 / m
%   par(6)  = yT0      目标初始 y 坐标 / m
%   par(7)  = HIT      命中半径 / m
%   par(8)  = K        比例导引系数
%   par(9)  = sigmaT   目标速度方向角 / rad
%   par(10) = sigmaM0  导弹初始速度方向角 / rad
%   par(11) = g0       标准重力加速度 / (m/s^2)
%   par(12) = tmax     最大仿真时间 / s

if nargin < 1 || isempty(action)
    action = 'run';
end
if nargin < 2
    methodName = '';
end

action = char(action);

switch lower(action)
    case {'params','param'}
        OUT = default_params();

    case {'init','slxinit'}
        if isempty(methodName)
            methodName = '三点法';
        end
        P = default_params();
        [x0, canonicalName] = initial_state(P, methodName);
        par = params_to_vector(P);

        assignin('base', 'par', par);
        assignin('base', 'x0', x0);
        assignin('base', 'param_struct', P);
        assignin('base', 'current_guidance_method', canonicalName);
        safe_assignin('base', '参数', par);
        safe_assignin('base', '初值', x0);

        S = struct();
        S.params = P;
        S.paramVec = par;
        S.x0 = x0;
        S.methodName = canonicalName;
        OUT = S;

        fprintf('Simulink 初始化完成：%s\n', canonicalName);
        fprintf('  状态 x=[x_M; y_M; σ; λ]\n');
        fprintf('  参数向量已写入 base workspace：par；若支持中文，也写入“参数”\n');
        fprintf('  初始状态已写入 base workspace：x0；若支持中文，也写入“初值”\n');

    case {'run','simulate','matlab'}
        OUT = run_matlab_simulation();

    otherwise
        error('exp3_task:UnknownAction', '未知入口：%s', action);
end
end

% ========================================================================
% 默认参数
% ========================================================================
function P = default_params()
P = struct();
P.v       = 1000.0;           % 导弹速度：1 km/s
P.vT      = 500.0;            % 目标速度：500 m/s
P.r0      = 10000.0;          % 目标初始距离：10 km
P.q0      = 30*pi/180;        % 目标初始视线角：30 deg
P.xT0     = P.r0*cos(P.q0);
P.yT0     = P.r0*sin(P.q0);
P.HIT     = 5.0;              % 命中半径：5 m
P.K       = 4.0;              % 比例导引系数
P.sigmaT  = pi;               % 目标水平向左匀速飞行
P.sigmaM0 = P.q0;             % 导弹初始速度方向指向目标初始位置
P.g0      = 9.80665;
P.tmax    = 60.0;
end

function par = params_to_vector(P)
par = [P.v; P.vT; P.r0; P.q0; P.xT0; P.yT0; P.HIT; P.K; ...
       P.sigmaT; P.sigmaM0; P.g0; P.tmax];
end

function P = vector_to_params(par)
P = struct();
P.v       = par(1);
P.vT      = par(2);
P.r0      = par(3);
P.q0      = par(4);
P.xT0     = par(5);
P.yT0     = par(6);
P.HIT     = par(7);
P.K       = par(8);
P.sigmaT  = par(9);
P.sigmaM0 = par(10);
P.g0      = par(11);
P.tmax    = par(12);
end %#ok<DEFNU>

function [x0, canonicalName] = initial_state(P, methodName)
canonicalName = normalize_method_name(methodName);
switch canonicalName
    case '三点法'
        x0 = [0; 0; P.sigmaM0; 0];
    case '追踪法'
        x0 = [0; 0; P.sigmaM0; 0];
    case '比例导引法 K=4'
        x0 = [0; 0; P.sigmaM0; 0];
    otherwise
        error('exp3_task:UnknownMethod', '未知制导方式：%s', char(methodName));
end
end

function name = normalize_method_name(methodName)
s = char(methodName);
if contains(s, '三点') || strcmpi(s, 'three_point')
    name = '三点法';
elseif contains(s, '追踪') || contains(s, '追跡') || strcmpi(s, 'pursuit')
    name = '追踪法';
elseif contains(s, '比例') || contains(lower(s), 'pn')
    name = '比例导引法 K=4';
else
    name = s;
end
end

function safe_assignin(ws, name, value)
try
    assignin(ws, name, value);
catch
    % 某些 MATLAB 版本或代码生成路径对中文变量名不稳定，忽略中文变量写入失败。
end
end

% ========================================================================
% MATLAB 版本仿真
% ========================================================================
function OUT = run_matlab_simulation()
P = default_params();
methods = {'三点法', '追踪法', '比例引导法 K=4'};
R = cell(numel(methods), 1);

outDir = fullfile(pwd, 'exp2_task2_outputs_matlab');
if exist(outDir, 'dir') ~= 7
    mkdir(outDir);
end

for k = 1:numel(methods)
    R{k} = simulate_method(methods{k}, P);
    write_method_csv(R{k}, outDir);
end
write_summary_csv(R, outDir);
write_matlab_figures(R, outDir);

OUT = struct('params', P, 'results', {R}, 'outDir', outDir);
fprintf('MATLAB 仿真完成。输出目录：%s\n', outDir);
end

function R = simulate_method(methodName, P)
[x0, canonicalName] = initial_state(P, methodName);
opts = odeset('RelTol',1e-7, 'AbsTol',1e-8, 'MaxStep',0.05, ...
              'Events', @(t,x) hit_event(t, x, canonicalName, P));
[t, X] = ode45(@(t,x) guidance_ode(t, x, canonicalName, P), [0 P.tmax], x0, opts);

Y = zeros(numel(t), 9);
for i = 1:numel(t)
    Y(i,:) = guidance_output(t(i), X(i,:).', canonicalName, P).';
end

R = struct();
R.method_name = canonicalName;
if strcmp(canonicalName, '三点法')
    R.method_id = 'three_point'; R.csv = 'three_point';
elseif strcmp(canonicalName, '追踪法')
    R.method_id = 'pursuit'; R.csv = 'pursuit';
else
    R.method_id = 'pn_K4'; R.csv = 'pn_K4';
end
R.t = t(:);
R.x = X;
R.y = Y;
R.x_M_m = Y(:,1);
R.y_M_m = Y(:,2);
R.sigma_rad = unwrap(Y(:,3));
R.lambda_m = X(:,4);
R.x_T_m = Y(:,4);
R.y_T_m = Y(:,5);
R.r_m = Y(:,6);
R.q_rad = unwrap(Y(:,7));
R.qdot_rad_s = Y(:,8);
R.a_n_m_s2 = Y(:,9);
R.sigma_deg = R.sigma_rad*180/pi;
R.q_deg = R.q_rad*180/pi;
R.qdot_deg_s = R.qdot_rad_s*180/pi;
R.n_g = R.a_n_m_s2/P.g0;
R.hit = R.r_m(end) <= P.HIT + 1e-6;
end

function dx = guidance_ode(t, x, methodName, P)
sig = x(3);
lam = max(x(4), 0);

if strcmp(methodName, '三点法')
    [xT, yT, vxT, vyT] = target_state(t, P);
    qT = atan2(yT, xT);
    qTdot = (xT*vyT - yT*vxT) / max(xT^2 + yT^2, 1e-9);
    lamDot = sqrt(max(P.v^2 - (lam*qTdot)^2, 0));
    dxM = lamDot*cos(qT) - lam*qTdot*sin(qT);
    dyM = lamDot*sin(qT) + lam*qTdot*cos(qT);
    sigDot = qTdot;
    dx = [dxM; dyM; sigDot; lamDot];
else
    [~, qdot] = relative_geometry(t, x, P);
    if strcmp(methodName, '追踪法')
        sigDot = qdot;
    else
        sigDot = P.K*qdot;
    end
    dx = [P.v*cos(sig); P.v*sin(sig); sigDot; 0];
end
end

function y = guidance_output(t, x, methodName, P)
[xT, yT, vxT, vyT] = target_state(t, P);
xM = x(1); yM = x(2); sig = x(3);
rx = xT - xM;
ry = yT - yM;
r = sqrt(rx^2 + ry^2);
q = atan2(ry, rx);
vmx = P.v*cos(sig);
vmy = P.v*sin(sig);
qdot = (rx*(vyT-vmy) - ry*(vxT-vmx)) / max(r^2, 1e-9);

if strcmp(methodName, '三点法')
    qTdot = (xT*vyT - yT*vxT) / max(xT^2 + yT^2, 1e-9);
    a_n = P.v*qTdot;
elseif strcmp(methodName, '追踪法')
    a_n = P.v*qdot;
else
    a_n = P.v*P.K*qdot;
end

y = [xM; yM; sig; xT; yT; r; q; qdot; a_n];
end

function [value, isterminal, direction] = hit_event(t, x, methodName, P)
% 停止于弹目最近点（脱靶量点）：接近率 dr/dt 由负变正过零。
% 不用 r<=HIT：5 m 命中窗太窄，粗步长会跨过最近点，导致三点法永不停机、轨迹发散，
% 比例导引停在假的时刻。最近点判据与求解器步长无关，三种律都能干净命中。
[xT, yT, vxT, vyT] = target_state(t, P);
dx = guidance_ode(t, x, methodName, P);   % 弹速分量（三点法为几何弹速，其余为 v·[cosσ;sinσ]）
rx = xT - x(1);
ry = yT - x(2);
r  = sqrt(rx^2 + ry^2);
value = (rx*(vxT - dx(1)) + ry*(vyT - dx(2))) / max(r, 1e-9);  % dr/dt
isterminal = 1;
direction = 1;   % 由负(接近)变正(远离) = 最近点
end

function [xT, yT, vxT, vyT] = target_state(t, P)
vxT = P.vT*cos(P.sigmaT);
vyT = P.vT*sin(P.sigmaT);
xT = P.xT0 + vxT*t;
yT = P.yT0 + vyT*t;
end

function [q, qdot] = relative_geometry(t, x, P)
[xT, yT, vxT, vyT] = target_state(t, P);
rx = xT - x(1);
ry = yT - x(2);
q = atan2(ry, rx);
r2 = max(rx^2 + ry^2, 1e-9);
vmx = P.v*cos(x(3));
vmy = P.v*sin(x(3));
qdot = (rx*(vyT-vmy) - ry*(vxT-vmx)) / r2;
end

% ========================================================================
% CSV 与图形
% ========================================================================
function write_method_csv(R, outDir)
time_s = R.t;
x_M_m = R.x_M_m;
y_M_m = R.y_M_m;
sigma_rad = R.sigma_rad;
sigma_deg = R.sigma_deg;
lambda_m = R.lambda_m;
x_T_m = R.x_T_m;
y_T_m = R.y_T_m;
r_m = R.r_m;
q_rad = R.q_rad;
q_deg = R.q_deg;
qdot_rad_s = R.qdot_rad_s;
qdot_deg_s = R.qdot_deg_s;
a_n_m_s2 = R.a_n_m_s2;
n_g = R.n_g;
T = table(time_s, x_M_m, y_M_m, sigma_rad, sigma_deg, lambda_m, ...
    x_T_m, y_T_m, r_m, q_rad, q_deg, qdot_rad_s, qdot_deg_s, a_n_m_s2, n_g);
writetable(T, fullfile(outDir, ['exp2_task2_matlab_' R.csv '.csv']));
end

function write_summary_csv(R, outDir)
N = numel(R);
method_id = cell(N, 1);
method_name = cell(N, 1);
t_end_s = zeros(N, 1);
hit = false(N, 1);
miss_m = zeros(N, 1);
peak_abs_a_n_g = zeros(N, 1);
terminal_a_n_g = zeros(N, 1);
terminal_qdot_deg_s = zeros(N, 1);
for i = 1:N
    method_id{i} = R{i}.method_id;
    method_name{i} = R{i}.method_name;
    t_end_s(i) = R{i}.t(end);
    hit(i) = R{i}.hit;
    miss_m(i) = R{i}.r_m(end);
    peak_abs_a_n_g(i) = max(abs(R{i}.n_g));
    terminal_a_n_g(i) = R{i}.n_g(end);
    terminal_qdot_deg_s(i) = R{i}.qdot_deg_s(end);
end
T = table(method_id, method_name, t_end_s, hit, miss_m, ...
    peak_abs_a_n_g, terminal_a_n_g, terminal_qdot_deg_s);
writetable(T, fullfile(outDir, 'exp2_task2_summary_matlab.csv'));
end

function write_matlab_figures(R, outDir)
figure('Name','三种制导律弹道对比','Color','w'); hold on; grid on; axis equal;
for i = 1:numel(R)
    plot(R{i}.x_M_m/1000, R{i}.y_M_m/1000, 'LineWidth', 1.8);
end
plot(R{1}.x_T_m/1000, R{1}.y_T_m/1000, 'k--', 'LineWidth', 1.2);
xlabel('x / km'); ylabel('y / km');
title('三种制导律弹道对比');
legend({'三点法导弹','追踪法导弹','比例引导法 K=4 导弹','目标'}, 'Location','best');
saveas(gcf, fullfile(outDir, '弹道对比.png'));

figure('Name','弹目距离随时间变化','Color','w'); hold on; grid on;
for i = 1:numel(R)
    plot(R{i}.t, R{i}.r_m, 'LineWidth', 1.8);
end
xlabel('时间 / s'); ylabel('弹目距离 r / m');
title('弹目距离随时间变化');
legend({'三点法','追踪法','比例引导法 K=4'}, 'Location','best');
saveas(gcf, fullfile(outDir, '弹目距离.png'));

figure('Name','需用法向过载','Color','w'); hold on; grid on;
for i = 1:numel(R)
    plot(R{i}.t, R{i}.n_g, 'LineWidth', 1.8);
end
xlabel('时间 / s'); ylabel('需用法向过载 n / g');
title('需用法向过载对比');
legend({'三点法','追踪法','比例引导法 K=4'}, 'Location','best');
saveas(gcf, fullfile(outDir, '法向过载.png'));
end
