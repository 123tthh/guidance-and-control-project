function satelliteACPlot(saveFigures)
% satelliteACPlot
% 实验一：三轴稳定卫星姿态控制仿真结果绘图程序
%
% 使用方法：
%   1) 将本文件放在 satelliteACModel.slx 和 satelliteACSetup.m 同一目录；
%   2) MATLAB 命令行运行：
%          satelliteACSetup
%          out = sim('satelliteACModel');
%          satelliteACPlot
%
% 也可以直接运行：
%          satelliteACPlot
%      若 base 工作区没有 out，本程序会自动调用 satelliteACSetup 并运行模型。
%
% 输出图像：
%   图1 三轴姿态角响应
%   图2 四飞轮力矩输出
%   图3 卫星本体实际控制力矩
%   图4 相对轨道系角速度响应
%
% 若 saveFigures = true，则自动保存 png 图片。
% 例如：
%          satelliteACPlot(true)

if nargin < 1
    saveFigures = true;
end

modelName = 'satelliteACModel';

% ===== 1. 获取仿真输出 out =====
if evalin('base', 'exist(''out'', ''var'')')
    out = evalin('base', 'out');
else
    if exist('satelliteACSetup.m', 'file') == 2
        satelliteACSetup;
    else
        warning('未找到 satelliteACSetup.m，请确认参数已在 base 工作区初始化。');
    end

    if exist([modelName '.slx'], 'file') ~= 2
        error('未找到模型文件 satelliteACModel.slx。请将本脚本放在模型所在目录。');
    end

    out = sim(modelName);
    assignin('base', 'out', out);
end

% ===== 2. 提取日志数据 =====
[tAtt, attitudeRad] = readLog(out, 'attitudeLog', 3);
[tWheel, wheelTorque] = readLog(out, 'wheelTorqueLog', 4);
[tActual, actualTorque] = readLog(out, 'actualTorqueLog', 3);
[tRate, rateBO] = readLog(out, 'rateLog', 3);

attitudeDeg = rad2deg(attitudeRad);

% ===== 3. 绘制三轴姿态角响应 =====
figure('Name', '三轴姿态角响应', 'Color', 'w');
plot(tAtt, attitudeDeg, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('姿态角 / deg');
legend('\phi 滚动角', '\theta 俯仰角', '\psi 偏航角', 'Location', 'northeast');
title('三轴姿态角响应');
set(gca, 'FontName', 'Microsoft YaHei', 'FontSize', 11);
if saveFigures
    exportgraphics(gcf, '三轴姿态角响应.png', 'Resolution', 300);
end

% ===== 4. 绘制四飞轮力矩输出 =====
figure('Name', '四飞轮力矩输出', 'Color', 'w');
plot(tWheel, wheelTorque, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('飞轮力矩 / N·m');
legend('飞轮一', '飞轮二', '飞轮三', '飞轮四', 'Location', 'northeast');
title('四飞轮力矩输出');
set(gca, 'FontName', 'Microsoft YaHei', 'FontSize', 11);
if saveFigures
    exportgraphics(gcf, '四飞轮力矩输出.png', 'Resolution', 300);
end

% ===== 5. 绘制卫星本体实际控制力矩 =====
figure('Name', '卫星本体实际控制力矩', 'Color', 'w');
plot(tActual, actualTorque, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('实际控制力矩 / N·m');
legend('u_x', 'u_y', 'u_z', 'Location', 'northeast');
title('卫星本体实际控制力矩');
set(gca, 'FontName', 'Microsoft YaHei', 'FontSize', 11);
if saveFigures
    exportgraphics(gcf, '卫星本体实际控制力矩.png', 'Resolution', 300);
end

% ===== 6. 绘制相对轨道系角速度响应 =====
figure('Name', '相对轨道系角速度响应', 'Color', 'w');
plot(tRate, rateBO, 'LineWidth', 1.2);
grid on;
xlabel('时间 / s');
ylabel('相对角速度 / rad/s');
legend('\omega_x', '\omega_y', '\omega_z', 'Location', 'northeast');
title('相对轨道系角速度响应');
set(gca, 'FontName', 'Microsoft YaHei', 'FontSize', 11);
if saveFigures
    exportgraphics(gcf, '相对轨道系角速度响应.png', 'Resolution', 300);
end

% ===== 7. 打印简单检查结果 =====
fprintf('\n===== 仿真结果检查 =====\n');
fprintf('最大姿态角绝对值 / deg:  phi %.4f, theta %.4f, psi %.4f\n', ...
    max(abs(attitudeDeg(:,1))), max(abs(attitudeDeg(:,2))), max(abs(attitudeDeg(:,3))));
fprintf('终端姿态角 / deg:        phi %.6f, theta %.6f, psi %.6f\n', ...
    attitudeDeg(end,1), attitudeDeg(end,2), attitudeDeg(end,3));
fprintf('最大飞轮力矩绝对值 / N·m: %.6f\n', max(abs(wheelTorque), [], 'all'));
fprintf('最大实际控制力矩绝对值 / N·m: %.6f\n', max(abs(actualTorque), [], 'all'));
fprintf('终端相对角速度范数 / rad/s: %.6e\n', norm(rateBO(end,:)));
fprintf('========================\n');

if saveFigures
    fprintf('已保存图片：\n');
    fprintf('  三轴姿态角响应.png\n');
    fprintf('  四飞轮力矩输出.png\n');
    fprintf('  卫星本体实际控制力矩.png\n');
    fprintf('  相对轨道系角速度响应.png\n');
end

end


function [t, y] = readLog(out, varName, nChannel)
% readLog
% 从 Simulink.SimulationOutput 或 base 工作区中读取 To Workspace 结构体。
%
% 支持常见保存形式：
%   1) out.attitudeLog
%   2) base 工作区 attitudeLog
%   3) Structure With Time: s.time, s.signals.values
%   4) Timeseries: s.Time, s.Data
%
% 输出：
%   t : N×1
%   y : N×nChannel

% 优先从 SimulationOutput 中读取
if isa(out, 'Simulink.SimulationOutput') && isprop(out, varName)
    s = out.(varName);
elseif evalin('base', sprintf('exist(''%s'', ''var'')', varName))
    s = evalin('base', varName);
else
    error('没有找到日志变量 %s。请检查 To Workspace 块的 Variable name。', varName);
end

% Structure With Time
if isstruct(s) && isfield(s, 'time') && isfield(s, 'signals')
    t = s.time(:);
    y = squeeze(s.signals.values);

% timeseries
elseif isa(s, 'timeseries')
    t = s.Time(:);
    y = squeeze(s.Data);

else
    error('日志变量 %s 的格式不支持。建议 To Workspace 选择 Structure With Time。', varName);
end

% 处理一维和三维向量信号
if isvector(y)
    y = y(:);
end

% Simulink 常见格式：通道数 × 1 × 时间点数，squeeze 后为 通道数 × 时间点数
if size(y, 2) == length(t) && size(y, 1) ~= length(t)
    y = y.';
end

% 如果仍不是 N×通道，尝试 reshape
if size(y, 1) ~= length(t)
    if numel(y) == length(t) * nChannel
        y = reshape(y, [nChannel, length(t)]).';
    else
        error('日志变量 %s 的维度无法转换。当前 size(y) = [%s], length(t) = %d。', ...
            varName, num2str(size(y)), length(t));
    end
end

% 截取或检查通道数
if size(y, 2) < nChannel
    error('日志变量 %s 的通道数不足。需要 %d 通道，实际 %d 通道。', ...
        varName, nChannel, size(y,2));
elseif size(y, 2) > nChannel
    y = y(:, 1:nChannel);
end

end
