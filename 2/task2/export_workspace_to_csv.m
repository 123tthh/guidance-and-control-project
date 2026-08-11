function OUT = export_workspace_to_csv(methodName, varargin)
%EXPORT_WORKSPACE_TO_CSV 适配新版 build_slx.m 的手动运行数据导出脚本。
%
% 用法：
%   export_workspace_to_csv
%   export_workspace_to_csv('三点法')
%   export_workspace_to_csv('追踪法')
%   export_workspace_to_csv('比例导引法 K=4')
%
% 本函数不运行 Simulink，不调用 build_slx，不搜索模型子文件夹。
% 它只读取当前 MATLAB base workspace 中的 yLog/xLog，或 Single simulation output
% 模式下 out.yLog / out.get('yLog') 中的数据。
%
% 新版 build_slx.m 约定：
%   yLog = y=[x_M,y_M,sigma,x_T,y_T,r,q,qdot,a_n]，9列
%   xLog = x=[x_M;y_M;sigma;lambda]，4列
%   exp3_task('init',方法名) 会写入 par、x0、current_guidance_method
%
% 输出目录：
%   pwd/exp2_task2_outputs_manual

opts = parse_options(varargin{:});

if nargin < 1 || isempty(methodName)
    methodName = infer_method_from_workspace();
end

method = normalize_method_name(methodName);

if opts.checkMethod
    check_workspace_method_consistency(method);
end

projectDir = pwd;
if isempty(opts.outDir)
    outDir = fullfile(projectDir, 'exp2_task2_outputs_manual');
else
    outDir = opts.outDir;
    if ~isabsolute_path(outDir)
        outDir = fullfile(projectDir, outDir);
    end
end

ensure_dir(outDir);
ensure_dir(fullfile(outDir, method.dir));

[yTime, Y] = read_workspace_record({'yLog','simout','输出记录'});
if isempty(Y)
    error('export_workspace_to_csv:MissingYLog', ...
        ['base workspace 中没有找到 yLog，也没有在 out.yLog / out.get(''yLog'') 中找到数据。\n' ...
         '请先手动运行当前 .slx 模型。新版 build_slx.m 的 To Workspace 变量名应为 yLog。\n' ...
         'yLog 应为 y=[x_M,y_M,sigma,x_T,y_T,r,q,qdot,a_n]。']);
end

Y = normalize_matrix_shape(Y, yTime, 9, 'yLog');

[xTime, X] = read_workspace_record({'xLog','stateLog','状态记录'});
if ~isempty(X)
    X = normalize_matrix_shape(X, xTime, 4, 'xLog');
end

if isempty(yTime)
    if ~isempty(xTime)
        yTime = xTime;
    else
        yTime = (0:size(Y,1)-1).';
    end
end

yTime = yTime(:);
if numel(yTime) ~= size(Y,1)
    if numel(yTime) >= 2
        yTime = linspace(yTime(1), yTime(end), size(Y,1)).';
    else
        yTime = (0:size(Y,1)-1).';
    end
end

P = read_params_from_workspace();
R = postprocess_record(yTime, Y, X, method, P);

[methodCsv, methodCsvInDir] = write_method_csv(R, outDir);

rebuild_summary_and_tacview(outDir, method, opts);

methodSummary = fullfile(outDir, ['exp2_task2_summary_slx_' method.csv '.csv']);
methodTacview = fullfile(outDir, ['exp2_task2_tacview_long_' method.csv '.csv']);
allSummary = fullfile(outDir, 'exp2_task2_summary_slx_all.csv');
allTacview = fullfile(outDir, 'exp2_task2_tacview_long_all.csv');

if exist(allTacview, 'file') == 2
    tacviewForAcmi = allTacview;
else
    tacviewForAcmi = methodTacview;
end

OUT = struct();
OUT.method = method;
OUT.methodCsv = methodCsv;
OUT.methodCsvInDir = methodCsvInDir;
OUT.methodSummary = methodSummary;
OUT.methodTacviewCsv = methodTacview;
OUT.allSummary = allSummary;
OUT.allTacviewCsv = allTacview;
OUT.tacviewCsvForAcmi = tacviewForAcmi;
OUT.outDir = outDir;
OUT.rows = numel(R.t);
OUT.tEnd = R.t(end);
OUT.minMiss = R.min_miss_m;
OUT.terminalMiss = R.terminal_miss_m;
OUT.hit = R.hit;

fprintf('\n已从 workspace 导出：%s\n', methodCsv);
fprintf('已同步复制到方法目录：%s\n', methodCsvInDir);
fprintf('当前方法 summary：%s\n', methodSummary);
fprintf('当前方法 Tacview CSV：%s\n', methodTacview);
fprintf('终止时间 %.6f s，最小弹目距离 %.6f m，末端弹目距离 %.6f m，最大 |n| %.6f g\n', ...
    OUT.tEnd, OUT.minMiss, OUT.terminalMiss, max(abs(R.n_g)));

if exist(allSummary, 'file') == 2
    fprintf('合并 summary：%s\n', allSummary);
end
if exist(allTacview, 'file') == 2
    fprintf('合并 Tacview CSV：%s\n', allTacview);
end

fprintf('下一步可执行：python build_acmi_exp2.py --csv "%s" --out exp2_intercept.acmi\n', tacviewForAcmi);
end

% ========================================================================
% 参数解析
% ========================================================================
function opts = parse_options(varargin)
opts = struct();
opts.outDir = '';
opts.checkMethod = true;
opts.parallelSpacing = 30000;

if mod(numel(varargin), 2) ~= 0
    error('export_workspace_to_csv:BadOptions', '可选参数必须为 name/value 成对输入。');
end

for i = 1:2:numel(varargin)
    key = lower(char(varargin{i}));
    val = varargin{i+1};

    switch key
        case {'outdir','outputdir'}
            opts.outDir = char(val);

        case {'checkmethod','check_method'}
            opts.checkMethod = logical(val);

        case {'parallelspacing','parallel_spacing','spacing'}
            opts.parallelSpacing = double(val);

        otherwise
            error('export_workspace_to_csv:UnknownOption', '未知选项：%s', key);
    end
end
end

function tf = isabsolute_path(p)
if ispc
    tf = numel(p) >= 2 && p(2) == ':';
else
    tf = ~isempty(p) && p(1) == '/';
end
end

function ensure_dir(p)
if exist(p, 'dir') ~= 7
    mkdir(p);
end
end

% ========================================================================
% 方法判断与防错
% ========================================================================
function method = infer_method_from_workspace()
method = '';

try
    method = evalin('base', 'current_guidance_method');
catch
end

if isempty(method)
    try
        mdl = bdroot;
        if contains(mdl, 'three_point')
            method = '三点法';
        elseif contains(mdl, 'pursuit')
            method = '追踪法';
        elseif contains(mdl, 'pn')
            method = '比例导引法 K=4';
        end
    catch
    end
end

if isempty(method)
    error('export_workspace_to_csv:NeedMethod', ...
        ['无法从 base workspace 自动判断制导方法。\n' ...
         '请使用 export_workspace_to_csv(''三点法'')、export_workspace_to_csv(''追踪法'') 或 export_workspace_to_csv(''比例导引法 K=4'')。']);
end
end

function check_workspace_method_consistency(method)
workspaceMethod = '';

try
    workspaceMethod = evalin('base', 'current_guidance_method');
catch
end

if isempty(workspaceMethod)
    return;
end

workspaceMethodNorm = normalize_method_name(workspaceMethod);

if ~strcmp(workspaceMethodNorm.id, method.id)
    error('export_workspace_to_csv:MethodMismatch', ...
        ['当前 workspace 数据来自：%s\n' ...
         '但你正在导出为：%s\n\n' ...
         '请先执行：clear yLog xLog rLog qLog qdotLog anLog out\n' ...
         '然后重新手动运行对应的 .slx 模型，再导出。'], ...
         workspaceMethodNorm.name, method.name);
end
end

% ========================================================================
% 工作区读取
% ========================================================================
function [time, values] = read_workspace_record(names)
time = [];
values = [];

for i = 1:numel(names)
    name = names{i};

    [ok, rec] = read_one_workspace_object(name);
    if ~ok
        continue;
    end

    [time, values] = unpack_record(rec);
    if ~isempty(values)
        return;
    end
end
end

function [ok, rec] = read_one_workspace_object(name)
ok = false;
rec = [];

try
    exists = evalin('base', sprintf('exist(''%s'',''var'')', name));
catch
    exists = 0;
end

if exists == 1
    try
        rec = evalin('base', name);
        ok = true;
        return;
    catch
    end
end

try
    hasOut = evalin('base', 'exist(''out'',''var'')');
catch
    hasOut = 0;
end

if hasOut == 1
    try
        simOut = evalin('base', 'out');

        try
            rec = simOut.get(name);
            if ~isempty(rec)
                ok = true;
                return;
            end
        catch
        end

        try
            rec = simOut.(name);
            if ~isempty(rec)
                ok = true;
                return;
            end
        catch
        end
    catch
    end
end
end

function [time, values] = unpack_record(rec)
time = [];
values = [];

if isa(rec, 'timeseries')
    time = rec.Time(:);
    values = rec.Data;

elseif isa(rec, 'Simulink.SimulationData.Dataset')
    if rec.numElements < 1
        return;
    end
    elem = rec.getElement(1);
    [time, values] = unpack_record(elem.Values);
    return;

elseif isa(rec, 'Simulink.SimulationData.Signal')
    [time, values] = unpack_record(rec.Values);
    return;

elseif isstruct(rec) && isfield(rec, 'Values')
    [time, values] = unpack_record(rec.Values);
    return;

elseif isstruct(rec) && isfield(rec, 'time') && isfield(rec, 'signals')
    time = rec.time(:);
    values = rec.signals.values;

elseif isnumeric(rec)
    values = rec;
    time = (0:size(values,1)-1).';

else
    return;
end

values = squeeze(values);

if isempty(values)
    return;
end

if isvector(values)
    values = values(:);
end
end

function values = normalize_matrix_shape(values, time, expectedCols, varName)
values = squeeze(values);

if isempty(values)
    return;
end

if isvector(values)
    values = values(:);
end

if ~isempty(time)
    nt = numel(time);
    if size(values, 1) ~= nt && size(values, 2) == nt
        values = values.';
    end
end

if size(values, 2) ~= expectedCols && size(values, 1) == expectedCols
    values = values.';
end

if size(values, 2) ~= expectedCols
    error('export_workspace_to_csv:BadDim', ...
        '%s 维度不符合要求：期望 %d 列，当前 size=[%d,%d]。', ...
        varName, expectedCols, size(values,1), size(values,2));
end
end

function P = read_params_from_workspace()
P = struct();
P.v = 1000;
P.vT = 500;
P.r0 = 10000;
P.q0 = 30*pi/180;
P.xT0 = P.r0*cos(P.q0);
P.yT0 = P.r0*sin(P.q0);
P.HIT = 5;
P.K = 4;
P.sigmaT = pi;
P.sigmaM0 = P.q0;
P.g0 = 9.80665;
P.tmax = 60;

try
    ps = evalin('base', 'param_struct');
    fields = fieldnames(P);
    for i = 1:numel(fields)
        if isfield(ps, fields{i})
            P.(fields{i}) = ps.(fields{i});
        end
    end
catch
end

try
    par = evalin('base', 'par');
    if numel(par) >= 12
        P.v = par(1);
        P.vT = par(2);
        P.r0 = par(3);
        P.q0 = par(4);
        P.xT0 = par(5);
        P.yT0 = par(6);
        P.HIT = par(7);
        P.K = par(8);
        P.sigmaT = par(9);
        P.sigmaM0 = par(10);
        P.g0 = par(11);
        P.tmax = par(12);
    end
catch
end
end

% ========================================================================
% 后处理
% ========================================================================
function R = postprocess_record(time, Y, X, method, P)
R = struct();
R.method_name = method.name;
R.method_id = method.id;
R.csv_tag = method.csv;
R.t = time(:);

R.x_M_m = Y(:,1);
R.y_M_m = Y(:,2);
R.sigma_rad = unwrap(Y(:,3));
R.x_T_m = Y(:,4);
R.y_T_m = Y(:,5);
R.r_m = Y(:,6);
R.q_rad = unwrap(Y(:,7));
R.qdot_rad_s = Y(:,8);
R.a_n_m_s2 = Y(:,9);

if ~isempty(X) && size(X,2) >= 4 && size(X,1) == size(Y,1)
    R.lambda_m = X(:,4);
elseif strcmp(method.id, 'three_point')
    R.lambda_m = hypot(R.x_M_m, R.y_M_m);
else
    R.lambda_m = zeros(size(R.t));
end

R.sigma_deg = R.sigma_rad * 180/pi;
R.q_deg = R.q_rad * 180/pi;
R.qdot_deg_s = R.qdot_rad_s * 180/pi;
R.n_g = R.a_n_m_s2 / P.g0;

[R.min_miss_m, idx] = min(R.r_m);
R.min_miss_time_s = R.t(idx);
R.terminal_miss_m = R.r_m(end);
R.hit = R.min_miss_m <= P.HIT + 1e-6;
R.P = P;
end

function [filePath, filePathInDir] = write_method_csv(R, outDir)
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

fileName = ['exp2_task2_slx_' R.csv_tag '.csv'];

filePath = fullfile(outDir, fileName);
writetable(T, filePath);

methodDir = fullfile(outDir, method_dir_from_csv(R.csv_tag));
ensure_dir(methodDir);

filePathInDir = fullfile(methodDir, fileName);
writetable(T, filePathInDir);
end

% ========================================================================
% Summary 与 Tacview long CSV
% ========================================================================
function rebuild_summary_and_tacview(outDir, currentMethod, opts)
methods = all_methods();

singleFile = find_method_csv(outDir, currentMethod);
if ~isempty(singleFile)
    T = readtable(singleFile);
    Rsingle = {struct('meta', currentMethod, 'T', T)};
    write_summary_csv(Rsingle, outDir);
    write_tacview_long_csv(Rsingle, outDir, opts);
    copy_single_outputs_to_method_dir(outDir, currentMethod);
end

R = {};
for i = 1:numel(methods)
    p = find_method_csv(outDir, methods{i});
    if ~isempty(p)
        T = readtable(p);
        R{end+1} = struct('meta', methods{i}, 'T', T); %#ok<AGROW>
    end
end

if numel(R) >= 2
    write_summary_csv(R, outDir);
    write_tacview_long_csv(R, outDir, opts);
end
end

function p = find_method_csv(outDir, meta)
p = '';

flat = fullfile(outDir, ['exp2_task2_slx_' meta.csv '.csv']);
if exist(flat, 'file') == 2
    p = flat;
    return;
end

inside = fullfile(outDir, meta.dir, ['exp2_task2_slx_' meta.csv '.csv']);
if exist(inside, 'file') == 2
    p = inside;
end
end

function copy_single_outputs_to_method_dir(outDir, meta)
methodDir = fullfile(outDir, meta.dir);
ensure_dir(methodDir);

files = { ...
    ['exp2_task2_summary_slx_' meta.csv '.csv'], ...
    ['exp2_task2_tacview_long_' meta.csv '.csv']};

for i = 1:numel(files)
    src = fullfile(outDir, files{i});
    dst = fullfile(methodDir, files{i});
    if exist(src, 'file') == 2
        copyfile(src, dst);
    end
end
end

function write_summary_csv(R, outDir)
N = numel(R);

method_id = cell(N,1);
method_name = cell(N,1);
t_end_s = zeros(N,1);
hit = false(N,1);
miss_m = zeros(N,1);
terminal_miss_m = zeros(N,1);
t_miss_min_s = zeros(N,1);
peak_abs_a_n_g = zeros(N,1);
terminal_a_n_g = zeros(N,1);
terminal_qdot_deg_s = zeros(N,1);

for i = 1:N
    T = R{i}.T;
    meta = R{i}.meta;

    method_id{i} = meta.id;
    method_name{i} = meta.name;
    t_end_s(i) = T.time_s(end);

    [miss_m(i), idx] = min(T.r_m);
    t_miss_min_s(i) = T.time_s(idx);
    terminal_miss_m(i) = T.r_m(end);
    hit(i) = miss_m(i) <= 5.0 + 1e-6;

    peak_abs_a_n_g(i) = max(abs(T.n_g));
    terminal_a_n_g(i) = T.n_g(end);
    terminal_qdot_deg_s(i) = T.qdot_deg_s(end);
end

S = table(method_id, method_name, t_end_s, hit, miss_m, terminal_miss_m, ...
    t_miss_min_s, peak_abs_a_n_g, terminal_a_n_g, terminal_qdot_deg_s);

if numel(R) == 1
    fileName = ['exp2_task2_summary_slx_' R{1}.meta.csv '.csv'];
else
    fileName = 'exp2_task2_summary_slx_all.csv';
end

writetable(S, fullfile(outDir, fileName));
end

function write_tacview_long_csv(R, outDir, opts)
method_id = {};
method_name = {};
object_id = {};
object_type = {};
time_s = [];
x_m = [];
y_m = [];
z_m = [];
heading_deg = [];
pitch_deg = [];
roll_deg = [];
speed_mps = [];
r_m = [];
q_deg = [];
qdot_deg_s = [];
a_n_m_s2 = [];
n_g = [];

for i = 1:numel(R)
    T = R{i}.T;
    meta = R{i}.meta;
    n = height(T);

    if numel(R) >= 2
        northOffset = method_north_offset(meta, opts.parallelSpacing);
    else
        northOffset = 0;
    end

    method_id{end+1,1} = meta.id; %#ok<AGROW>
    method_name{end+1,1} = meta.name; %#ok<AGROW>
    object_id{end+1,1} = ['station_' meta.id]; %#ok<AGROW>
    object_type{end+1,1} = 'station'; %#ok<AGROW>
    time_s(end+1,1) = 0; %#ok<AGROW>
    x_m(end+1,1) = 0; %#ok<AGROW>
    y_m(end+1,1) = northOffset; %#ok<AGROW>
    z_m(end+1,1) = 0; %#ok<AGROW>
    heading_deg(end+1,1) = 0; %#ok<AGROW>
    pitch_deg(end+1,1) = 0; %#ok<AGROW>
    roll_deg(end+1,1) = 0; %#ok<AGROW>
    speed_mps(end+1,1) = 0; %#ok<AGROW>
    r_m(end+1,1) = 0; %#ok<AGROW>
    q_deg(end+1,1) = 0; %#ok<AGROW>
    qdot_deg_s(end+1,1) = 0; %#ok<AGROW>
    a_n_m_s2(end+1,1) = 0; %#ok<AGROW>
    n_g(end+1,1) = 0; %#ok<AGROW>

    hM = heading_from_xy(T.x_M_m, T.y_M_m, T.sigma_deg);
    pM = pitch_from_xy(T.x_M_m, T.y_M_m, T.sigma_deg);
    hT = heading_from_xy(T.x_T_m, T.y_T_m, 180*ones(n,1));
    pT = pitch_from_xy(T.x_T_m, T.y_T_m, zeros(n,1));

    method_id = [method_id; repmat({meta.id}, n, 1)]; %#ok<AGROW>
    method_name = [method_name; repmat({meta.name}, n, 1)]; %#ok<AGROW>
    object_id = [object_id; repmat({['missile_' meta.id]}, n, 1)]; %#ok<AGROW>
    object_type = [object_type; repmat({'missile'}, n, 1)]; %#ok<AGROW>
    time_s = [time_s; T.time_s]; %#ok<AGROW>
    x_m = [x_m; T.x_M_m]; %#ok<AGROW>
    y_m = [y_m; northOffset*ones(n,1)]; %#ok<AGROW>
    z_m = [z_m; T.y_M_m]; %#ok<AGROW>
    heading_deg = [heading_deg; hM]; %#ok<AGROW>
    pitch_deg = [pitch_deg; pM]; %#ok<AGROW>
    roll_deg = [roll_deg; zeros(n,1)]; %#ok<AGROW>
    speed_mps = [speed_mps; 1000*ones(n,1)]; %#ok<AGROW>
    r_m = [r_m; T.r_m]; %#ok<AGROW>
    q_deg = [q_deg; T.q_deg]; %#ok<AGROW>
    qdot_deg_s = [qdot_deg_s; T.qdot_deg_s]; %#ok<AGROW>
    a_n_m_s2 = [a_n_m_s2; T.a_n_m_s2]; %#ok<AGROW>
    n_g = [n_g; T.n_g]; %#ok<AGROW>

    method_id = [method_id; repmat({meta.id}, n, 1)]; %#ok<AGROW>
    method_name = [method_name; repmat({meta.name}, n, 1)]; %#ok<AGROW>
    object_id = [object_id; repmat({['target_' meta.id]}, n, 1)]; %#ok<AGROW>
    object_type = [object_type; repmat({'target'}, n, 1)]; %#ok<AGROW>
    time_s = [time_s; T.time_s]; %#ok<AGROW>
    x_m = [x_m; T.x_T_m]; %#ok<AGROW>
    y_m = [y_m; northOffset*ones(n,1)]; %#ok<AGROW>
    z_m = [z_m; T.y_T_m]; %#ok<AGROW>
    heading_deg = [heading_deg; hT]; %#ok<AGROW>
    pitch_deg = [pitch_deg; pT]; %#ok<AGROW>
    roll_deg = [roll_deg; zeros(n,1)]; %#ok<AGROW>
    speed_mps = [speed_mps; 500*ones(n,1)]; %#ok<AGROW>
    r_m = [r_m; zeros(n,1)]; %#ok<AGROW>
    q_deg = [q_deg; NaN(n,1)]; %#ok<AGROW>
    qdot_deg_s = [qdot_deg_s; NaN(n,1)]; %#ok<AGROW>
    a_n_m_s2 = [a_n_m_s2; zeros(n,1)]; %#ok<AGROW>
    n_g = [n_g; zeros(n,1)]; %#ok<AGROW>
end

L = table(method_id, method_name, object_id, object_type, time_s, x_m, y_m, z_m, ...
    heading_deg, pitch_deg, roll_deg, speed_mps, r_m, q_deg, qdot_deg_s, a_n_m_s2, n_g);

if numel(R) == 1
    fileName = ['exp2_task2_tacview_long_' R{1}.meta.csv '.csv'];
else
    fileName = 'exp2_task2_tacview_long_all.csv';
end

writetable(L, fullfile(outDir, fileName));
end

function h = heading_from_xy(x, y, fallback)
if nargin < 3 || isempty(fallback)
    fallback = zeros(size(x));
end

x = x(:);
y = y(:);
fallback = fallback(:);

if numel(x) >= 2
    dx = gradient(x);
    dy = gradient(y);
    h = atan2d(dx, dy);
    h = mod(h, 360);
else
    h = mod(fallback, 360);
end
end

function p = pitch_from_xy(x, y, fallbackSigmaDeg)
x = x(:);
y = y(:);

if nargin < 3 || isempty(fallbackSigmaDeg)
    fallbackSigmaDeg = zeros(size(x));
end

if numel(x) >= 2
    dx = gradient(x);
    dy = gradient(y);
    p = atan2d(dy, max(abs(dx), 1e-9));
else
    p = fallbackSigmaDeg(:);
end
end

% ========================================================================
% 方法元数据
% ========================================================================
function methods = all_methods()
methods = { ...
    struct('id','three_point','name','三点法','csv','three_point','dir','三点法'), ...
    struct('id','pursuit','name','追踪法','csv','pursuit','dir','追踪法'), ...
    struct('id','pn_K4','name','比例导引法 K=4','csv','pn_K4','dir','比例引导法')};
end

function method = normalize_method_name(methodName)
s = char(methodName);
sl = lower(s);

if contains(s, '三点') || strcmpi(s, 'three_point') || contains(sl, 'three')
    method = struct('name','三点法','id','three_point','csv','three_point','dir','三点法');
elseif contains(s, '追踪') || contains(s, '追跡') || strcmpi(s, 'pursuit') || contains(sl, 'pursuit')
    method = struct('name','追踪法','id','pursuit','csv','pursuit','dir','追踪法');
elseif contains(s, '比例') || contains(sl, 'pn') || contains(sl, 'proportional')
    method = struct('name','比例导引法 K=4','id','pn_K4','csv','pn_K4','dir','比例引导法');
else
    error('export_workspace_to_csv:UnknownMethod', '未知制导方法：%s', s);
end
end

function d = method_dir_from_csv(csvTag)
switch csvTag
    case 'three_point'
        d = '三点法';
    case 'pursuit'
        d = '追踪法';
    case 'pn_K4'
        d = '比例引导法';
    otherwise
        d = csvTag;
end
end

function north = method_north_offset(meta, spacing)
switch meta.id
    case 'three_point'
        north = -spacing;
    case 'pursuit'
        north = 0;
    case 'pn_K4'
        north = spacing;
    otherwise
        north = 0;
end
end