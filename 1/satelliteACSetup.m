function varargout = satelliteACSetup(verbose)
%SATELLITEACSETUP  三轴稳定卫星姿态控制实验参数初始化。
%
% 直接运行：
%   satelliteACSetup
%
% 由建模函数调用：
%   par = satelliteACSetup(false)

if nargin < 1
    verbose = true;
end

par.Re = 6378e3;
par.mu = 3.986e14;
par.H  = 450e3;
par.a  = par.Re + par.H;

par.w0 = sqrt(par.mu/par.a^3);
par.Torb = 2*pi/par.w0;

par.J = diag([3.2e2, 1.06e3, 1.3e3]);

par.Cw = [1 0 0 1/sqrt(3);
          0 1 0 1/sqrt(3);
          0 0 1 1/sqrt(3)];

par.Cwp = par.Cw'/(par.Cw*par.Cw');

par.Tmax = 0.05;
par.Td = zeros(3,1);

zeta = 0.90;
wn = 0.020;
pint = wn/8;

Iax = diag(par.J);
par.Kp  = Iax.*(wn^2 + 2*zeta*wn*pint);
par.Kd  = Iax.*(2*zeta*wn + pint);
par.Ki  = Iax.*(wn^2*pint);
par.Kaw = sqrt(par.Ki./par.Kd);

phi0   = deg2rad(10);
theta0 = deg2rad(5);
psi0   = deg2rad(-4);

Cbo0 = attitude312(phi0, theta0, psi0);
wbo0 = zeros(3,1);
wbody0 = Cbo0*[0; -par.w0; 0] + wbo0;

hw0 = zeros(4,1);

par.x0 = [phi0; theta0; psi0; wbody0; hw0];
par.z0 = zeros(3,1);
par.r0 = zeros(3,1);
par.Tsim = 800;

exportToBase(par);

if verbose
    fprintf('轨道角速度 w0 = %.4e rad/s，轨道周期 = %.1f s\n', par.w0, par.Torb);
    fprintf('比例增益 Kp = [%g %g %g]\n', par.Kp);
    fprintf('微分（阻尼）增益 Kd = [%g %g %g]\n', par.Kd);
    fprintf('积分增益 Ki = [%g %g %g]\n', par.Ki);
    fprintf('抗饱和增益 Kaw = [%g %g %g]\n', par.Kaw);
end

if nargout > 0
    varargout{1} = par;
end
end

function exportToBase(par)
assignin('base','Re',par.Re);
assignin('base','mu',par.mu);
assignin('base','H',par.H);
assignin('base','a',par.a);
assignin('base','w0',par.w0);
assignin('base','Torb',par.Torb);

assignin('base','J',par.J);
assignin('base','Cw',par.Cw);
assignin('base','Cwp',par.Cwp);
assignin('base','Tmax',par.Tmax);
assignin('base','Td',par.Td);

assignin('base','Kp',par.Kp(:));
assignin('base','Kd',par.Kd(:));
assignin('base','Ki',par.Ki(:));
assignin('base','Kaw',par.Kaw(:));

assignin('base','x0',par.x0);
assignin('base','z0',par.z0);
assignin('base','r0',par.r0);
assignin('base','Tsim',par.Tsim);
end

function Cbo = attitude312(phi, theta, psi)
cphi = cos(phi);
sphi = sin(phi);
cth  = cos(theta);
sth  = sin(theta);
cps  = cos(psi);
sps  = sin(psi);

Rx = [1 0 0; 0 cphi sphi; 0 -sphi cphi];
Ry = [cth 0 -sth; 0 1 0; sth 0 cth];
Rz = [cps sps 0; -sps cps 0; 0 0 1];

Cbo = Ry*Rx*Rz;
end
