% A PID Control Demo for GQUAV
% Each module is designed independently, allowing you to add your own custom code.
% You can edit the code or parameters noted with "***"
clear,clc
exprname = 'MPC_demo';
%% 参数设置
% *** 仿真持续时间(s) ***
T = 30;
% *** 控制频率(Hz) ***
global freq_ctr %#ok<*GVMIS>
freq_ctr = 20;
%% 仿真准备
% 导入核心库
addpath(genpath('../Core'));
addpath(genpath('MPCSim_Core'));
% *** 启动仿真环境（本例选择[加速度-偏航角]输入模式）***
SimEnv = parfeval(@GQUAV_Environment_Mode_AccPsi,0);
    % Available Modes:
    %   - AccPsi: [ax,ay,az,psi(yaw)] unit: m/s^2, rad
    %   - AttThr: *TO BE CONSTRUCTED* [phi(roll),theta(pitch),psi(yaw),Thrust(Force)] unit: rad, N
    %   - TrqThr: *TO BE CONSTRUCTED* (not recommended)[Tx,Ty,Tz,Thrust(Force)] unit: N.m, N
    %   - Motors: *TO BE CONSTRUCTED* (not recommended)[Omega1,Omega2,Omega3,Omega4] unit: rad/s

% 创建交互接口（用于获取反馈信息和发送控制指令）
Interface = GQUAV_Environment_Interface();
% 验证环境与接口是否正常
GQUAV_Environment_Check(SimEnv,Interface);
%% 初始化与预处理
% 系统初始化
tspan = 0:1/freq_ctr:T;
N = length(tspan);
u = [0;0;0;0];
% 数据记录初始化
exprdata.u = zeros(4,N);
exprdata.states = zeros(12,N);
exprdata.tspan = 0:1/freq_ctr:T;
% *** BEGIN: 用户初始化 ***
% 此处用于定义用户的初始化代码，如轨迹生成、控制所需中间变量等

% 初始化MPC控制器参数:
global prob
E = [0,1;0,0];  I = [0;1];
A = blkdiag(E,E,E);
B = blkdiag(I,I,I);
R = diag([1,1,1]);
Q = diag([1,1,1,1,1,1]);
x_ub = [inf;10;inf;10;inf;10];
x_lb = -x_ub;
u_ub = [5;5;5];
u_lb = -u_ub;
prob = MPCSim_init(A,B,Q,R,50,1/freq_ctr,x_ub,x_lb,u_ub,u_lb);
H = prob.Horizen;
Ts = prob.Ts;

% 轨迹生成：
tspan_r = 0:1/freq_ctr:(T+1.2*H*Ts);      % 稍长的时间域
TrajGen = GQUAV_Utils_TrajGen(tspan_r);   % 初始化轨迹生成器
global xi_r u_r psi_r
[x,y,z] = TrajGen.SpiralTraj(1,1,0.5,0.1,0.5,1); % 生成螺旋轨迹
x_diff = (x(2:end)-x(1:end-1))/Ts;
y_diff = (y(2:end)-y(1:end-1))/Ts;
z_diff = (z(2:end)-z(1:end-1))/Ts;

u_r = [x_diff(2:end)-x_diff(1:end-1);...
       y_diff(2:end)-y_diff(1:end-1);...
       z_diff(2:end)-z_diff(1:end-1)]./Ts;

xi_r = [x(1:end-1);x_diff;y(1:end-1);y_diff;z(1:end-1);z_diff];

psi_r = 0.0*ones(N,1);    % 偏航角

% 记录参考值：
exprdata.u_r = u_r;
exprdata.xi_r = xi_r;
exprdata.psi_r = psi_r;
% *** END: 用户初始化 ***
%% 控制过程
fprintf('开始控制，持续时间: %.2f (s)\n',T);
rate = ros2rate(Interface.ROS2Node,freq_ctr);   % 用于实时时间同步
for k = 1:N
    % 1) 反馈
    [time_msr,feedback] = Interface.Measure();
    exprdata.states(:,k) = feedback;    % 记录状态信息
    % 2) 控制
    u = CustomController(feedback,k);
    % 3) 执行
    time_ctr = Interface.Control(u);
    exprdata.u(:,k) = u;                % 记录控制信息
    % 4) 同步
    waitfor(rate);
end
fprintf('控制过程结束\n');
GQUAV_Environment_Close(SimEnv,Interface);
%% 保存实验数据
% 创建ExperimentData文件夹（如果不存在）
if ~exist('../ExperimentData', 'dir')
    mkdir('../ExperimentData');
end
% 生成包含时间的文件名
exprtime = datestr(now, 'yyyymmdd_HHMMSS'); %#ok<*TNOW1,*DATST>
filename = fullfile('../ExperimentData', [exprname,'_data_', exprtime, '.mat']);
% 保存数据
save(filename, 'exprdata', 'exprname', 'exprtime', '-v7.3');
fprintf('\n实验数据已保存至: %s\n', filename);
%% *** 自定义控制器 ***
% 此处用于定义控制器的具体形式，并在循环中被调用
% 输入
%   feedback    -   反馈信息
%   k           -   时间步（用于跟踪、时变、前馈等含时间的场景）
% 输出
%   u           -   将要被执行的控制量
function u = CustomController(feedback,k)
% *** 预处理部分 ***
% 通常用于定义和初始化参数
    global xi_r u_r psi_r prob
% *** 核心部分 ****
% 通常为控制算法主要内容
    xi_measured = feedback([1,7,2,8,3,9]);
    err = xi_r(:,k)-xi_measured;
    % 注意：必须取预测时域长度的参考值送入MPC--------------------------------
    N = prob.Horizen;       % 预测时域
    ur_part = u_r(:,k:k+N-1);     % 取预测时域长度，用于约束
    xr_part = xi_r(:,k+1:k+N);   % 取预测时域长度，用于约束
    % -----------------------------------------------------------------
    u_sol = u_r(:,k) - MPCSim_solv(prob,err,xr_part,ur_part);
    u = [u_sol;psi_r(k)];
% *** 后处理部分 ***
% 通常用于迭代更新、记录保存等
end
%% *** 其他自定义函数 ***
% 此处用于定义其他的函数，如观测器、滤波器等
