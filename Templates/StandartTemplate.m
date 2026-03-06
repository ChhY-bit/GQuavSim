% A PID Control Demo for GQUAV
% Each module is designed independently, allowing you to add your own custom code.
% You can edit the code or parameters noted with "***"
clear,clc
Exprname = 'A_Tempalate';
%% 参数设置
% *** 仿真持续时间(s) ***
T = 60;
% *** 控制频率(Hz) ***
global freq_ctr %#ok<*GVMIS>
freq_ctr = 20;
%% 仿真准备
% 导入核心库
addpath(genpath('../Core'));
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
exprdata.t_u = zeros(1,N);
exprdata.states = zeros(12,N);
exprdata.t_s = zeros(1,N);
% *** BEGIN: 用户初始化 ***
% 此处用于定义用户的初始化代码，如轨迹生成、控制所需中间变量等

% e.g.
%TrajGen = GQUAV_Utils_TrajGen(tspan);   % 初始化轨迹生成器
%global x y z psi %#ok<*GVMIS>
%[x,y,z] = TrajGen.SpiralTraj(1,1,1,0.1,0.5,1); % 生成螺旋轨迹
%psi = 0.5*ones(N,1);    % 偏航角

% *** END: 用户初始化 ***
%% 控制过程
fprintf('开始控制，持续时间: %.2f (s)\n',T);
rate = ros2rate(Interface.ROS2Node,freq_ctr);   % 用于实时时间同步
for k = 1:N
    % 1) 反馈
    [time_msr,feedback] = Interface.Measure();
    exprdata.states(:,k) = feedback;    % 记录状态信息
    exprdata.t_s(k) = time_msr;         % 记录状态时间戳
    % 2) 控制
    u = CustomController(feedback,k);
    % 3) 执行
    time_ctr = Interface.Control(u);
    exprdata.u(:,k) = u;                % 记录控制信息
    exprdata.t_u(k) = time_ctr;         % 记录控制时间戳
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
saving_time = datestr(now, 'yyyymmdd_HHMMSS');
filename = fullfile('../ExperimentData', [Exprname,'_exprdata_', saving_time, '.mat']);
% 保存数据
save(filename, 'exprdata', '-v7.3');
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
    global freq_ctr

% *** 核心部分 ****
% 通常为控制算法主要内容
    
% *** 后处理部分 ***
% 通常用于迭代更新、记录保存等
end
%% *** 其他自定义函数 ***
% 此处用于定义其他的函数，如观测器、滤波器等
