% A PID Control Demo for GQUAV
% This example demonstrates how GQuavSim works with a PID controller.
% Each module is designed independently, allowing you to add your own custom code.
% You can edit the code or parameters noted with "***"
clear,clc
%% 参数设置
% *** 仿真持续时间(s) ***
T = 60;
% *** 控制频率(Hz) ***
global freq_ctr
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
% *** BEGIN: 用户初始化 ***
% 此处用于定义用户的初始化代码，如轨迹生成、控制所需中间变量等
TrajGen = GQUAV_Utils_TrajGen(tspan);   % 初始化轨迹生成器
global last_err last_last_err last_u
last_err = [0;0;0];
last_last_err = [0;0;0];
last_u = [0;0;0];       % PID中间变量
global x y z psi %#ok<*GVMIS>
[x,y,z] = TrajGen.SpiralTraj(1,1,1,0.1,0.5,1); % 生成螺旋轨迹
psi = 0.5*ones(N,1);    % 偏航角
% *** END: 用户初始化 ***
%% 控制过程
fprintf('开始控制，持续时间: %.2f (s)\n',T);
rate = ros2rate(Interface.ROS2Node,freq_ctr);   % 用于实时时间同步
for k = 1:N
    % 1) 反馈
    feedback = Interface.Measure();
    % 2) 控制
    u = CustomController(feedback,k);
    % 3) 执行
    Interface.Control(u);
    % 4) 同步
    waitfor(rate);
end
fprintf('控制过程结束\n');
GQUAV_Environment_Close(SimEnv,Interface);
%% *** 自定义控制器 ***
% 此处用于定义控制器的具体形式，并在循环中被调用
% 输入
%   feedback    -   反馈信息
%   k           -   时间步（用于跟踪、时变、前馈等含时间的场景）
% 输出
%   u           -   将要被执行的控制量
function u = CustomController(feedback,k)
    global freq_ctr
% *** 预处理部分 ***
    global x y z psi
    global last_last_err last_err last_u
    % PID参数
    Kp = 1.5*[1;1;1];
    Ki = 0.2*[1;1;1];
    Kd = 1.5*[1.35;1.35;1.5];
% *** 核心部分 ****
    err = [x(k);y(k);z(k)] - feedback(1:3); % 获取误差
    du = Kp.* (err - last_err) + ...
         Ki.* err / freq_ctr + ...
         Kd.* (err - 2*last_err + last_last_err) * freq_ctr;
    u_temp = last_u + du;
    %u(1:3) = max(min(u_temp, 5), -5);    % 加速度限幅（正负5）
    u(1:3) = u_temp;
    u(4) = psi(k);
    
% *** 后处理部分 ***
    last_last_err = last_err;    % 更新历史误差
    last_err = err;
    last_u = u_temp;
end
%% *** 其他自定义函数 ***
% 此处用于定义其他的函数，如观测器、滤波器等
