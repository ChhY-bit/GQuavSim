%% 简介
% - 本m文件提供了一个[加速度-偏航角]输入方式的仿真环境
% - 一旦运行本m文件，将创建一个名为/uav_node的ROS2节点
% - 该ROS2节点将发布名为/uav_state的消息，该消息包含了无人机状态信息（12+1维数组）
% - 该ROS2节点将订阅名为/uav_cmd的消息，该消息包含了无人机控制指令（4维数组）
% - 外部务必使用parfeval函数并行执行
clear,clc

%% ROS2配置
% 设置ROS2域ID（确保所有通信节点在同一域）
setenv('ROS_DOMAIN_ID', '33');  % 可以使用0-101之间的任意值

% 等待ROS2初始化
pause(2);
disp('ROS2节点已初始化，等待控制指令...');
disp('========================================');
%% 仿真参数
dt = 1e-4;
%% 控制参数
freq_pub = 100;      % 发布频率 (Hz)
freq_att = 100;    % 姿态环控制频率 (Hz)
freq_rate = 1000;    % 角速度环控制频率 (Hz)
ifdiff = 0;        % 是否计算导数
%% 初始化
k = 0;  % 标记仿真步数
UAV1 = GQUAV_Model_UAV();
last_eta_d = [0;0;0];
last_eta_d_dot = [0;0;0];
w_d = [0;0;0];  % 初始化角速度指令
T_d = [0;0;0];  % 初始化力矩指令
%% 实时仿真时间同步
t_start = tic;  % 记录仿真开始时间
%% 仿真计算
while(1)
    %% 信息发布
    if mod(k,freq_pub) == 0
        UAV1.PublishState();
    end
    %% 位置环（外层输入）
    % 位置环的控制输入
    % 数据准备
    m = 1.65;       % 【缺少闭环，若参数不准很可能造成加速度执行不准】

    % 主动轮询获取最新指令（在并行环境中回调可能不工作）
    try
        latest_msg = UAV1.CmdSub.LatestMessage;
        if ~isempty(latest_msg)
            UAV1.CmdData = latest_msg.data;
        end
    catch
        % 如果轮询失败，使用回调方式（主线程模式）
    end

    % 控制量
    u = UAV1.CmdData;
    [phi_d,theta_d] = GQUAV_Controller_ComputeAttitude(u(1),u(2),u(3),u(4));    %【这步是开环】
    psi_d = u(4);
    eta_d = [phi_d;theta_d;psi_d];
    thrust_d = GQUAV_Controller_ComputeThrust(u(1),u(2),u(3),m);
    % 计算导数
    if ifdiff
        Q = 0.90;   % 滤波因子
        eta_d_dot = (eta_d - last_eta_d)/dt;  % Euler差分法
        eta_d_dot = Q*eta_d_dot + (1-Q)*last_eta_d_dot;  % 滤波处理
        last_eta_d = eta_d;
        last_eta_d_dot = eta_d_dot;
    else
        eta_d_dot = 0;
    end
    %% 姿态环
    if mod(k,freq_att) == 0
        % 数据准备
        eta = [UAV1.States.phi;UAV1.States.theta;UAV1.States.psi];  % 直接测量
        % 控制量
        w_d = GQUAV_Controller_AttitudeLoop(eta_d,eta_d_dot,eta);
    end
    %% 角速度环
    if mod(k,freq_rate) == 0
        % 数据准备
        J = diag([15.50e-3,15.50e-3,3.30e-2]);
        w = [UAV1.States.w_x;UAV1.States.w_y;UAV1.States.w_z];  % 直接测量
        % 控制量
        [Tx,Ty,Tz] = GQUAV_Controller_AngleSpeedLoop(w_d,w,J);  %【这步是开环】【但该模式下角速度有闭环,参数不准的影响没有加速度那么显著】
        T_d = [Tx;Ty;Tz];
    end
    %% 状态更新
    % 控制分配
    [omg1,omg2,omg3,omg4] = GQUAV_Controller_Allocate(thrust_d,T_d(1),T_d(2),T_d(3));
    if ~isreal([omg1,omg2,omg3,omg4]')
        warning('出现负转速');
        % 出现负转速的原因：加速度给定太大；内环增益过大（收敛速度要求过快）
        % 解决：==> 引入约束机制
        omg1 = real(omg1);omg2 = real(omg2);omg3 = real(omg3);omg4 = real(omg4);
    end
    % 更新
    UAV1.UpdateState(omg1,omg2,omg3,omg4,k,dt);
    % disp([omg1;omg2;omg3;omg4]);    % debug
    %% 实时同步：等待真实时间达到仿真时间
    t_real = toc(t_start);
    if t_real < (k+1)*dt
        pause((k+1)*dt - t_real);
    end
    k = k+1;
end
UAV1.ShutdownROS2();