clear,clc
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'Core'));
%% 仿真参数
dt = 1e-4;
T = 20;
%% 控制参数
freq_pub = 10;      % 发布频率 (Hz)
freq_pos = 20;      % 位置环控制频率 (Hz)
freq_att = 100;    % 姿态环控制频率 (Hz)
freq_rate = 1000;    % 角速度环控制频率 (Hz)
ifdiff = 1;        % 是否计算导数
%% 初始化
tspan = 0:dt:T;
N = length(tspan);

UAV1 = GQUAV_Model_UAV();
%u = [2*cos(tspan);2*sin(tspan+pi/4);0.5*ones(1,N);0.5*ones(1,N)];
u = [[0.1;0.2;0.5;0.5].*ones(4,(N-1)/2),[0;0;0;-1].*ones(4,N-(N-1)/2)];
%% 实时仿真时间同步
t_start = tic;  % 记录仿真开始时间

%% 仿真计算
for k = 1:N
    %% 信息发布
    if mod(tspan(k),1/freq_pub) == 0
        UAV1.PublishState();
    end
    %% 位置环（外层输入）
    if mod(tspan(k),1/freq_pos) == 0
        % 位置环的控制输入
        % 数据准备
        m = 1.65;
        % 控制量
        [phi_d,theta_d] = GQUAV_Controller_ComputeAttitude(u(1,k),u(2,k),u(3,k),u(4,k));
        psi_d = u(4,k);
        eta_d = [phi_d;theta_d;psi_d];
        thrust_d = GQUAV_Controller_ComputeThrust(u(1,k),u(2,k),u(3,k),m);
        % 计算导数
        if k ~= 1 && ifdiff
            eta_d_dot = (eta_d - last_eta_d)*freq_pos;  % Euler差分法
        else
            eta_d_dot = 0;
        end
        last_eta_d = eta_d;
    end
    %% 姿态环
    if mod(tspan(k),1/freq_att) == 0
        % 数据准备
        eta = [UAV1.States.phi;UAV1.States.theta;UAV1.States.psi];  % 直接测量
        % 控制量
        w_d = GQUAV_Controller_AttitudeLoop(eta_d,eta_d_dot,eta);
    end
    %% 角速度环
    if mod(tspan(k),1/freq_rate) == 0
        % 数据准备
        J = diag([15.50e-3,15.50e-3,3.30e-2]);
        w = [UAV1.States.w_x;UAV1.States.w_y;UAV1.States.w_z];  % 直接测量
        % 控制量
        [Tx,Ty,Tz] = GQUAV_Controller_AngleSpeedLoop(w_d,w,J);
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
    
    %% 实时同步：等待真实时间达到仿真时间
    t_real = toc(t_start);
    if t_real < tspan(k)
        pause(tspan(k) - t_real);
    end
end
UAV1.ShutdownROS2();