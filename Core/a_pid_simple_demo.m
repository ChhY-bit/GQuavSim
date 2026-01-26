clear,clc
%% 仿真参数
dt = 1e-4;
T = 10;
%% 控制参数
freq_pos = 20;      % 位置环控制频率 (Hz)
freq_att = 100;    % 姿态环控制频率 (Hz)
freq_rate = 1000;    % 角速度环控制频率 (Hz)
ifdiff = 1;        % 是否计算导数
%% 初始化
tspan = 0:dt:T;
N = length(tspan);

UAV1 = GQUAV_Model_UAV();
state = zeros(12,N);

% ----- 控制目标 -----
x = 3; 
y = 3;
z = 3;
psi = 0;    
% -------------------
% ----- PID参数 -----
Kp = 10*[1;1;10];
Ki = 0*[1;1;1];
Kd = 1*[1;1;10];
% ------------------ 
% ----- 控制初始化 -----
u = zeros(4,N);
last_err = [0;0;0];
last_last_err = [0;0;0];
last_u = [0;0;0];

% 
eta_d = zeros(3,N); % 期望姿态
eta_d_dot_rec = zeros(3,N); % 期望姿态导数
force_d = zeros(1,N);   % 期望升力
w_d = zeros(3,N);   % 期望角速度
T_d = zeros(3,N);   % 期望力矩
Omega = zeros(4,N); % 电机转速
last_eta_d = zeros(3,1);
%% 仿真计算
for k = 1:N
    %% 位置环（外层输入）
    if mod(tspan(k),1/freq_pos) == 0
        % 位置环的控制输入
        % 数据准备
        m = 1.65;
        % 控制量
        err = [x;y;z]-[UAV1.States.x;UAV1.States.y;UAV1.States.z];
        du = Kp.* (err - last_err) + Ki.* err / freq_pos + Kd.* (err - 2*last_err + last_last_err) * freq_pos;
        u_temp = last_u + du;
        u(1:3,k) = min(abs(u_temp),[10;10;10]).*sign(u_temp);
        u(4,k) = psi;
        last_last_err = last_err;
        last_err = err;
        last_u = u(1:3,k);
        % 执行：
        [phi_d,theta_d] = GQUAV_Controller_ComputeAttitude(u(1,k),u(2,k),u(3,k),u(4,k));
        psi_d = u(4,k);
        eta_d(:,k) = [phi_d;theta_d;psi_d];
        force_d(k) = GQUAV_Controller_ComputeThrust(u(1,k),u(2,k),u(3,k),m);
        % 计算导数
        if k ~= 1 && ifdiff
            Q = 0.9;   % 滤波因子
            eta_d_dot = (eta_d(:,k) - last_eta_d)*freq_pos;
            eta_d_dot = eta_d_dot.*Q + (1-Q).*eta_d_dot_rec(:,k-1);
            
        else
            eta_d_dot = 0;
        end
        last_eta_d = eta_d(:,k);
        eta_d_dot_rec(:,k) = eta_d_dot;
    else
        u(:,k) = u(:,k-1);
        eta_d(:,k) = eta_d(:,k-1);
        force_d(k) = force_d(k-1);
        eta_d_dot_rec(:,k) = eta_d_dot_rec(:,k-1);
    end
    %% 姿态环
    if mod(tspan(k),1/freq_att) == 0
        % 数据准备
        eta = [UAV1.States.phi;UAV1.States.theta;UAV1.States.psi];  % 直接测量
        % 控制量
        w_d(:,k) = GQUAV_Controller_AttitudeLoop(eta_d(:,k),eta_d_dot,eta);
    else
        w_d(:,k) = w_d(:,k-1);
    end
    %% 角速度环
    if mod(tspan(k),1/freq_rate) == 0
        % 数据准备
        J = diag([15.50e-3,15.50e-3,3.30e-2]);
        w = [UAV1.States.w_x;UAV1.States.w_y;UAV1.States.w_z];  % 直接测量
        % 控制量
        [Tx,Ty,Tz] = GQUAV_Controller_AngleSpeedLoop(w_d(:,k),w,J);
        T_d(:,k) = [Tx;Ty;Tz];
    else
        T_d(:,k) = T_d(:,k-1);
    end
    %% 状态更新
    % 控制分配
    [omg1,omg2,omg3,omg4] = GQUAV_Controller_Allocate(force_d(k),T_d(1,k),T_d(2,k),T_d(3,k));
    if ~isreal([omg1,omg2,omg3,omg4]')
        warning('出现负转速');
        % 出现负转速的原因：加速度给定太大；内环增益过大（收敛速度要求过快）
        % 解决：==> 引入约束机制
        omg1 = real(omg1);omg2 = real(omg2);omg3 = real(omg3);omg4 = real(omg4);
    end
    Omega(:,k) = [omg1;omg2;omg3;omg4];
    % 更新
    UAV1.UpdateState(omg1,omg2,omg3,omg4,k,dt);
    state(:,k) = struct2array(UAV1.States)';
end
%% 结果输出
figure(1)
plot(tspan,state(7:9,:))
title('v')

figure(2)
plot(tspan,state(1:3,:))
title('p')

figure(3)
plot(tspan,state(10:12,:))
title('w')

figure(4)
plot(tspan,state(4:6,:))
hold on
plot(tspan,eta_d.*ones(3,N),'--')
title('eta')
legend('\phi','\theta','\psi')

figure(5)
plot(tspan(1:N-1),(state(7:9,2:end)-state(7:9,1:end-1))/dt)
hold on 
plot(tspan,u(1:3,:),'--');
title('a')
legend('a_x','a_y','a_z')

figure(6)
plot(tspan,Omega)
title('Speed of rotors')
legend('\Omega_1','\Omega_2','\Omega_3','\Omega_4')

figure(7)
plot(tspan,eta_d_dot_rec)
title('eta_d dot')
