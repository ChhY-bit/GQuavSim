clear,clc
%% 参数设置
T = 20;         % 仿真持续时间(s)
freq_ctr = 50;  % 控制频率(Hz)
%% 仿真准备
addpath(genpath('../Core'));
SimEnv = parfeval(@GQUAV_Environment_AccPsi,0);     % 启动仿真环境
Interface = GQUAV_Environment_Interface();           % 创建交互接口
GQUAV_Environment_Check(SimEnv,Interface);          % 验证是否成功
%% 控制过程
% ----- 控制目标 -----
x = 0;
y = 0;
z = 3;
psi = 0;
% -------------------
% ----- PID参数 -----
Kp = [0.4;0.4;1];
Ki = [0;0;0];
Kd = [1;1;0.7];
% ------------------
% ----- 控制初始化 -----
u = [0;0;0;0];
last_err = [0;0;0];
last_last_err = [0;0;0];
last_u = [0;0;0];
% --------------------
fprintf('开始控制过程，持续时间: %.2f (s)\n',T);
t_start = tic;
k = 0;
while toc(t_start) < T
    %% TODO: 在此处添加控制逻辑
    % 获取反馈
    feedback = Interface.Measure();
    err = [x;y;z] - feedback(1:3);
    % 控制律（增量式PID）
    du = Kp.* (err - last_err) + Ki.* err / freq_ctr + Kd.* (err - 2*last_err + last_last_err) * freq_ctr;
    u(1:3) = last_u + du;
    u(4) = psi;
    % 执行
    Interface.Control(u);
    % 更新历史误差
    last_last_err = last_err;
    last_err = err;
    last_u = u(1:3);
    %% 实时同步：等待真实时间达到仿真时间
    k = k+1;
    t_real = toc(t_start);
    if t_real < 1/freq_ctr*(k+1)
        pause(1/freq_ctr*(k+1) - t_real);
    else
        warning('控制超时，请考虑改进时间复杂度！')
    end
end
fprintf('控制过程结束\n');
%% 结束仿真环境
GQUAV_Environment_Close(SimEnv,Interface);

