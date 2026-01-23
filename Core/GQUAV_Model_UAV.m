classdef GQUAV_Model_UAV < handle
    %QUAV 四旋翼无人机模型
    %   创建一个确定的四旋翼无人机模型
    %   参数说明：
    %   C_A         -   1.气动升力常数
    %   C_R         -   2.反力矩系数
    %   L           -   3.旋翼臂长
    %   m           -   4.无人机质量
    %   J           -   5.无人机惯性张量
    %   J_r         -   6.翼桨惯量
    %   mu_x        -   7.x方向平移阻力系数
    %   mu_y        -   8.y方向平移阻力系数
    %   mu_z        -   9.z方向平移阻力系数
    %   lambda_x    -   10.x方向旋转阻力系数
    %   lambda_y    -   11.y方向旋转阻力系数
    %   lambda_z    -   12.z方向旋转阻力系数
    %   g           -   13.重力加速度
    properties
        Params
        States
    end

    methods
        function obj = GQUAV_Model_UAV(params,states_init)
            % QUAV 构造此类的实例
            %   创建无人机示例，参数初始化
            if nargin == 0
                params = GQUAV_Model_GetDefaultParams();
                states_init = zeros(12,1);
            elseif length(params(:)) ~= 18
                error('Wrong Number of Parameters!')
            end
            obj.Params.C_A = params(1);
            obj.Params.C_R = params(2);
            obj.Params.L = params(3);
            obj.Params.m = params(4);
            % ----- 惯性张量 -----
            Jxx = params(5); Jyy = params(6); Jzz = params(7);
            Jxy = params(8); Jxz = params(9); Jyz = params(10);
            obj.Params.J = [Jxx Jxy Jxz;Jxy Jyy Jyz;Jxz Jyz Jzz];
            % -------------------
            obj.Params.J_r = params(11);
            obj.Params.mu_x = params(12);
            obj.Params.mu_y = params(13);
            obj.Params.mu_z = params(14);
            obj.Params.lambda_x = params(15);
            obj.Params.lambda_y = params(16);
            obj.Params.lambda_z = params(17);
            obj.Params.g = params(18);

            obj.States.x = states_init(1);
            obj.States.y = states_init(2);
            obj.States.z = states_init(3);
            obj.States.phi = states_init(4);
            obj.States.theta = states_init(5);
            obj.States.psi = states_init(6);
            obj.States.dx = states_init(7);
            obj.States.dy = states_init(8);
            obj.States.dz = states_init(9);
            obj.States.w_x = states_init(10);
            obj.States.w_y = states_init(11);
            obj.States.w_z = states_init(12);
        end

        function UpdateState(obj,Omega_1,Omega_2,Omega_3,Omega_4,t,dt)
            %state_input 依据电机转速指令输入更新状态
            %   参数说明
            %   Omega_1     -   1号电机转速
            %   Omega_2     -   2号电机转速
            %   Omega_3     -   3号电机转速
            %   Omega_4     -   4号电机转速
            %   dt          -   仿真时间步长
            it = obj.Params;
            % 分配矩阵：
            B = diag([it.C_A,...
                      it.L*it.C_A/sqrt(2),...
                      it.L*it.C_A/sqrt(2),...
                      it.C_R])*...
                      [1 1 1 1;1 1 -1 -1;-1 1 1 -1;-1 1 -1 1];
            % 计算动力：
            tau = B*[Omega_1^2;Omega_2^2;Omega_3^2;Omega_4^2];
            %% 动力学方程
            % 数据准备
            eta = [obj.States.phi,obj.States.theta,obj.States.psi]';
            p = [obj.States.x,obj.States.y,obj.States.z]';
            dp = [obj.States.dx,obj.States.dy,obj.States.dz]';
            w = [obj.States.w_x,obj.States.w_y,obj.States.w_z]';
            mu = -diag([it.mu_x,it.mu_y,it.mu_z]);
            J = obj.Params.J;
            lambda = diag([it.lambda_x,it.lambda_y,it.lambda_z]);
            % 动态方程
            Jxx = obj.Params.J(1,1); Jyy = obj.Params.J(2,2); Jzz = obj.Params.J(3,3);
            Omega_r = Omega_1-Omega_2+Omega_3-Omega_4;
            Lambda = @(w)[0,-it.J_r*Omega_r-Jzz*w(3),Jyy*w(2);...
                          it.J_r*Omega_r+Jzz*w(3),0,-Jxx*w(1);...
                          -Jyy*w(2),Jxx*w(1),0]-lambda;
            dynaics = @(dp,w,eta,tau)...
                [dp;... % \dot{p}
                 (mu*dp-[0;0;it.m*it.g]+GQUAV_Utils_R(eta(1),eta(2),eta(3))*[0;0;tau(1)])/it.m;... % \ddot{p}
                 J\(Lambda(w)*w+tau(2:4));... % \dot{\omega}
                 GQUAV_Utils_W(eta(1),eta(2),eta(3))*w]; % \dot{\eta}
            %% 状态更新
            x = [p;dp;w;eta];
            u = tau;
            fun = @(x,u,t)dynaics(x(4:6),x(7:9),x(10:12),u);
            new_states=GQUAV_Utils_UpdateRK4(fun,x,u,t,dt);
            obj.States.x = new_states(1);
            obj.States.y = new_states(2);
            obj.States.z = new_states(3);
            obj.States.dx = new_states(4);
            obj.States.dy = new_states(5);
            obj.States.dz = new_states(6);
            obj.States.w_x = new_states(7);
            obj.States.w_y = new_states(8);
            obj.States.w_z = new_states(9);
            obj.States.phi = new_states(10);
            obj.States.theta = new_states(11);
            obj.States.psi = new_states(12);
        end
    end
end