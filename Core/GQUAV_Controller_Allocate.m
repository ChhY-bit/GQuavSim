function [Omega_1,Omega_2,Omega_3,Omega_4]=GQUAV_Controller_Allocate(F,Tx,Ty,Tz)
% QUAVS_Controller_Allocate
% 输入控制量F,Tx,Ty,Tz
% 输出分配后的电机转速

    % 参数
    C_A = 3.13e-6;
    C_R = 7.50e-7;
    L = 0.23;
    B = diag([C_A,L*C_A/sqrt(2),L*C_A/sqrt(2),C_R])*...
             [1 1 1 1;1 1 -1 -1;-1 1 1 -1;-1 1 -1 1];
    % 分配
    OMG = sqrt(B\[F;Tx;Ty;Tz]);
    Omega_1 = OMG(1); Omega_2 = OMG(2); Omega_3 = OMG(3); Omega_4 = OMG(4);
end