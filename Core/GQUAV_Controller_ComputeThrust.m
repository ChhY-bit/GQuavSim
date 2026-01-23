function F_d=GQUAV_Controller_ComputeThrust(ax,ay,az,m)
% GQUAV_Controller_ComputeThrust
% 设质量为m，输入期望加速度ax,ay,az
    g = 9.81;
% 计算得出期望的升力大小F_d
    F_d = m*norm([ax,ay,az+g]);
end