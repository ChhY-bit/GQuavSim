function params = GQUAV_Model_GetDefaultParams()
    params = zeros(18,1);
    params(1) = 3.13e-6;    %   C_A        -   1.  气动阻力常数
    params(2) = 7.50e-7;    %   C_R        -   2.  反力矩系数
    params(3) = 0.23;       %   L          -   3.  旋翼臂长
    params(4) = 1.65;       %   m          -   4.  无人机质量
    params(5) = 15.50e-3;   %   Jxx        -   5.  x主轴惯量
    params(6) = 15.50e-3;   %   Jyy        -   6.  y主轴惯量
    params(7) = 3.30e-2;    %   Jzz        -   7.  z主轴惯量
    params(8) = 6.00e-5;    %   Jxy        -   8.  xy轴互惯量
    params(9) = 6.00e-5;    %   Jxz        -   9.  xz轴互惯量
    params(10) = 6.00e-5;   %   Jyz        -   10. yz轴互惯量
    % ---------- 扰动因素 ----------
    params(11) = 6.00e-5;   %   Jr         -   11. 翼桨惯量
    params(12) = 0.01;      %   mu_x       -   12. x方向平移阻力系数
    params(13) = 0.01;      %   mu_y       -   13. y方向平移阻力系数
    params(14) = 0.01;      %   mu_z       -   14. z方向平移阻力系数
    params(15) = 0.01;      %   lambda_x   -   15. x方向旋转阻力系数
    params(16) = 0.01;      %   lambda_y   -   16. y方向旋转阻力系数
    params(17) = 0.01;      %   lambda_z   -   17. z方向旋转阻力系数
    % ---------- -------- ----------
    params(18) = 9.81;      %   g          -   18. 重力加速度
end
