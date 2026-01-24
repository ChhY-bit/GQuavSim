clear,clc
uav = GQUAV_Model_UAV();    % 创建的同时自动初始化ros2
for i = 1:100
    uav.PublishState();
    pause(0.1);     % 间隔0.1秒发布一次，共100次
end
uav.ShutdownROS2();