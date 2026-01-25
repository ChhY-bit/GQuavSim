function GQUAV_Environment_Close(SimEnv,Interface)
    fprintf('仿真环境关闭中...\n');
    pause(1);
    SimEnv.cancel;
    Interface.Shutdown;
    fprintf('仿真环境已关闭.\n');
    fprintf('=====================================================');
end