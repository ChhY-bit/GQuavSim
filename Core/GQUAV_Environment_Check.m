function GQUAV_Environment_Check(SimEnv,Interface)
fprintf('- 正在启动[加速度-偏航角]输入的四旋翼无人机仿真环境...\n');
pause(1)
% 判断 SimEnv 是否正常运行
try
    % 检查 SimEnv 是否为有效的 Future 对象
    if ~isempty(SimEnv) && isa(SimEnv, 'parallel.FevalFuture')
        % 检查 Future 对象的实际运行状态
        if strcmp(SimEnv.State, 'running')
            fprintf('- 仿真环境启动完成.\n');            
        else
            error('- 仿真环境启动异常，可能是路径不存在或进程被中断.');
        end
    else
        error('仿真环境不存在.');
    end
catch ME
    fprintf('仿真环境启动失败：%s\n', ME.message);
    % 清理资源
    if exist('SimEnv', 'var') && ~isempty(SimEnv)
        try
            SimEnv.cancel;
        catch
            % 忽略取消时的错误
        end
    end
    rethrow(ME);
end
fprintf('=====================================================\n');
end