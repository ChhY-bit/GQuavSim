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
% 等待数据稳定
fprintf('- 等待通讯稳定，请稍候...\n');
max_wait_time = 10;  % 最大等待时间(秒)
check_interval = 0.2;  % 检查间隔(秒)
start_time = tic;

while toc(start_time) < max_wait_time
    try
        % 检查Interface中是否收到有效的状态数据
        if ~isempty(Interface.StateSub.LatestMessage)
            latest_msg = Interface.StateSub.LatestMessage;
            
            % 检查数据长度是否满足要求(至少12个元素)
            if iscell(latest_msg.data)
                data_len = length(cell2mat(latest_msg.data));
            elseif isstruct(latest_msg.data) && isfield(latest_msg.data, 'data')
                data_len = length(latest_msg.data.data);
            elseif isnumeric(latest_msg.data)
                data_len = length(latest_msg.data);
            else
                data_len = 0;
            end
            
            if data_len >= 12
                fprintf('- 通讯已建立并稳定\n');
                break;
            end
        end
    catch
        % 忽略检查过程中的错误，继续等待
    end
    pause(check_interval);
end

if toc(start_time) >= max_wait_time
    warning('警告：等待通讯稳定超时，但继续执行...');
end

fprintf('- 就绪\n');  
fprintf('=====================================================\n');
end