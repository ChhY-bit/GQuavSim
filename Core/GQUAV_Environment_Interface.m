%% 简介
% 本文件用于创建一个接口，以MATLAB类的形式存在
% 该接口具有两个功能：
% - 获取反馈信息
% - 发布控制指令
classdef GQUAV_Environment_Interface < handle
    properties
        ROS2Node       % ROS2节点
        CmdPub         % 控制指令发布器
        StateSub       % 状态订阅器
        LatestStateMsg % 存储最新的状态消息
    end
    
    methods
        function obj = GQUAV_Environment_Interface()
            % 设置ROS2域ID（必须与环境文件一致才能通信）
            setenv('ROS_DOMAIN_ID', '33');

            % 等待ROS2初始化
            pause(2);

            % 初始化ROS2节点
            obj.ROS2Node = ros2node('interface_node');

            % 创建发布器
            obj.CmdPub = ros2publisher(obj.ROS2Node, 'uav_cmd', 'std_msgs/Float64MultiArray');

            % 创建订阅器 - 接收无人机状态信息
            obj.LatestStateMsg = ros2message('std_msgs/Float64MultiArray');
            obj.StateSub = ros2subscriber(obj.ROS2Node, 'uav_state', 'std_msgs/Float64MultiArray', @(src,msg)obj.StateCallback(src,msg));
        end

        function Control(obj, u)
            % 发布控制指令到ROS2话题 'uav_cmd'
            %   参数说明:
            %   u   - 4维数组，含义因控制方式而异
            %         常用含义: [ax, ay, az, psi_d]
            
            % 创建ROS2消息
            msg = ros2message(obj.CmdPub);
            
            % 设置4维控制数组
            msg.data = u;
            
            % 发布消息
            send(obj.CmdPub, msg);
        end
        
        function feedback = Measure(obj)
            % 获取无人机位置信息
            %   返回值:
            %   feedback   - 包含位置信息的结构体
            %                .x, .y, .z - 位置坐标
            %                .phi, .theta, .psi - 姿态角
            %                .dx, .dy, .dz - 速度
            %                .w_x, .w_y, .w_z - 角速度

            % 注意：在并行环境中，回调可能不工作，使用LatestMessage方式轮询
            % 这与环境文件GQUAV_Environment_AccPsi.m中的处理方式一致
            try
                latest_msg = obj.StateSub.LatestMessage;
                if ~isempty(latest_msg)
                    obj.LatestStateMsg = latest_msg;
                end
            catch ME
                fprintf('轮询LatestMessage失败: %s\n', ME.message);
            end

            % 从最新消息中提取状态信息
            % 处理ROS2 Float64MultiArray消息的不同格式
            if isempty(obj.LatestStateMsg.data)
                error('未收到状态消息，请确认ROS2连接正常');
            end

            % 尝试多种方式获取数据
            if iscell(obj.LatestStateMsg.data)
                state_data = cell2mat(obj.LatestStateMsg.data);
            elseif isstruct(obj.LatestStateMsg.data)
                % 检查是否有嵌套的data字段
                if isfield(obj.LatestStateMsg.data, 'data')
                    state_data = obj.LatestStateMsg.data.data;
                else
                    state_data = obj.LatestStateMsg.data;
                end
            elseif isnumeric(obj.LatestStateMsg.data)
                state_data = obj.LatestStateMsg.data;
            else
                error('无法识别的数据类型: %s', class(obj.LatestStateMsg.data));
            end

            % 确保是列向量
            state_data = state_data(:);

            % 检查数据长度
            if length(state_data) < 12
                if length(state_data) >= 1
                    feedback = [state_data(1); 0; 0; 0; 0; 0];
                    return;
                else
                    error('状态数据为空');
                end
            end

            % TODO：测量的真实性（噪声、延时等的引入）
            measured_data = state_data;

            % 构建反馈信息（返回前6个：x, y, z, phi, theta, psi）
            feedback = measured_data(1:6);
        end
        
        function StateCallback(obj, ~, msg)
            % 状态回调函数
            obj.LatestStateMsg = msg;
        end
        
        function Shutdown(obj)
            % 关闭ROS2节点并清理资源
            if ~isempty(obj.ROS2Node)
                delete(obj.ROS2Node);
                obj.ROS2Node = [];
                obj.CmdPub = [];
                obj.StateSub = [];
            end
        end
    end

end