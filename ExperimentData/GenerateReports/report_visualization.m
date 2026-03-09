clear,clc,close all,warning off
%% 读取实验数据
% 绘图前必须先将实验产生的mat数据移动至当前文件夹下
experiment = report_read_exprdata();
expr_num = length(experiment);  % 实验数据组数
%% 绘图设置
% 默认字体字号
set(0, 'DefaultAxesFontName', 'Times New Roman');   % 坐标轴字体
set(0, 'DefaultAxesFontSize', 12);                  % 坐标轴字号
set(0, 'DefaultTextFontName', 'Times New Roman');   % 文本字体
set(0, 'DefaultTextFontSize', 12);                  % 文本字号
% 默认解释器 - latex
set(0, 'DefaultAxesTickLabelInterpreter', 'latex');
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
%% 绘制实验结果
ylabel_tag_states = {'$x\;\mathrm{(m)}$','$y\;\mathrm{(m)}$','$z\;\mathrm{(m)}$',...
              '$\phi\;\mathrm{(rad)}$','$\theta\;\mathrm{(rad)}$','$\psi\;\mathrm{(rad)}$',...
              '$\dot{x}\;\mathrm{(m/s)}$','$\dot{y}\;\mathrm{(m/s)}$','$\dot{z}\;\mathrm{(m/s)}$',...
              '$\dot{\phi}\;\mathrm{(rad)}$','$\dot{\theta}\;\mathrm{(rad)}$','$\dot{\psi}\;\mathrm{(rad)}$'};
ylabel_tag_input = {'$\ddot{x}\;\mathrm{(m/s^2)}$','$\ddot{y}\;\mathrm{(m/s^2)}$',...
                    '$\ddot{z}\;\mathrm{(m/s^2)}$','$\psi\;\mathrm{(rad})$'};

color_tag = {'r','b','g','m','c','y'};
for expr_id = 1:expr_num
    %% 绘制状态变量
    % 位置
    figure(1)
    i = 1;
    for k = [1,7,2,8,3,9]
        subplot(3,2,i)
        plot(experiment{expr_id}.exprdata.tspan,experiment{expr_id}.exprdata.states(k,:),...
             'DisplayName',experiment{expr_id}.exprname,'Color',color_tag{expr_id},'LineWidth',1.5)
        hold on
        legend('Location','best')
        ylabel(ylabel_tag_states{k})
        xlabel('$t\;\mathrm{(s)}$')
        title(['Position ','$\xi_',num2str(i),'(t)$'])
        i = i+1;
    end
    % 姿态
    figure(2)
    i = 1;
    for k = [4,10,5,11,6,12]
        subplot(3,2,i)
        plot(experiment{expr_id}.exprdata.tspan,experiment{expr_id}.exprdata.states(k,:),...
             'DisplayName',experiment{expr_id}.exprname,'Color',color_tag{expr_id},'LineWidth',1.5)
        hold on
        legend('Location','best')
        ylabel(ylabel_tag_states{k})
        xlabel('$t\;\mathrm{(s)}$')
        title(['Attitude ','$\eta_',num2str(i),'(t)$'])
        i = i+1;
    end
    %% 绘制三轴控制输入
    figure(3)
    for k = 1:3
        subplot(3,1,k)
        stairs(experiment{expr_id}.exprdata.tspan,experiment{expr_id}.exprdata.u(k,:),...
            'DisplayName',experiment{expr_id}.exprname,'Color',color_tag{expr_id},'LineWidth',1.5)
        hold on
        legend('Location','best')
        ylabel(ylabel_tag_input{k})
        xlabel('$t\;\mathrm{(s)}$')
        title(['Control Input ','$u_',num2str(k),'(t)$'])
    end
    %% 绘制空间轨迹
    figure(4)
    plot3(experiment{expr_id}.exprdata.states(1,:),...
          experiment{expr_id}.exprdata.states(2,:),...
          experiment{expr_id}.exprdata.states(3,:),...
          'DisplayName',experiment{expr_id}.exprname,'Color',color_tag{expr_id},'LineWidth',1.5)
    xlabel('$x\;\mathrm{(m)}$')
    ylabel('$y\;\mathrm{(m)}$')
    zlabel('$z\;\mathrm{(m)}$')
    hold on
    grid on
    axis equal
    legend('Location','best')
end

%% 参考量对比：
u_ref = experiment{1}.exprdata.u_r;
x_ref = experiment{1}.exprdata.xi_r;
tspan = experiment{1}.exprdata.tspan;
N = length(tspan);
% 位置：
figure(1)
i = 1;
for k = 1:6
    subplot(3,2,i)
    plot(tspan,x_ref(k,1:N),'k--','DisplayName','Reference','LineWidth',1.5)
    hold on
    legend('Location','best')
    i = i+1;
end

% 输入：
figure(3)
for k = 1:3
    subplot(3,1,k)
    stairs(tspan,u_ref(k,1:N),'k--','DisplayName','Reference','LineWidth',1.5)
    hold on
    legend('Location','best')
end

figure(4)
plot3(x_ref(1,1:N),x_ref(3,1:N),x_ref(5,1:N),'k--','DisplayName','Reference','LineWidth',1.5)
legend('Location','best')
%% 保存所有图形
% report_save_figure();

