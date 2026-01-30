classdef GQUAV_Utils_TrajGen < handle
    properties
        tspan
    end
    
    methods
        function obj = GQUAV_Utils_TrajGen(tspan)
            obj.tspan = tspan;
        end

        function [x,y,z] = SquareTraj(obj,xc,yc,zc,v,L)
            % 生成正方形轨迹（从第一象限开始）
            % xc    -   中心x坐标
            % yc    -   中心y坐标
            % zc    -   中心z坐标
            % v     -   速度
            % L     -   边长
            T = L/v;    % 单边耗时
            x_fun = @(t) ...
                    (mod(t,4*T)<=T).*((xc+L/2)-v*mod(t,4*T))+...
                    ((mod(t,4*T)>T).*(mod(t,4*T)<=2*T)).*(xc-L/2)+...
                    ((mod(t,4*T)>2*T).*(mod(t,4*T)<=3*T)).*((xc-5*L/2)+v*mod(t,4*T))+...
                    (mod(t,4*T)>3*T).*(xc+L/2);
            y_fun = @(t) ...
                    (mod(t-T,4*T)<=T).*((yc+L/2)-v*mod(t-T,4*T))+...
                    ((mod(t-T,4*T)>T).*(mod(t-T,4*T)<=2*T)).*(yc-L/2)+...
                    ((mod(t-T,4*T)>2*T).*(mod(t-T,4*T)<=3*T)).*((yc-5*L/2)+v*mod(t-T,4*T))+...
                    (mod(t-T,4*T)>3*T).*(yc+L/2);
            x = x_fun(obj.tspan);
            y = y_fun(obj.tspan);
            z = zc*ones(1,length(obj.tspan));
        end
    end
end