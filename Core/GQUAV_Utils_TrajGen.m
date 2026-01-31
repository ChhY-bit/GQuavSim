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
            % xc    -   中心x坐标(m)
            % yc    -   中心y坐标(m)
            % zc    -   中心z坐标(m)
            % v     -   速度(m/s)
            % L     -   边长(m)
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

        function [x,y,z] = SpiralTraj(obj,xc,yc,z0,v,w,r)
            % 生成螺旋轨迹（从第一象限开始）
            % xc    -   中心x坐标(m)
            % yc    -   中心y坐标(m)
            % z0    -   起始z坐标(m)
            % v     -   上升速度(m/s)
            % w     -   角速度(rad/s)
            % r     -   半径(m)
            x_fun = @(t) xc+r*cos(w*t);
            y_fun = @(t) yc+r*sin(w*t);
            x = x_fun(obj.tspan);
            y = y_fun(obj.tspan);
            z = z0+ v*obj.tspan;
        end
    end
end