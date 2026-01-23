function [W,W_inv]=GQUAV_Utils_W(phi,theta,psi) %#ok<*INUSD>
    W = [1 sin(phi)*tan(theta) cos(theta)*tan(theta);...
           0 cos(phi) -sin(phi);
           0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
    W_inv = [1 0 -sin(theta);0 cos(phi) sin(phi)*cos(theta);0 -sin(phi) cos(phi)*cos(theta)];
end