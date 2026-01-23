function R=GQUAV_Utils_roty(theta)
    s_th=sin(theta);c_th=cos(theta);
    R=[c_th,0,s_th;0,1,0;-s_th,0,c_th];
end