function R = GQUAV_Utils_R(phi,theta,psi)
    R = GQUAV_Utils_rotz(psi)*GQUAV_Utils_roty(theta)*GQUAV_Utils_rotx(phi);
end