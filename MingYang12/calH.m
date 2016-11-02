function H = calH(ekfState,validIndex,checkerBoardPoints,camera)
validPointsNum = length(validIndex);
H = [];
for i = 1:validPointsNum
    % calculate 3d position in camera frame
    P_f_G = checkerBoardPoints(:,validIndex(i));
    R_C_I = quatToRotMat(ekfState.q_C_I);
    R_I_G = quatToRotMat(ekfState.q_I_G);
    P_f_C = R_C_I*R_I_G*(P_f_G - ekfState.p_I_G) + ekfState.p_I_C;
    X = P_f_C(1);
    Y = P_f_C(2);
    Z = P_f_C(3);
    
    Jcam = calJcam(X,Y,Z);
    
    H_theta_i = Jcam*R_C_I*R_I_G*crossMat(P_f_G - ekfState.p_I_G);
    
    H_p_i = -Jcam*R_C_I*R_I_G;
    
    PI_phi_i = Jcam*R_C_I*crossMat(R_I_G*(P_f_G - ekfState.p_I_G));
    PI_p_i = Jcam;
  
    
    Hi = [H_theta_i H_p_i zeros(2,9) PI_phi_i PI_p_i];
    H = [H;
        Hi];
 
end

end