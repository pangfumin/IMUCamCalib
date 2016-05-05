function H = calH(ekfState,validIndex,checkerBoardPoints,camera)
validPointsNum = length(validIndex);
H = [];
for i = 1:validPointsNum
    % calculate 3d position in camera frame
    P_f_G = checkerBoardPoints(:,validIndex(i));
    R_C_I = camera.R_C_I;
    R_I_G = quatToRotMat(ekfState.q_I_G);
    P_f_C = R_C_I*R_I_G*(P_f_G - ekfState.p_I_G)  - R_C_I*camera.Ip_C;
    X = P_f_C(1);
    Y = P_f_C(2);
    Z = P_f_C(3);
    
    Jcam = calJcam(X,Y,Z);
    
    H_theta_i = Jcam*R_C_I*R_I_G*crossMat(P_f_G - ekfState.p_I_G);
    
    H_p_i = -Jcam*R_C_I*R_I_G;
  
    
    Hi = [H_theta_i H_p_i zeros(2,9)];
    H = [H;
        Hi];
 
end

end