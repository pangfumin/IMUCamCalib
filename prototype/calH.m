function H = calH(ekfState,validIndex,checkerBoardPoints)
validPointsNum = length(validIndex);
H = [];
for i = 1:validPointsNum
    % calculate 3d position in camera frame
    P_f_G = checkerBoardPoints(:,validIndex(i));
    C_C_I = quatToRotMat(ekfState.q_I_C)';
    P_f_C = C_C_I*quatToRotMat(ekfState.q_I_G)*(P_f_G - ekfState.p_I_G) ...
        - C_C_I*ekfState.p_C_I;
    X = P_f_C(1);
    Y = P_f_C(2);
    Z = P_f_C(3);
    
    Jcam = calJcam(X,Y,Z);
    
    J_theta_G = C_C_I*crossMat(quatToRotMat(ekfState.q_I_G)*(P_f_G - ekfState.p_I_G));
    J_theta_C = -C_C_I*crossMat(quatToRotMat(ekfState.q_I_G)*(P_f_G - ekfState.p_I_G) - ekfState.p_C_I);
    J_p_I = -C_C_I*quatToRotMat(ekfState.q_I_G);
    J_p_C = -C_C_I;
    
    Hi = Jcam*[J_theta_G zeros(3,9) J_p_I J_theta_C J_p_C];
    H = [H;
        Hi];
    
    
    
    
    
end

end