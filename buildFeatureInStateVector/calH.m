function H = calH(ekfState,validIndex,PointCloud,camera)
validPointsNum = length(validIndex);
H = [];
points = reshape(ekfState.featureState,[3,PointCloud.num]);
for i = 1:validPointsNum
    % calculate 3d position in camera frame
    P_f_G = points(:,validIndex(i));
    R_C_I = camera.R_C_I;
    R_I_G = quatToRotMat(ekfState.q_I_G);
    P_f_C = R_C_I*R_I_G*(P_f_G - ekfState.p_I_G)  - R_C_I*camera.Ip_C;
    X = P_f_C(1);
    Y = P_f_C(2);
    Z = P_f_C(3);
    
    Jcam = calJcam(X,Y,Z);
    
    H_theta_i = Jcam*R_C_I*R_I_G*crossMat(P_f_G - ekfState.p_I_G);
    
    H_p_i = -Jcam*R_C_I*R_I_G;
    
    H_f_i = Jcam*R_C_I*R_I_G;
  
    Hi = zeros(2,15+PointCloud.num*3);
    Hi(:,1:6) = [ H_theta_i H_p_i];
    Hi(:,(15+(i-1)*3+1):(15+i*3)) = H_f_i;
    H = [H;
        Hi];
 
end

end