function r = calcResidual(Cam_pose_z_i,Cam_pose_zHat_i)

r_p = Cam_pose_z_i(1:3) - Cam_pose_zHat_i(1:3);
Cam_pose_zHat_i(4:7) = Cam_pose_zHat_i(4:7)/ norm(Cam_pose_zHat_i(4:7));
err_q = quatMult(  quatInv(Cam_pose_zHat_i(4:7)),  Cam_pose_z_i(4:7) );
r_q = 2 * ( err_q(1:3));
%  r_q = [0;0;0];
r = [r_p;
    r_q];
end