function [projectedPoints,index] = projectPoints(q_C_G,p_c_G,ekfState,PointCloud,Cam,image_noise)
R_C_G = quatToRotMat(q_C_G);
points = reshape(ekfState.featureState,[3,PointCloud.num]);
% Project Points
Cp_g0 = -R_C_G*p_c_G;
Cp_g0 = repmat(Cp_g0,[1,PointCloud.num]);
pointCam = R_C_G*points + Cp_g0;

isFrontCam = pointCam(3,:) > 0; % Z axis > 0
idealPoint(1,:) = (pointCam(1,:)./pointCam(3,:))';
idealPoint(2,:) = (pointCam(2,:)./pointCam(3,:))';
F = [Cam.fx,0;
    0,Cam.fy];
Center = [Cam.cx;
          Cam.cy];
Center = repmat(Center,[1,PointCloud.num]);
Pixel = F*idealPoint + Center;
% Apply noise
n_im =  image_noise*randn(2,PointCloud.num);
Pixel =Pixel + n_im;

isInWidthRange =  Pixel(1,:) < (Cam.Width - 10) &  Pixel(1,:) >10; 
isInHeightRange = Pixel(2,:) < (Cam.Height - 10) & Pixel(2,:) >10;
isInRange = isInWidthRange & isInHeightRange & isFrontCam;
index = find(isInRange == 1);
projectedPoints = Pixel(:,index);

end