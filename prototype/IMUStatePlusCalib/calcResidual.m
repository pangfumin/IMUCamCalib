function r = calcResidual(Cam_z_i,Cam_zHat_i,validIndex,camera)
validSampleNum = length(validIndex);
r = [];

for i = 1:validSampleNum
    r_i =   Cam_z_i(:,validIndex(i))- Cam_zHat_i(:,validIndex(i));
    
    % Idealize
%     r_i(1,1) =(r_i(1,1) - camera.c_u)/camera.f_u;
%     r_i(2,1) =(r_i(2,1) - camera.c_v)/camera.f_v;
    
    
    r = [r;
        r_i(1,1) / camera.f_u;
        r_i(2,1) / camera.f_v;];
    
    
end

end