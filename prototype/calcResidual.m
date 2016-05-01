function r = calcResidual(Cam_z_i,Cam_zHat_i,validIndex,camera)
validSampleNum = length(validIndex);
r = [];

for i = 1:validSampleNum
    r_i =   Cam_z_i(:,validIndex(i))- Cam_zHat_i(:,validIndex(i));
    %Idealize
    r = [r;
        r_i(1,1)/camera.f_u;
        r_i(2,1)/camera.f_v];
    
    
end

end