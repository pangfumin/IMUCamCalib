function r = calcResidual(Cam_z_i,Cam_zHat_i,validIndex)
validSampleNum = length(validIndex);
r = [];

for i = 1:validSampleNum
    r_i =   Cam_z_i(:,validIndex(i))- Cam_zHat_i(:,validIndex(i));
    r = [r;
        r_i];
    
    
end

end