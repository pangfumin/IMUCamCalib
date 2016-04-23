function [camParam,imuParam]=  getSensorInfo(camConfigFile,imuConfigFile)

% set file path
% camConfigFile = '../MH_01_easy/cam_sensor.yaml';
% imuConfigFile = '../MH_01_easy/imu_sensor.yaml';

camStruct = ReadYaml(camConfigFile);
imuStruct = ReadYaml(imuConfigFile);
T = cell2mat(camStruct.T_BS.data);
T_C_B = [T(1:4);T(5:8);T(9:12);T(13:16)];
camParam.R_C_B = T_C_B(1:3,1:3);
camParam.Cp_b0 = T_C_B(1:3,4);

camParam.resolution = cell2mat(camStruct.resolution);
camParam.intrinsics = cell2mat(camStruct.intrinsics);
camParam.distortion = cell2mat(camStruct.distortion_coefficients);
camParam.freq = camStruct.rate_hz;



imuParam.sigma_gc = imuStruct.gyroscope_noise_density;
imuParam.sigma_wgc = imuStruct.gyroscope_random_walk;
imuParam.sigma_ac = imuStruct.accelerometer_noise_density;
imuParam.sigma_wac = imuStruct.accelerometer_random_walk;
imuParam.freq = imuStruct.rate_hz;



