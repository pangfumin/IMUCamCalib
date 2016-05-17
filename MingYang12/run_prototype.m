%run_prototype
%%===============notation===================%%
%R_B_A: rotate from A frame to B frame
%p_a_B, Bp_a: position a in B frame
%q_B_A: rotate from a frame to b frame  
%%==========================================%%

clc;
close all;
clear;
addpath('../Simulation/plotutils');
addpath('../utils');
datafile = '../dataset/data_sim_05_line_rot.mat';
load(datafile);

checkerBoardPoints = points;
PointCloud.num = size(checkerBoardPoints,2);

%Set up the camera parameters 
camera.c_u      = Cam.cx; %cu;                   % Principal point [u pixels] 
camera.c_v      = Cam.cy; % cv;                   % Principal point [v pixels]
camera.f_u      = Cam.fx; %  fu;                   % Focal length [u pixels]
camera.f_v      = Cam.fx; % fv;                   % Focal length [v pixels]
camera.distortion      = [0,0,0,0];%distortion;    % distortion

% Body frame is same to Imu frame
camera.R_C_I = R_C_B;
camera.Ip_C = Bp_c0;
camera.Cp_I = -camera.R_C_I*Bp_c0;

Ta = eye(3);
Tg = eye(3);
Ts = zeros(3,3);

imageTimeStamp = Cam_t;
imuTimeStamp = imu_output(:,1);


imageStart = 1;
imageEnd = size(Cam_t,1);
imuStart = 1;


%Set up the noise parameters
noiseParams.u_var_prime = noise.sigma_im*noise.sigma_im;
noiseParams.v_var_prime = noise.sigma_im*noise.sigma_im;
noiseParams.sigma_gc = noise.sigma_gc;               % rot vel var  
noiseParams.sigma_ac = noise.sigma_ac;               % lin accel var  
noiseParams.sigma_wgc = noise.sigma_wgc;            % gyro bias change var 
noiseParams.sigma_wac = noise.sigma_wac;            % accel bias change var


% state : [q  p v bg ba ]
q_var_init = 1e-6 * ones(1,3);         % init rot var
p_var_init = 1e-6 * ones(1,3);         % init pos var
v_var_init = 1e-6 * ones(1,3);         % init velocity var
bg_var_init = 1e-6 * ones(1,3);        % init gyro bias var
ba_var_init = 1e-6 * ones(1,3);        % init accel bias var
qCI_var_init = 1 * ones(1,3);
pIC_var_init = 1 * ones(1,3); 


initialCovar = diag([q_var_init, p_var_init  v_var_init,bg_var_init, ba_var_init,qCI_var_init,pIC_var_init]);


figure(1);
subplot(1,2,1);
plot_checherboard(checkerBoardPoints');
hold on;
view(-40,10);
axis equal;
xlabel('X');ylabel('Y');zlabel('Z');
SE3_plot([0;0;0;1],[0;0;0],0.01,0);


%% ========================Initialize ========================= %%
% nominal state is 16*1   Error state is 15*1     3-D Motion Estimation and Online Temporal Calibration for Camera-IMU Systems
firstEkfState.q_I_G = ground_truth(imageStart,5:8)';
firstEkfState.p_I_G = ground_truth(imageStart,2:4)';
firstEkfState.v_I_G = ground_truth_vels(imageStart,2:4)';
firstEkfState.bg = [0;0;0];
firstEkfState.ba = [0;0;0];

% calib
attitude = [3.0/180,4.0/180,-1/180]*3.14;
init_q_C_I = rotMatToQuat(PQR2rotmat(attitude));
firstEkfState.q_C_I = init_q_C_I;
[r1,r2,r3] = quat2angle(init_q_C_I([4,1:3])','XYZ');
% [r1,r2,r3]*180/3.14
firstEkfState.p_I_C = [-0.1;0.06;-0.04];


% covariance
firstEkfState.Covar = initialCovar;

ekfState = firstEkfState;
purePropagateState = firstEkfState;


pHat = ground_truth(imageStart,2:4)';
onlyImuPro = ground_truth(imageStart,2:4)';
p_I_CHat = [firstEkfState.p_I_C];
rotHat = [r1,r2,r3]';

%% ==========================Loop ==============================%%


imuCnt = imuStart;
imu_t = imuTimeStamp(imuCnt);
for state_i = imageStart+1 : imageEnd
    image_t = Cam_t(state_i);
    while( imu_t <= image_t)
        % todo
        
        if  imuCnt ~= imuStart 
            fprintf('imu_t = %f\n',imu_t);
            %r = randn(12,1);
            last_wm = imu_output(imuCnt-1,5:7)';
            last_am = imu_output(imuCnt-1,2:4)';
            
            cur_wm = imu_output(imuCnt,5:7)';
            cur_am = imu_output(imuCnt,2:4)';
            
            dt = imu_t - imuTimeStamp(imuCnt-1);
            ekfState = ProapagateIMUStateAndCovar(ekfState ,cur_wm,cur_am,last_wm,last_am, dt,noiseParams);
            purePropagateState = ProapagateIMUStateAndCovar(purePropagateState ,cur_wm,cur_am,last_wm,last_am, dt,noiseParams);
%             % For visualize
%             
            pHat = [ pHat ekfState.p_I_G];
            p_I_CHat = [p_I_CHat ekfState.p_I_C];
            q_C_IHat = ekfState.q_C_I;
            [r1,r2,r3] = quat2angle(q_C_IHat([4,1:3])','XYZ');
            rotHat = [rotHat [r1,r2,r3]'];
            
            onlyImuPro = [onlyImuPro purePropagateState.p_I_G];
  
        end
  
        imuCnt = imuCnt + 1;
        imu_t = imuTimeStamp(imuCnt);
        
       
    end
    subplot(1,2,1);
    SE3_plot(ekfState.q_I_G,ekfState.p_I_G,0.0001,0.00001);
    
    % Get actual measuremnt 
    Cam_z_i = Cam_z(2*state_i-1:2*state_i,:);
 
    
    % Get visual Measurement 
    q_I_G = ekfState.q_I_G;
    p_I_G = ekfState.p_I_G;

    R_I_G = quatToRotMat(q_I_G);
    R_C_I = quatToRotMat(ekfState.q_C_I);
    q_C_G = rotMatToQuat(R_C_I*R_I_G);
    q_C_G = q_C_G/norm(q_C_G);
    Ip_C = - R_C_I'*ekfState.p_I_C;
    p_C_G = p_I_G + R_I_G'*Ip_C;
   

    [observedPoints,validIndex] = projectPoints(q_C_G,p_C_G,checkerBoardPoints,PointCloud,Cam,noise.sigma_im);
    Cam_zHat_i = NaN*ones(2,PointCloud.num);
    Cam_zHat_i(:,validIndex) = observedPoints;
    % Plot features
    subplot(1,2,2);
    im  = 255*ones(Cam.Height,Cam.Width);
    showMatchedFeatures(im, im, observedPoints', Cam_z_i(:,validIndex)');
    
    if isempty(validIndex)
        continue;
    end
    
    
    r = calcResidual(Cam_z_i,Cam_zHat_i,validIndex,camera);
    H = calH(ekfState,validIndex,checkerBoardPoints,camera);
    R = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r)/2]));
    P = ekfState.Covar;
%     
    K = (P*H')/( H*P*H' + R);
    % State correction
    deltaX = K * r;
%     
    ekfState = updateState(ekfState, deltaX); 
     % Covariance correction
    tempMat = (eye(21) - K*H);
    P_corrected = tempMat * P * tempMat' + K * R * K';
    ekfState.Covar = P_corrected;

end


%% ================ PLOT =============================%%
plot_result;








