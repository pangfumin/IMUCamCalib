%run_prototype
%%===============notation===================%%
%R_B_A: rotate from A frame to B frame
%p_a_B, Bp_a: position a in B frame
%q_B_A: rotate from A frame to B frame (JPL)  
%%==========================================%%

clc;
close all;
clear;
addpath('../../Simulation/plotutils');
addpath('../../utils');
datafile = '../../dataset/data_sim_05_circle_rot4.mat';
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


initialCovar = diag([q_var_init, p_var_init  v_var_init,bg_var_init, ba_var_init]);


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


% covariance
firstEkfState.Covar = initialCovar;

ekfState = firstEkfState;
purePropagateState = firstEkfState;


pHat = ground_truth(imageStart,2:4)';
pEst =  ground_truth(imageStart,2:4)';
pMeas = ground_truth(imageStart,2:4)';
qEst =  ground_truth(imageStart,5:8)';
qMeas =  ground_truth(imageStart,5:8)';
vEst = ground_truth_vels(imageStart,2:4)';
vGt = ground_truth_vels(imageStart,2:4)';
bgEst = b_g(:, imageStart);
baEst = b_a(:, imageStart);
bgGt = b_g(:, imageStart);
baGt = b_a(:, imageStart);



onlyImuPro = ground_truth(imageStart,2:4)';
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
            onlyImuPro = [onlyImuPro purePropagateState.p_I_G];
  
        end
  
        imuCnt = imuCnt + 1;
        imu_t = imuTimeStamp(imuCnt);
        
       
    end
    
    % insert GT for compare
    
    subplot(1,2,1);
    SE3_plot(ekfState.q_I_G,ekfState.p_I_G,0.0001,0.00001);
    
    % Get actual measuremnt 
    Cam_z_i = Cam_z(2*state_i-1:2*state_i,:);
 
    
    % Get visual Measurement 
    q_I_G = ekfState.q_I_G;
    p_I_G = ekfState.p_I_G;

    R_I_G = quatToRotMat(q_I_G);
    q_C_G = rotMatToQuat(camera.R_C_I*R_I_G);
    q_C_G = q_C_G/norm(q_C_G);
    p_C_G = p_I_G + R_I_G'*camera.Ip_C;
   

    % for visualization
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
    
    Cam_pose_zHat_i = [p_I_G; q_I_G];
    % generate meas
    Cam_pose_z_i = Cam_pose(state_i,:)';
    Cam_pose_z_i(1:3) = Cam_pose_z_i(1:3) + 0.25 * randn(3,1);
    noise_q = buildUpdateQuat(0.015 * randn(3,1));
    Cam_pose_z_i(4:7) = quatMult(Cam_pose_z_i(4:7), noise_q);
    
    
    
    % calculate 6D residuals
    % and jacobians
    r = calcResidual(Cam_pose_z_i,Cam_pose_zHat_i);
    H = calH();
    R = diag([0.05, 0.05, 0.05, 0.0001, 0.0001, 0.0001]);
    P = ekfState.Covar;
%     
    K = (P*H')/( H*P*H' + R);
    % State correction
    deltaX = K * r;
%     
    ekfState = updateState(ekfState, deltaX); 
     % Covariance correction
    tempMat = (eye(15) - K*H);
    P_corrected = tempMat * P * tempMat' + K * R * K';
    ekfState.Covar = P_corrected;
    
    pEst = [ pEst ekfState.p_I_G];
    pMeas = [ pMeas Cam_pose_z_i(1:3)];
    qEst = [qEst ekfState.q_I_G];
    qMeas = [qMeas Cam_pose_z_i(4:7)];
    vEst = [vEst ekfState.v_I_G];
    bgEst = [bgEst ekfState.bg];
    baEst = [baEst ekfState.ba];
    

    vGt = [vGt ground_truth_vels(imuCnt,2:4)'];
    bgGt = [bgGt b_g(:,imuCnt)];
    baGt = [baGt b_a(:,imuCnt)];


end


%% ================ PLOT =============================%%

p0 = ground_truth(1,2:4);
figure(2);
plot3(p0(1),p0(2),p0(3),'*');
hold on;
plot3(ground_truth(:,2),ground_truth(:,3),ground_truth(:,4),'r');
hold on;
plot3(pHat(1,:),pHat(2,:),pHat(3,:),'g');
hold on;
plot3(onlyImuPro(1,:),onlyImuPro(2,:),onlyImuPro(3,:),'b');

legend('Start','Truth','Est.','Pure IMU');
axis equal;

figure(3);
t = 1: size(pEst,2);
subplot(3,1,1);
plot(t, pEst(1,:), 'r', t, Cam_pose(:,1), 'g', t, pMeas(1,:), 'b');
xlabel('x')
subplot(3,1,2);
plot(t, pEst(2,:), 'r', t, Cam_pose(:,2), 'g', t, pMeas(2,:), 'b');
xlabel('y')
subplot(3,1,3);
plot(t, pEst(3,:), 'r', t, Cam_pose(:,3), 'g', t, pMeas(3,:), 'b');
xlabel('z');

estEuler = quat2eul(qEst');
gtEuler = quat2eul(Cam_pose(:, 4:7));
measEuler = quat2eul(qMeas');

figure(4);

subplot(3,1,1);
plot(t, estEuler(:,1), 'r', t, gtEuler(:,1), 'g', t, measEuler(:,1), 'b' );
xlabel('x')
subplot(3,1,2);
plot(t, estEuler(:,2), 'r', t, gtEuler(:,2), 'g', t, measEuler(:,2), 'b' );
xlabel('y')
subplot(3,1,3);
plot(t, estEuler(:,3), 'r', t, gtEuler(:,3), 'g', t, measEuler(:,3), 'b' );










