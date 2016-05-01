% sim_prototype
%%===============notation===================%%
%R_B_A: rotate from A frame to B frame
%p_a_B, Bp_a: position a in B frame
%q_B_A: rotate from a frame to b frame  
%%==========================================%%

clc;
close all;
clear;
addpath('../Simulation/plotutils');
datafile = '../dataset/data_sim_04_noise_circle.mat';
load(datafile);

globalPoints = points;

imageStart = 1;
imageEnd = size(Cam_t,1);
imuStart = 1;
%Set up the camera parameters 
camera.c_u      = Cam.cx; %cu;                   % Principal point [u pixels] 
camera.c_v      = Cam.cy; % cv;                   % Principal point [v pixels]
camera.f_u      = Cam.fx; %  fu;                   % Focal length [u pixels]
camera.f_v      = Cam.fx; % fv;                   % Focal length [v pixels]
camera.distortion      = [0,0,0,0];%distortion;    % distortion

camera.R_C_B = R_C_B;
camera.Bp_c0 = Bp_c0;
camera.Cp_b0 = -camera.R_C_B*Bp_c0;

Ta = eye(3);
Tg = eye(3);
Ts = zeros(3,3);

imageTimeStamp = Cam_t;
imuTimeStamp = imu_output(:,1);

% measStart = syn_index(3,imageStart);
% measEnd = syn_index(3,imageEnd);
% initial_p = p_truth(:,measStart); % for visualization

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

initialIMUCovar = diag([q_var_init, p_var_init,v_var_init,bg_var_init, ba_var_init]);

% MSCKF parameters
msckfParams.minTrackLength = 8;        % Set to inf to dead-reckon only
msckfParams.maxTrackLength = 20;      % Set to inf to wait for features to go out of view
msckfParams.maxGNCostNorm  = 0.25;     % Set to inf to allow any triangulation, no matter how bad
msckfParams.minRCOND       = 1e-12;
msckfParams.doNullSpaceTrick = true;
msckfParams.doQRdecomp = true;





%% =============================== Prepare Data ================= %%

numLandmarks = size(Cam_z,2);
Cam_z(Cam_z == -1) = NaN;
index = size(Cam_z,1)/2;

% idealize
for i = 1:index
    validMeas = ~isnan(Cam_z(2*i-1,:));
    Cam_z(2*i-1,validMeas) = (Cam_z(2*i-1,validMeas) - camera.c_u)/camera.f_u;
    Cam_z(2*i,validMeas) = (Cam_z(2*i,validMeas)- camera.c_v)/camera.f_v;
end

% global recorder
featureTracks = {};
trackedFeatureIds = [];

%% =============================== Initialize ================== %%
firstCamMeasurements(1,:) = Cam_z(imageStart*2-1,:);
firstCamMeasurements(2,:) = Cam_z(imageStart*2,:);

%initial
imuState.q_B_G = ground_truth(imageStart,5:8)';
imuState.p_b_G =  ground_truth(imageStart,2:4)';
imuState.v_b_G =  ground_truth_vels(imageStart,2:4)';
imuState.bg =[0;0;0]; 
imuState.ba = [0;0;0];

onlyImuState = imuState;


msckfState.imuState = imuState;
msckfState.imuCovar = initialIMUCovar;
msckfState.bodyCovar = [];
msckfState.imuBodyCovar = [];
msckfState.bodyStates = {};  %camera pose


[msckfState, featureTracks, trackedFeatureIds] = initializeMSCKF(msckfState, firstCamMeasurements, imageStart);
pHat = ground_truth(imageStart,2:4)';
onlyImuPro = ground_truth(imageStart,2:4)';


%% ============================= LOOP  ===================================== %%

map = [];

imuCnt = imuStart;
imu_t = imuTimeStamp(imuCnt);
for state_i = imageStart+1 : imageEnd
    image_t = Cam_t(state_i);
    while( imu_t <= image_t)
        % todo
        
        if  imuCnt ~= imuStart 
%             fprintf('imu_t = %f\n',imu_t);
            %r = randn(12,1);
            last_wm = imu_output(imuCnt-1,5:7)';
            last_am = imu_output(imuCnt-1,2:4)';
            
            cur_wm = imu_output(imuCnt,5:7)';
            cur_am = imu_output(imuCnt,2:4)';
            
            dt = imu_t - imuTimeStamp(imuCnt-1);
            msckfState = ProapagateIMUStateAndCovar(msckfState ,cur_wm,cur_am,last_wm,last_am, dt,noiseParams);
            onlyImuState = ProapagateIMUState(onlyImuState ,cur_wm,cur_am,last_wm,last_am, dt);
            % For visualize
            
            pHat = [ pHat msckfState.imuState.p_b_G];
            onlyImuPro = [onlyImuPro onlyImuState.p_b_G];
         
        end
  
        imuCnt = imuCnt + 1;
        imu_t = imuTimeStamp(imuCnt);
    end
    % todo
%     fprintf('image_t = %f\n',image_t);
    msckfState = augmentState(msckfState, state_i);
    
    %% ========================== Track Features ===================================== %%
    featureTracksToResidualize = {};
    tracksToResidualizeID = [];
    curCamMeasurements(1,:)= Cam_z(state_i*2-1,:);
    curCamMeasurements(2,:) = Cam_z(state_i*2,:);
     
     % For statistics
     addNum = 0;
     deleteNum = 0;
     for featureId = 1:numLandmarks
         meas_k = curCamMeasurements(1:2,featureId);
         outOfView = isnan(meas_k(1,1));
         
         
          if ismember(featureId, trackedFeatureIds)

            if ~outOfView
                %Append observation and append id to cam states
                featureTracks{trackedFeatureIds == featureId}.observations(:, end+1) = meas_k;
                
                %Add observation to current camera
                msckfState.bodyStates{end}.trackedFeatureIds(end+1) = featureId;
            end
            track = featureTracks{trackedFeatureIds == featureId};
            
             if outOfView || (size(track.observations, 2) >= msckfParams.maxTrackLength) || (state_i >= imageEnd)
                                
                %Feature is not in view, remove from the tracked features
                [msckfState, bodyStates, bodyStateIndices] = removeTrackedFeature(msckfState, featureId);
                deleteNum = deleteNum + 1;
                %Add the track, with all of its camStates, to the
                %residualized list
                if length(bodyStates) >= msckfParams.minTrackLength
                    track.bodyStates = bodyStates;
                    track.bodyStateIndices = bodyStateIndices;
                    featureTracksToResidualize{end+1} = track;
                    tracksToResidualizeID(end+1) = featureId;
                end
               
                %Remove the track
                featureTracks = featureTracks(trackedFeatureIds ~= featureId);
                trackedFeatureIds(trackedFeatureIds == featureId) = []; 
             end
         elseif ~outOfView && state_i < imageEnd % && ~ismember(featureId, trackedFeatureIds)
            %Track new feature
            track.featureId = featureId;
            track.observations = meas_k;
            featureTracks{end+1} = track;
            trackedFeatureIds(end+1) = featureId;
            %Add observation to current camera
            msckfState.bodyStates{end}.trackedFeatureIds(end+1) = featureId;
            addNum = addNum + 1;
            
          end
         
     end
     
     
     
   %% ==========================FEATURE RESIDUAL CORRECTIONS======================== %%
    if ~isempty(featureTracksToResidualize)
        % update part
        
        H_o = [];
        r_o = [];
        R_o = [];

        for f_i = 1:length(featureTracksToResidualize)
            track = featureTracksToResidualize{f_i};
            featureID = tracksToResidualizeID(f_i);
            point3D = globalPoints(:,featureID);
            
            % get camStates
            camStates = {};
            for i = 1:length(track.bodyStates)
                q_B_G = track.bodyStates{i}.q_B_G;
                R_B_G = quatToRotMat(q_B_G);
                camstate.q_CG = quatMult( rotMatToQuat(R_C_B),q_B_G);
                camstate.p_c_G = track.bodyStates{i}.p_b_G + R_B_G'*Bp_c0;
                camStates{end+1} = camstate;
                
                
            end
            %Estimate feature 3D location through Gauss Newton inverse depth                    
            [p_f_G, Jcost, RCOND] = calcGNPosEst(camStates, track.observations, noiseParams);
            r = point3D - p_f_G;
            r = sqrt(r'*r);
            fprintf('Error for est: r = %f Jcost = %f RCOND = %f length = %d\n',r,Jcost,RCOND,length(camStates));
            nObs = size(track.observations,2);
            JcostNorm = Jcost / nObs^2;
            
            if JcostNorm > msckfParams.maxGNCostNorm ...
                    || RCOND < msckfParams.minRCOND
%                     || norm(p_f_G) > 50
                
                break;
            else
                map(:,end+1) = p_f_G;
%                 numFeatureTracksResidualized = numFeatureTracksResidualized + 1;
%                 fprintf('Using new feature track with %d observations. Total track count = %d.\n',...
%                     nObs, numFeatureTracksResidualized);
            end
            
            % measurement model
            [r_j] = calcResidual(p_f_G, camStates, track.observations);
            R_j = diag(repmat([noiseParams.u_var_prime, noiseParams.v_var_prime], [1, numel(r_j)/2]));
            [H_o_j, A_j, H_x_j] = calcHoj(p_f_G, msckfState, track.bodyStateIndices,camera);
            
            % Stacked residuals and friends
            if msckfParams.doNullSpaceTrick
                H_o = [H_o; H_o_j];

                if ~isempty(A_j)
                    r_o_j = A_j' * r_j;
                    r_o = [r_o ; r_o_j];

                    R_o_j = A_j' * R_j * A_j;
                    R_o(end+1 : end+size(R_o_j,1), end+1 : end+size(R_o_j,2)) = R_o_j;
                end
                
            else
                H_o = [H_o; H_x_j];
                r_o = [r_o; r_j];
                R_o(end+1 : end+size(R_j,1), end+1 : end+size(R_j,2)) = R_j;
            end
   
        end
        
        if ~isempty(r_o)
                % Put residuals into their final update-worthy form
                if msckfParams.doQRdecomp
                    [T_H, Q_1] = calcTH(H_o);
                    r_n = Q_1' * r_o;
                    R_n = Q_1' * R_o * Q_1;
                else
                    T_H = H_o;
                    r_n = r_o;
                    R_n = R_o;
                end      
                
                 % Build MSCKF covariance matrix
                P = [msckfState.imuCovar, msckfState.imuBodyCovar;
                       msckfState.imuBodyCovar', msckfState.bodyCovar];

                % Calculate Kalman gain
                %K = (P*T_H') / ( T_H*P*T_H' + R_n ); % == (P*T_H') * inv( T_H*P*T_H' + R_n )
                K = (P*T_H') * inv( T_H*P*T_H' + R_n );

                % State correction
                deltaX = K * r_n;
                msckfState = updateState(msckfState, deltaX);

            % Covariance correction
            tempMat = (eye(15 + 9*size(msckfState.bodyStates,2)) - K*T_H);
            P_corrected = tempMat * P * tempMat' + K * R_n * K';

            msckfState.imuCovar = P_corrected(1:15,1:15);
            msckfState.bodyCovar = P_corrected(16:end,16:end);
            msckfState.imuBodyCovar = P_corrected(1:15, 16:end);
            
        end
    end
    
    
  %% ==========================STATE PRUNING======================== %%
    %Remove any camera states with no tracked features
    [msckfState, deletedCamStates] = pruneStates(msckfState);
    fprintf('imageIndex= %d newAdded = %d deleted = %d nowNum = %d bodyState = %d \n',state_i,addNum,deleteNum,size(trackedFeatureIds,2),length(msckfState.bodyStates));
    

    
    
    
    
end

p0 = ground_truth(1,2:4);
figure(1);
plot3(p0(1),p0(2),p0(3),'*');
hold on;
plot3(ground_truth(:,2),ground_truth(:,3),ground_truth(:,4),'r');
hold on;
plot3(pHat(1,:),pHat(2,:),pHat(3,:),'g');
hold on;
plot3(onlyImuPro(1,:),onlyImuPro(2,:),onlyImuPro(3,:),'b');

legend('Start','Truth','Est.','Pure IMU');


figure(2);
t = 1:size(pHat,2);
subplot(3,1,1);
plot(t,ground_truth(t,2),'r',t,pHat(1,:),'g',t,onlyImuPro(1,:),'b');
subplot(3,1,2);
plot(t,ground_truth(t,3),'r',t,pHat(2,:),'g',t,onlyImuPro(2,:),'b');
subplot(3,1,3);
plot(t,ground_truth(t,4),'r',t,pHat(3,:),'g',t,onlyImuPro(3,:),'b');






