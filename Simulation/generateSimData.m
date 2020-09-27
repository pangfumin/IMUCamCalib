%generateSimData.m

clc;
close all;
clear;
addpath('./plotutils');

%% ===== option == %%
fileName = 'data_sim_05_circle_rot4.mat';
%trajectory time in seconds
tf = 4*pi;
syms tsym;
% x = [ 1 y z roll pitch yaw]
%x_func = [cos(tsym)-1 ; sin(tsym); tsym*0.2  ; sin(tsym)*0.2 ; -tsym ; -sin(tsym)*0.2];
%x_func = [cos(tsym)-1 ; sin(tsym); tsym*0.2  ; sin(tsym)*0.2 ; -sin(tsym)*0.1 ;cos(tsym)*0.2 ];
% x_func = [0.3*cos(tsym);  0.3*sin(tsym); 0.1*sin(3*tsym);0;0;0 ];
x_func = [0.1*cos(10*tsym); 0.1*sin(20*tsym); 0.1*sin(30*tsym);0.30*sin(2*tsym);0.30*sin(10*tsym);1.42*sin(2*tsym)];
% x_func = [cos(tsym)-1 ; 0;tsym*0.2  ; 0;0;0];
%x_func = [tsym*0.2 ; 0;0  ; 0;0;0];
%x_func = [cos(tsym)-1 ; sin(tsym); tsym*0.2  ;-sin(tsym)*0.2;0 ;0];

% Camera param
Cam.freq = 20;
Cam.Width = 640;
Cam.Height = 480;
Cam.fx = 255; 
Cam.fy = 255; 
Cam.cx = 320;  
Cam.cy = 240;

% standard deviation
sigma_im = 11;               % pixel coord sigma , give it a small value ( not 0)
noise.sigma_im = sigma_im/Cam.fx;
sigma_gc  = 0.01 ;             % rot vel var  
sigma_ac  = 0.08 ;              % lin accel var  
sigma_wgc = 0.001 ;            % gyro bias change var 
sigma_wac = 0.001;           % accel bias change var
noise.sigma_gc = sigma_gc;
noise.sigma_ac = sigma_ac;
noise.sigma_wgc = sigma_wgc;
noise.sigma_wac = sigma_wac;


%% == parameter == %%
R_C_B = [1,0,0;
    0,1,0;
    0,0,1];
Bp_c0 = [0.0;0;0];

imuRate = 200; % [Hz]



checkerBoardParam.width = 5;
checkerBoardParam.height = 5;
checkerBoardParam.distance = 0.1;
checkerBoardParam.startPosition = [-0.2 -0.2];
checkerBoardParam.Z = 1;
checherBoard = createChecherBoard(checkerBoardParam.width,checkerBoardParam.height,...
                              checkerBoardParam.distance,checkerBoardParam.startPosition,checkerBoardParam.Z);






v_func = diff(x_func(1:3),tsym);
w_func = diff(x_func(4:6),tsym);
a_func = diff(v_func,tsym);

%max segments
maxSegments = floor(tf/(1/Cam.freq));
dt = 1/imuRate;
segmentLength = floor((tf/maxSegments)/dt);


%create a bunch of accelerations, and angular rates
t = 0:dt:tf;

%calculate T_iv (inertial to vicon reference)
x_ic = [0 ; 0 ; 0 ; 0 ; 0 ; 0];

%gravity
x_ig = [0; 0; -9.80665];

%starting state $6 pose 6 vel 6 bias
x_0 = zeros(18,1);

%push forward a dynamics model using these values to obtain inertial
%and camera poses
x = double(subs(x_func,tsym,t));
v = double(subs(v_func,tsym,t));
w = double(subs(w_func,tsym,t));
a = double(subs(a_func,tsym,t));

a(3,:) = a(3,:) - x_ig(3);


% Noise for a ba g bg
num = length(t);
b_a0 = 0.001*a(:,1);
b_a = repmat(b_a0,[1,num]);
n_ba = sigma_wac*randn(3,num);
b_a = b_a + n_ba;
n_a = sigma_ac*randn(3,num);

b_g0 = 0.001*w(:,1);
b_g = repmat(b_g0,[1,num]);
n_bg = sigma_wgc*randn(3,num);
b_g = b_g + n_bg;
n_g = sigma_gc*randn(3,num);


figure(1);
subplot(1,2,1);
plot_checherboard(checherBoard);
hold on;
view(-40,10);
axis equal;
xlabel('X');ylabel('Y');zlabel('Z');
SE3_plot([0;0;0;1],[0;0;0],0.01,0);

points = checherBoard(:,1:3)';
pointNum = size(checherBoard,1);
freq_ratio = imuRate /Cam.freq;
z=[];
Cam_z = [];
Cam_t = [];
Cam_pose = [];
for i = 1:size(t,2)
    R_I_W = CartToRot(x(4:6,i));
    w(:,i) = EulerRates2BodyRates( x(4:6,i), w(:,i));
    % For measuremment
    am = (R_I_W*a(:,i)) + n_a(:,i) + b_a(:,i);
    wm = w(:,i) + n_g(:,i) + b_g(:,i);
    
    z = [z ; am' wm'];
    
      % For camera 
    if mod(i,freq_ratio) == 0
        Cam_t = [Cam_t;
            t(i)];
        % Camera pose
        R_B_G = PQR2rotmat(x(4:6,i));
        R_C_G =R_C_B* R_B_G;
        q_C_G = rotMatToQuat(R_C_G);
        % camera position
        Gp_b0 = x(1:3,i);
        Gp_c0 = Gp_b0 + R_B_G'*Bp_c0;
        
        Cam_pose = [Cam_pose;Gp_c0' q_C_G'];
        % plot 
        subplot(1,2,1);
        SE3_plot(q_C_G,Gp_c0,0.1,0.1);
        
        % Project Points
        Cp_g0 = -R_C_G*Gp_c0;
        Cp_g0 = repmat(Cp_g0,[1,pointNum]);
        pointCam = R_C_G*points + Cp_g0;
        
        isFrontCam = pointCam(3,:) > 0; % Z axis > 0
        idealPoint(1,:) = (pointCam(1,:)./pointCam(3,:))';
        idealPoint(2,:) = (pointCam(2,:)./pointCam(3,:))';
        F = [Cam.fx,0;
            0,Cam.fy];
        Center = [Cam.cx;
                  Cam.cy];
        Center = repmat(Center,[1,pointNum]);
        Pixel = F*idealPoint + Center;
        
        % Apply noise
        n_im =  noise.sigma_im*randn(2,pointNum);
        Pixel =Pixel + n_im;
        
        isInWidthRange =  Pixel(1,:) < (Cam.Width-10) &  Pixel(1,:) >10; 
        isInHeightRange = Pixel(2,:) < (Cam.Height - 10) & Pixel(2,:) >10;
        isInRange = isInWidthRange & isInHeightRange & isFrontCam;
        index = find(isInRange == 1);
        observedPoints = Pixel(:,index);
        observedPoints = [observedPoints;
                          index];
        % Record ideal points
        Cam_z_i = -1*ones(2,pointNum);
        
        Cam_z_i(:,index) = Pixel(:,index);
             
        Cam_z = [Cam_z;
            Cam_z_i];
                      
        % Plot features
        subplot(1,2,2);
        im  = 255*ones(Cam.Height,Cam.Width);
        imshow(im);
        hold on;
        plot(observedPoints(1,:), observedPoints(2,:), 'r.');
        title('camera measurement');
        axis equal;
        hold off;
 
    end
end

%% ==================== PLOT =============== %%
figure(2);
subplot(1,2,1);
plot(t,z(:,1)','r',t,z(:,2)','g',t,z(:,3)','b');
title('accel');
subplot(1,2,2);
plot(t,z(:,4)','r',t,z(:,5)','g',t,z(:,6)','b');
title('gyro');

%% ===================SAVE DATA ====== %%
imu_output = [t' z];

%rpy to quaternion
% quat = [];
for k = 1:size(x,2);
    p = x(1:3,k);
    rpy = x(4:6,k);
    R = PQR2rotmat(rpy);
    q = rotMatToQuat(R);
    q = q/norm(q);
%     quat = [quat q];
    x(4:7,k) = q;
end

ground_truth = [t' x'];
ground_truth_vels = [t' v'];

% csvwrite('imu_data.csv',imu_poses);
% csvwrite('ground_truth.csv',ground_truth );
% csvwrite('ground_truth_vels.csv',ground_truth_vels );



save(['../dataset/' fileName],'R_C_B','Bp_c0', 'imu_output','ground_truth','ground_truth_vels','b_g', 'b_a','points','Cam_t','Cam_z','Cam_pose','Cam','noise');



