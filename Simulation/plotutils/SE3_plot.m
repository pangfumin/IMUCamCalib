function SE3_plot(q,p,length,dt)


R = quatToRotMat(q);
% generate axis vectors
tx = [length,0.0,0.0];
ty = [0.0,length,0.0];
tz = [0.0,0.0,length];
% Rotate it by R
t_x_new = R'*tx';
t_y_new = R'*ty';
t_z_new = R'*tz';


% translate vectors to camera position. Make the vectors for plotting
origin=[p(1),p(2),p(3)];
tx_vec(1,1:3) = origin;
tx_vec(2,:) = t_x_new + origin';
ty_vec(1,1:3) = origin;
ty_vec(2,:) = t_y_new + origin';
tz_vec(1,1:3) = origin;
tz_vec(2,:) = t_z_new + origin';
hold on;

% Plot the direction vectors at the point
p1=plot3(tx_vec(:,1), tx_vec(:,2), tx_vec(:,3));
set(p1,'Color','Green','LineWidth',1);
p1=plot3(ty_vec(:,1), ty_vec(:,2), ty_vec(:,3));
set(p1,'Color','Blue','LineWidth',1);
p1=plot3(tz_vec(:,1), tz_vec(:,2), tz_vec(:,3));
set(p1,'Color','Red','LineWidth',1);
drawnow;
pause(dt);
    
    
end