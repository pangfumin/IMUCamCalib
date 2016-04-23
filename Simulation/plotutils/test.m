% test plotting routines. requries MLV kinematics toolbox

hold on; 
light;
axis equal;
grid on;
set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );  % NASA convention
xlim([-0.05,0.3]);
ylim([-0.15,0.2]);
zlim([-0.15,0.15]);
xlabel('x');
ylabel('y');
zlabel('z');
text( 0, 0, -0.1, 'press any key...');
view( 45, 15 );

P = diag([220,1,1]);
pose = zeros(6,1);
cfpose = pose+[0;0;0.05;0;0;0];
nposes = 15;
dpose = [0;0.05;0;0;0; -2*pi/nposes]; % delta pose

plot_robot( pose, 0.1 );
fprintf('drew a robot... ish... thing\npress a button to continue...');

waitforbuttonpress;

plot_camera( 1,1, pose, 2 ); % demonstrats scale factor

h = [ plot_camera( 1,1, pose )...
      plot_cf( cfpose, 0.03 ) ...
	  plot_3d_gaussian(cfpose(1:3), P, 0.005, 'm' ) ];
h = [ h plot_camera( 1,1, pose )...
        plot_cf( cfpose, 0.03 ) ...
		plot_3d_gaussian(cfpose(1:3), P, 0.005, 'm') ];

for( t=1:nposes )
	h = [ h plot_camera( 1,1, pose )...
	        plot_cf( cfpose, 0.03 ) ...
			plot_3d_gaussian(cfpose(1:3), P, 0.005, 'm', 1 ) ];
	pose = compound_op( pose, dpose );
	cfpose = pose+[0;0;0.05;0;0;0];
	J = jac_compound_op( pose, dpose ); % used to project covariance
	P = J(4:6,4:6).'*P*J(4:6,4:6); % select right terms
	waitforbuttonpress;
	delete(h(1:3));
	h(1:3) = [];
end


