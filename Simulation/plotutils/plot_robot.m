
function h = plotrobot( pose, scale )

	if( nargin == 1 )
		scale = 1;
	end

	 position = pose(1:3);
	 % body triangle
	 pre_left = scale * [-0.10  0.10  0];
	 pre_right = scale * [-0.10 -0.10  0];
	 pre_tip = scale * [ 0.40  0     0];

	 % fin triangle
	 pre_fin_back  = scale * [ -0.1   0  0];
	 pre_fin_front = scale * [ 0.2   0  0];
	 pre_fin_tip   = scale * [ -0.1   0  -0.10];

	 R = eulerRPY_to_rotmat(pose(4:6)); % yaw, roll, pitch

	 tip = position + R * pre_tip';
	 left = position + R * pre_left';
	 right = position + R * pre_right';

	 fin_back = position + R * pre_fin_back';
	 fin_front = position + R * pre_fin_front';
	 fin_tip = position + R * pre_fin_tip';

	 %   verts = [ left'; right'; tip'; fin_back'; fin_front'; fin_tip' ];
	 verts = [ left'; right'; tip'; fin_back'; fin_front'; fin_tip' ];
	 faces = [ 1 2 3 ; 4 5 6];
	 face_colors = [ 1 0 0 ; 0.5 0.5 0.5 ];

	 %   h=patch(verts(:,1), verts(:,2), verts(:,3), 'b');
	 h=patch('Vertices',verts,'Faces',faces, 'FaceVertexCData',...
			 face_colors,'FaceColor','flat', 'BackFaceLighting', 'lit');

	 %   set(h,'FaceAlpha', 0.5');
