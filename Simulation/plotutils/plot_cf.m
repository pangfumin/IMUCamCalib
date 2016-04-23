%
% h = plot_cf( pose, scale ) 
%	
%   Draws an origin indicator at pose.  Pose is represented by a six vector,
%   <x,y,z,r,p,q>, composed of a 3D point, <x,y,z>, and an Euler
%   roll-pitch-yaw angle <r,p,q>. Angles are positive counterclockwise. The
%   orientation r=0,p=0,q=0 is +x forward, +y right and +z down. This is a
%   standard aeronautical right-handed convention.
%
%   The optional scale parameter scales the graphics object. 

function  h = plot_cf( pose, scale )

	if( nargin == 1 )
		scale = 1;
	end

	R = eulerPQR_to_rotmat( pose(4:6) );
	N = 8;

	[x,y,z] = cylinder( 0.05, N-1 );
   	x = [x; x(:,end) x(:,1:end-1)];
	y = [y; y(:,end) y(:,1:end-1)];
	z = [z; z([2,1],end) z([2,1],1:end-1)];
	pts = [ reshape(x, 1, 4*N); reshape(y, 1, 4*N); reshape(z, 1, 4*N) ]*scale;

	x = x.*1.6;
	x([2,3],:) = 0;
	y = y.*1.6;
	y([2,3],:) = 0;
	z = z*0.4 + 1;
	conepts = [ reshape(x, 1, size(x,1)*size(x,2)); ...
                reshape(y, 1, size(y,1)*size(y,2)); ...
                reshape(z, 1, size(z,1)*size(z,2)) ]*scale;

	% rotate
	xpts = R*[ 0 0 1; 0 1 0; -1 0 0]*[pts conepts] + repmat( pose(1:3,1), 1, 8*N);
	Xx = reshape( xpts(1,:), size(x,1), 2*size(x,2) );
	Xy = reshape( xpts(2,:), size(x,1), 2*size(x,2) );
	Xz = reshape( xpts(3,:), size(x,1), 2*size(x,2) );

	ypts = R*[ 1 0 0; 0 0 1; 0 -1 0]*[pts conepts] + repmat( pose(1:3,1), 1, 8*N);
	Yx = reshape( ypts(1,:), size(x,1), 2*size(x,2));
	Yy = reshape( ypts(2,:), size(y,1), 2*size(y,2));
	Yz = reshape( ypts(3,:), size(z,1), 2*size(z,2));

	zpts = R*[pts conepts] + repmat( pose(1:3,1), 1, 8*N);
	Zx = reshape( zpts(1,:), size(x,1), 2*size(x,2));
	Zy = reshape( zpts(2,:), size(y,1), 2*size(y,2));
	Zz = reshape( zpts(3,:), size(z,1), 2*size(z,2));

    m = size([Xx,Yx,Xz],1);
	n = size([Xx,Yx,Xz],2);
	C = repmat( 1, [m, n, 3]);  % strange, but 'patch' breaks without this line
	C(1,1:n,1:3) = [repmat([1 0 0], n/3,1);repmat([0 1 0],n/3,1);repmat([0 0 1], n/3,1)];
	C(2,1:n,1:3) = [repmat([1 0 0], n/3,1); repmat([0 1 0], n/3,1); repmat([0 0 1], n/3,1)];
	C(3,1:n,1:3) = [repmat([1 0 0], n/3,1); repmat([0 1 0], n/3,1); repmat([0 0 1], n/3,1)];
	h = patch([Xx Yx Zx],[Xy Yy Zy],[Xz Yz Zz],C,... 
            'FaceLighting','flat','LineStyle','none','BackFaceLighting',...
            'reverselit');

%    x=[Xx Yx Zx]
%    y=[Xy Yy Zy]
%    z=[Xz Yz Zz]
%    C

 %   h = patch(Xx,Xy,Xz,'red', 'LineStyle','none', 'FaceLighting','flat',...
  %          'BackFaceLighting', 'reverselit');
  %  h = patch(Yx,Yy,Yz,'green', 'LineStyle','none', 'FaceLighting','flat',...
  %          'BackFaceLighting', 'reverselit');
  %  h = patch(Zx,Zy,Zz,'blue','LineStyle','none', 'FaceLighting','phong',...
%            'BackFaceLighting', 'reverselit');

