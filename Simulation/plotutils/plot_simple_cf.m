%
% h = plot_cf( pose, scale ) 
%	
%   Draw an origin indicator at 'pose'.  Pose is represented by a six vector,
%   <x,y,z,r,p,q>, composed of a 3D point, <x,y,z>, and an Euler
%   roll-pitch-yaw angle <r,p,q>. Angles are positive counterclockwise. The
%   orientation r=0,p=0,q=0 is +x forward, +y right and +z down. This is a
%   standard aeronautical right-handed convention.
%
%   The optional scale parameter scales the graphics object. 
%
function h = plot_simple_cf( pose, scale )
    if( nargin == 1 )
		scale = 1;
	end

	R = scale*eulerPQR_to_rotmat( pose(4:6) );

    h(1) = line( [0; R(1,1)]+pose(1), [0; R(2,1)]+pose(2), [0; R(3,1)]+pose(3), 'Color', 'r' );
    h(2) = line( [0; R(1,2)]+pose(1), [0; R(2,2)]+pose(2), [0; R(3,2)]+pose(3), 'Color', 'g' ); 
    h(3) = line( [0; R(1,3)]+pose(1), [0; R(2,3)]+pose(2), [0; R(3,3)]+pose(3), 'Color', 'b' );

