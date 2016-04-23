% function h = plot_plane( normal, base, side, color, alpha )
%
function h = plot_plane( normal, base, side, color, alpha)

	if( nargin == 2 )
        side = 1;
		color = [ 0 0 1 ];	
		alpha = 0.5;
    elseif( nargin == 3 )
		color = [ 0 0 1 ];	
		alpha = 0.5;
	elseif( nargin == 4 )
		alpha = 0.5;
	end

    if( size(color,1) ~= 1|| size(color,2) ~= 3 )
        error 'Color must be a 1x3 vector'
    end

    f = normalize( normal);  % "forward"
    % figure out how to draw a plane patch in some known ground truth plane
    dx = abs(dot([1;0;0],f));
    dy = abs(dot([0;1;0],f));
    dz = abs(dot([0;0;1],f));

    if( dz > 0.001 ) % normal is pointing up-ish, cross with "fowrard" first
        r = normalize(cross([1;0;0 ], f(1:3))); % "right"
        d = normalize(cross( f, r ));  % down
        R = [ r d f ];
     else % 
        r = normalize(cross( [0;0;1], f(1:3) ));
        d = normalize(cross( f, r ));
        R = [ r d f ];
    end

    p0 = R * [  side/2  side/2 0 ].' + base;
    p1 = R * [  side/2 -side/2 0 ].' + base;
    p2 = R * [ -side/2  side/2 0 ].' + base;
    p3 = R * [ -side/2 -side/2 0 ].' + base;

    verts = [p0 p1 p2 p3].';
    faces = [ 1 2 3 ; 2 4 3 ];
    face_colors = [ color ; color ];

    h = line( [base(1);f(1)], [base(2);f(2)], [base(3);f(3)], 'Color', 'r' );
    h = [h patch('Vertices',verts,'Faces',faces, 'EdgeColor', 'none', 'FaceVertexCData', face_colors,...
         'FaceColor','flat', 'BackFaceLighting', 'lit', 'FaceAlpha', alpha) ];

function n = normalize( n )
    n = n / norm( n );


function test1
    hold on;
    grid on;
    axis equal;
    xlim([-10,10]);
    ylim([-10,10]);
    zlim([-10,10]);
    view([-30,30]);
    set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' ); % goto NASA frame
    n = [1;0;0];
    p = [0;0;0];
    for( th = 0:0.03:100 )
        R = Cart2R( [th;2*th;3*th] );
        h = plot_plane( R*n, p, 10 );
        waitforbuttonpress;
        delete(h); 
    end

