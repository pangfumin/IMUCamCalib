% draw a robot trajectory
% rpath is a 6xN vector of 6D poses
function h = plot_simple_path( rpath, scale )

    if( nargin == 1 )
        scale = 1;
    end

    hold on; 
    light;
    axis equal;
    grid on;
    set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );  % NASA convention

    xlabel('x');
    ylabel('y');
    zlabel('z');

    h = [];
    for ii = 1:size(rpath,2)
        h = [h plot_simple_cf( rpath(:,ii), scale )];
    end

    h = [h plot3( rpath(1,:), rpath(2,:), rpath(3,:), 'm-' ) ];
    h = [h plot3( rpath(1,1), rpath(2,1), rpath(3,1), 'm.' ) ];
    h = [h plot3( rpath(1,end), rpath(2,end), rpath(3,end), 'm.' ) ];