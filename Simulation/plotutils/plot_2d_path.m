% draw a robot trajectory
% rpath is a 3xN vector of 2D poses
function h = plot_2d_path( rpath, scale )

if( nargin == 1 )
    scale = 1;
    end

    hold on; 
    axis equal;
    grid on;

    xlabel('x');
    ylabel('y');

    h = [];
    for ii = 1:size(rpath,2)
        h = [ h,  plot_2d_cf( rpath(:,ii), scale ) ];
    end


