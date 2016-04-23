
function plot_path( rpath, scale )

    if( nargin == 1 )
        scale = 1;
    end

    hold on; 
    light;
    axis equal;
    grid on;
    set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );  % NASA convention

%    xlim([-0.05,0.3]);
%    ylim([-0.15,0.2]);
%    zlim([-0.15,0.15]);
    xlabel('x');
    ylabel('y');
    zlabel('z');
%    view( 45, 15 );

    for ii = 1:size(rpath,2)
        plot_cf( rpath(:,ii), scale );
    end

