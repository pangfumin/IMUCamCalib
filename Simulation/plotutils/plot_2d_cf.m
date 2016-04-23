%
% h = plot_2d_cf( pose, scale ) 
%
function h = plot_2d_cf( pose, scale )
    if( nargin == 1 )
        scale = 1;
    end
    x = pose(1);
    y = pose(2);
    th = pose(3);

    R = scale * [  cos(th),  -sin(th); ...
                   sin(th),   cos(th) ];

    f = R(:,1);
    r = R(:,2);

    h(1) = line( [x; f(1)+x], [y; f(2)+y],  'Color', [0, 0.8, 0], 'LineWidth', 1 );
    h(2) = line( [x; r(1)+x], [y; r(2)+y],  'Color', 'r' , 'LineWidth', 1 );

