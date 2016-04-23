function mvl3d

    hold on;
    grid on;
    axis equal;
    axis vis3d;

    set( gca, 'ZDir', 'reverse', 'YDir', 'reverse' );  % NASA convention

    view(3);

    xlabel('x');
    ylabel('y');
    zlabel('z');

    camlight;
