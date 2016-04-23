% h = plot_2d_gaussian( mu, sigma, col, nsigma )
function h = plot_2d_gaussian( mu, sigma, col, nsigma )

	if( nargin == 2 )
		col = 'r';
		nsigma = 1;
	elseif( nargin == 3 )
		nsigma = 1;
	end

	pts = 30;
	[x, y, z] = cylinder(1,pts);
	circle = [x(1, :); y(1, :)];
    [V, D] = eig( (sigma + sigma.')/2 );
	PC = V*nsigma*D.^(1/2);  % principle components
	ellipse = real( PC * circle + repmat(mu, 1, pts+1) );

    hold on;
    axis equal;
	h = plot(ellipse(1,:)', ellipse(2,:)', col);
	h = [ h; plot( [mu(1)-PC(1,1); ...
             PC(1,1)+mu(1)], [mu(2)-PC(2,1); PC(2,1)+mu(2)], col )];
	h = [ h; plot( [mu(1)-PC(1,2); ...
             PC(1,2)+mu(1)], [mu(2)-PC(2,2); PC(2,2)+mu(2)], col )];

