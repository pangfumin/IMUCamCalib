% function h = plot_3d_gaussian( mu, sig, scale, col, alpha )
%
%   Plot a 3d Gaussian,  scale, col and alpha are optional.
%


function h = plot_3d_gaussian(mu, sig, scale, col, alpha )

	if( nargin == 2 )
		scale = 1;
		col = 'g';
		alpha = 0;
	elseif( nargin == 3 )
		col = 'g';
		alpha = 0;	
	elseif( nargin == 4 )
		alpha = 0;
	end

	[x, y, z] = sphere(8);
	[szy szx] = size(x);
	c = [reshape(x, [1, szy * szx]); reshape(y, [1, szy * szx]); reshape(z, [1, szy * szx])];
	[V, D] = eig( 0.5*(sig+sig.'));
	D=abs(D);
	C = real(V * D^(1/2) * scale * c);
	CX = reshape(C(1, :), [szy, szx]);
	CY = reshape(C(2, :), [szy, szx]);
	CZ = reshape(C(3, :), [szy, szx]);
	s = surf(mu(1) + CX, mu(2)+CY, mu(3)+CZ );

	d = det(sig);
	if( alpha )
		if( d == 0)
			blend = 0;
		else
			blend = max( 0.1, min(1, scale * alpha/d)); % determinant is a measure of volume
			if( d < 0)
				warning('Determinant of covariance matrix is negative (implying a reflection).');
				blend = 1;
				col = 'm';
			end
		end
		h = patch( surf2patch( s ), 'EdgeColor', 'none', 'FaceLighting', 'phong',...
				'FaceColor',col, 'FaceAlpha', blend );
	else
		h = patch( surf2patch( s ), 'EdgeColor', 'none', 'FaceLighting', 'phong',...
				'FaceColor',col );
	end
	delete( s );

