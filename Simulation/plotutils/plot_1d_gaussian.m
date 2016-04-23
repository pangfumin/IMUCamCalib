% h = plot_1d_gaussian( mu )
%   Plot gaussian centered at mu with standard deviation 1
%
% h = plot_1d_gaussian( mu, sig )
%   Plot gaussian centered at mu with standard deviation sig
%
% h = plot_1d_gaussian( mu, sig, linestyle )
%   Plot gaussian centered at mu with standard deviation sig and a linestyle 
%
% h = plot_1d_gaussian( mu, sig, linestyle, scale )
%   Plot scaled gaussian centered at mu with standard deviation sig, a linestyle
%

function h = plot_1d_gaussian( mu, sig, col, scale )

	if( nargin == 1 )
		sig = 1;
		scale = 1;
		col = 'b';
	elseif( nargin == 2 )
		col = 'b';
		scale = 1;
	elseif( nargin == 3 )
		scale = 1;
	end

	npoints = 100;
	parameter = 3.5;
	low = mu - parameter*sig;
	high = mu + parameter* sig;
	dx = (high - low) / npoints;
	x=low:dx:high;
	fx = scale*(1/(sig * sqrt(2*pi)) * exp(-1/2 * ((x-mu)/sig).^2 ));

	h = plot( x, fx, col );

function h = plot_1d_gaussian_from_data(x)

	sig2    = std(x,1)^2;
	u       = mean(x);
	N       = length(x);

	[n,x_c] = hist( x,100 );
	n       = n / sum(n*abs(x_c(2)-x_c(1)));
	y       = sqrt(1/2/pi/sig2)*exp(-((x_c-u).^2)/(2*sig2));

	plot(x_c,y,'-');

%if 0
%function h = plot_1d_gaussian(mu,sig,scale,col)

%  npoints = 100;
%  parameter = 3.5;
%  low = mu - parameter*sig;
%  high = mu + parameter* sig;
%  interval = (high - low) / npoints;

%  x=low:interval:high;

%  fx = 1/(sig * sqrt(2*pi)) * exp(-1/2 * ((x-mu)/sig).^2 );

  % normalize, scale
%  fx = scale*(fx/sum(fx));
%  sum(fx)
  
  % Plot the random variable x versus the PDF function
%  plot(x,fx,'Color',col)
%  xlabel('Random variable - x')
%  ylabel('f(x)')
%  grid
%end
