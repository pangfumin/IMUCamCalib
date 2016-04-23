% note that I use the Roll-Pitch-Yaw convention,
% where p is roll, q is pitch and r is yaw
% e.g. attitude = (roll,pitch,yaw) = (p,q,r)

% This is rotation about the fixed frame. see eqn 2.20 of Modelling and Control of Robot Manip.

%  roll  about x axis, sinze +x is forward
%  pitch about y axis, since +y is right
%  yaw   about x axis, sinze +z is down

function R = CartToRot( attitude )

%    if numel(attitude) ~= 3
%        error('ERROR: attitude vector must be 3x1 -- did you pass a 6x1 pose vector?');
%    end

%    attitude = angle_wrap( attitude );

%  verify
%    syms p q r;
%    attitude = [p,q,r];
    
    cp = cos(attitude(1));
    cq = cos(attitude(2));
    cr = cos(attitude(3));

    sp = sin(attitude(1));
    sq = sin(attitude(2));
    sr = sin(attitude(3));   

  
% same thing as:
   Rz = [  cr    sr    0; ...
           -sr    cr    0; ...
            0     0    1];

   Ry = [  cq   0   -sq  ; ...
            0   1    0  ; ...
           sq   0   cq ];

   Rx = [  1     0    0 ;...
           0    cp   sp ; ...
           0    -sp   cp ];

   R = Rz*Ry*Rx;


