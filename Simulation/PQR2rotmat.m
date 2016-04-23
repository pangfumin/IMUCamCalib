function R = PQR2rotmat(attitude)

% WARNING this derviation uses "rpq" instead of the more standard "pqr" to
% represent roll,pitch,yaw.  The above has been fixed.

%    attitude = angle_wrap( attitude );

%  verify
%    syms p q r;
%    attitude = [p,q,r];

    cp = cos( attitude(1) );
    cq = cos( attitude(2) );
    cr = cos( attitude(3) );

    sp = sin( attitude(1) );
    sq = sin( attitude(2) );
    sr = sin( attitude(3) );

    % check roll, p:
    %     Rx(pi/2)*[0;1;0] = [0;   0; 1]
    %     Rx(pi/2)*[0;0;1] = [0;  -1; 0]
    Rx = [  1     0    0 ;...
            0    cp  sp ; ...
            0    -sp   cp];

    % check pitch, q:
    %     Ry(pi/2)*[1;0;0] = [ 0;  0; -1]
    %     Ry(pi/2)*[0;0;1] = [ 1;  0;  0]
    Ry = [  cq   0   -sq  ; ...
             0   1    0  ; ...
           sq   0   cq ];

    % check yaw, r:
    %     Rz(pi/2)*[1;0;0] = [ 0;  1; 0]
    %     Rz(pi/2)*[0;1;0] = [-1;  0; 0]
    Rz = [  cr   sr    0; ...
            -sr    cr    0; ...
             0     0    1];

    % ZYX order: roll, pitch, yaw
    R = Rz*Ry*Rx;