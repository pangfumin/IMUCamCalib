%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% omega = EulerRates2BodyRates( pqr, dEdt )
% 
% Convert Euler-rates, dEdt, (time derivative of roll, pitch, yaw) into 
% body rates. 
%
% pqr     is the roll-pitch-yaw zyx-Euler angle defining Rnb, the rotation
%         of the body-frame in the navigation-frame.  This is important.
%
% dEdt    is the time derivative of the zyx-Euler angles.
%
% omega   is the body-rate angular velocity.
%
% The best derivation I've found for this transform is the short MS thesis
% "Gyroscope Calibration and Dead Reconing for an Autonomous Underwater
% Vehicle" by Aaron J Kapaldo.  I'm still looking for a good textbook
% reference.  NB: the rotation matrices A. Kapaldo computes are transposed
% zyx-Euler convention (which is pretty standard).
%
function omega = EulerRates2BodyRates( pqr, dEdt )

    sp = sin(pqr(1)); % roll phi
    cp = cos(pqr(1)); % roll
    sq = sin(pqr(2)); % pitch
    cq = cos(pqr(2)); % pitch

    T = [ 1,   0,   -sq;...
          0,  cp, cq*sp;...
          0, -sp, cq*cp];

    omega = T*dEdt;

% Checking:
if 0
    syms p q r dp dq dr;

    pqr = [p;q;r];
    dEdT = [dp; dq; dr];

    sp = sin(pqr(1)); % roll phi
    cp = cos(pqr(1)); % roll
    sq = sin(pqr(2)); % pitch
    cq = cos(pqr(2)); % pitch
    sr = sin(pqr(3)); % pitch
    cr = cos(pqr(3)); % pitch
    
    Rxt = [  1     0    0 ;...
            0    cp   sp ; ...
            0   -sp   cp];

    Ryt = [  cq   0  -sq  ; ...
             0   1    0  ; ...
            sq   0   cq ];

    Rzt = [  cr    sr    0; ...
           -sr    cr    0; ...
             0     0    1];

% NB: The given orientation, Rnb = R(pqr), is "body-frame" in the
% "nav-frame".  To match A Kapaldo's derivation we will be using 
% Rbn = Rnb' = [RzRyRx]' = [Rx'Ry'Rz'] and hence the rotation matrices we
% compute here are Rx', Ry' and Rz' -- e.g. when compared to my Cart2R
% functions and standard zyx-Euler convetion. Basically, Kapaldo's "Rx" is
% really Rx'.

    omega =  [ dEdt(1);0;0] + Rxt*[ 0; dEdt(2); 0] + Rxt*Ryt*[ 0;0;dEdt(3) ]
    
    T = [ 1,   0,   -sq;...
          0,  cp, cq*sp;...
          0, -sp, cq*cp];
     
    omega = T*dEdt
end

end

