function Gc = calGc(q_I_G)
% Gc is 21 * 12
Gc = zeros(15,12);
C_I_G = quatToRotMat(q_I_G);

Gc(1:3,1:3) = -eye(3);
Gc(4:6,4:6) = eye(3);
Gc(7:9,7:9) = -C_I_G';
Gc(10:12,10:12) =eye(3);
Gc(13:15,4:6) = eye(3);

end