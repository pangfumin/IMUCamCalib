function Fc = calFc(q_I_G , wHat ,aHat)
% Fc is 21*21
Fc = zeros(15, 15);
C_I_G = quatToRotMat(q_I_G);

Fc(1:3,1:3) = -crossMat(wHat);
Fc(1:3,4:6) = -eye(3);

Fc(7:9,1:3) = -C_I_G'*crossMat(aHat);
Fc(7:9,10:12) = -C_I_G';

Fc(13:15,7:9) = eye(3);
end