function ekfState_pro = ProapagateIMUStateAndCovar(ekfState ,cur_wm,cur_am,last_wm,last_am, dt,noiseParams)

%% ===========================propagate state================== %%
gG = [0;0;-9.81];
aI = cur_am - ekfState.ba;
wI = cur_wm - ekfState.bg;

last_aI = last_am - ekfState.ba;
last_wI = last_wm - ekfState.bg;

% rotation
q0 = [0;0;0;1];
OmegaMat1 = omegaMat(last_wI);
OmegaMat2 = omegaMat((last_wI + wI)*0.5);
OmegaMat3 = omegaMat(wI);
k1 = 0.5*OmegaMat1*q0;
k2 = 0.5*OmegaMat2*(q0+dt*0.5*k1);
k3 = 0.5*OmegaMat2*(q0+dt*0.5*k2);
k4 = 0.5*OmegaMat3*(q0+dt*k3);

dq = q0 +dt*(k1+2*k2+2*k3+k4)/6.0;
dq = dq/norm(dq);

ekfState_pro.q_I_G = quatMult(dq,ekfState.q_I_G);
ekfState_pro.q_I_G = ekfState_pro.q_I_G/norm(ekfState_pro.q_I_G);
% v and p
R_I_G = quatToRotMat(ekfState.q_I_G);
aG = R_I_G'*aI + gG;
dR_cur_last = quatToRotMat(dq);
s = dt*0.5*(dR_cur_last'*aI + last_aI);
y = dt*0.5*s;
ekfState_pro.v_I_G = ekfState.v_I_G + R_I_G'*s+dt*gG;
ekfState_pro.p_I_G = ekfState.p_I_G +ekfState.v_I_G*dt+ R_I_G'*y + 0.5*gG*dt*dt;

% bias
ekfState_pro.bg = ekfState.bg;
ekfState_pro.ba = ekfState.ba;


%% ============= Propagte error and covar ================== %%
% according to Shelly's Master Thesis Appendix B
R_lastT = R_I_G';
R_T = quatToRotMat(ekfState.q_I_G)';
Rsum = (R_lastT + R_T);
Pqq = eye(3);
Ppq = -crossMat(R_lastT*y);
Pvq = -crossMat(R_lastT*s);

% Ppq = -crossMat(ekfState_pro.p_I_G - ekfState.p_I_G - ekfState.v_I_G*dt - 0.5*gG*dt*dt);
% Pvq = -crossMat(ekfState_pro.v_I_G - ekfState.v_I_G - gG*dt);

Pqbg = -0.5*dt*(Rsum);
Pvbg = 0.25*dt*dt*crossMat(aG - gG)*Rsum;
Ppbg = 0.5*dt*Pvbg;

Pqba = zeros(3,3);
Pvba = -0.5*dt*Rsum;
Ppba = 0.5*dt*Pvba;

P = eye(15);
P(1:3,1:3) = Pqq;
P(1:3,10:12) = Pqbg;
P(1:3,13:15) = Pqba;

P(4:6,1:3) = Ppq;
P(4:6,4:6) = eye(3);
P(4:6,7:9) = eye(3)*dt;
P(4:6,10:12) = Ppbg;
P(4:6,13:15) = Ppba;

P(7:9,1:3) = Pvq;
P(7:9,10:12) = Pvbg;
P(7:9,13:15) = Pvba;

Nc = zeros(15);
Nc(1:3,1:3) = noiseParams.sigma_gc*noiseParams.sigma_gc*eye(3);
Nc(7:9,7:9) = noiseParams.sigma_ac*noiseParams.sigma_ac*eye(3);
Nc(10:12,10:12) = noiseParams.sigma_wgc*noiseParams.sigma_wgc*eye(3);
Nc(13:15,13:15) = noiseParams.sigma_wac*noiseParams.sigma_wac*eye(3);
Qd = 0.5*dt*P*Nc*P'+Nc;

ekfState_pro.Covar = P*ekfState.Covar*P'+Qd;
 % Enforce PSD-ness
ekfState_pro.Covar = enforcePSD(ekfState_pro.Covar);

end