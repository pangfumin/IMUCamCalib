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
R_B_G = quatToRotMat(ekfState.q_I_G);
aG = R_B_G'*aI + gG;
dR_cur_last = quatToRotMat(dq);
s = dt*0.5*(dR_cur_last'*aI + last_aI);
y = dt*0.5*s;
ekfState_pro.v_I_G = ekfState.v_I_G + R_B_G'*s+dt*gG;
ekfState_pro.p_I_G = ekfState.p_I_G +ekfState.v_I_G*dt+ R_B_G'*y + 0.5*gG*dt*dt;

% bias
ekfState_pro.bg = ekfState.bg;
ekfState_pro.ba = ekfState.ba;


%   covariance propagetion
Fc = calFc(ekfState.q_I_G,last_wI,last_aI);
Gc = calGc(ekfState.q_I_G);

phi = eye(15) + Fc*dt;

Qc = eye(12);
Qc(1:3,1:3) = noiseParams.sigma_gc*noiseParams.sigma_gc*eye(3);
Qc(4:6,4:6) = noiseParams.sigma_wgc*noiseParams.sigma_wgc*eye(3);
Qc(7:9,7:9) = noiseParams.sigma_ac*noiseParams.sigma_ac*eye(3);
Qc(10:12,10:12) = noiseParams.sigma_wac*noiseParams.sigma_wac*eye(3);

Nc = Gc*Qc*Gc';  % shelley04 master thesis
Qd = dt*Nc;%0.5*dt*phi*Nc*phi'+Nc;

P = phi*ekfState.P*phi' + Qd;


ekfState_pro.P = enforcePSD(P);

end