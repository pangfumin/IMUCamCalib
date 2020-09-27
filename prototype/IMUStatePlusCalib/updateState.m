function ekfState_up = updateState(ekfState, deltaX)
    % Initialize updated state with current state
    ekfState_up = ekfState;

    % Update IMU State
    deltatheta_I_G = deltaX(1:3);
    deltap_I_G = deltaX(4:6);
    deltav_I_G = deltaX(7:9);
    deltabg = deltaX(10:12);
    deltaba = deltaX(13:15);
    deltatheta_C_I = deltaX(16:18);
    deltap_I_C = deltaX(19:21);
  
    % q
    deltaq_I_G = buildUpdateQuat(deltatheta_I_G);
%     ekfState_up.q_I_G = quatLeftComp(deltaq_I_G) * ekfState.q_I_G;
    ekfState_up.q_I_G = quatMult(ekfState.q_I_G, deltaq_I_G);
    ekfState_up.q_I_G = ekfState_up.q_I_G/norm(ekfState_up.q_I_G);
    
    % p
    ekfState_up.p_I_G = ekfState.p_I_G + deltap_I_G;
    
    % v
    ekfState_up.v_I_G = ekfState.v_I_G + deltav_I_G;
    
    % bias
    ekfState_up.bg = ekfState.bg + deltabg;
    ekfState_up.ba = ekfState.ba + deltaba;
    
    % extrinsic
    deltaq_C_I = buildUpdateQuat(deltatheta_C_I);
    ekfState_up.q_C_I = quatLeftComp(deltaq_C_I) * ekfState.q_C_I;
    ekfState_up.q_C_I = ekfState_up.q_C_I/norm(ekfState_up.q_C_I);
    
    ekfState_up.p_I_C = ekfState.p_I_C + deltap_I_C;
    

end