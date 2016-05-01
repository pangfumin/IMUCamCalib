function ekfState_up = updateState(ekfState, deltaX)


    % Initialize updated state with current state
    ekfState_up = ekfState;

    % Update IMU State
    deltatheta_I_G = deltaX(1:3);
    deltabg = deltaX(4:6);
    deltav_I_G = deltaX(7:9);
    deltaba = deltaX(10:12);
    deltap_I_G = deltaX(13:15);
    deltatheta_I_C = deltaX(16:18);
    deltap_C_I = deltaX(19:21);
    
    
    deltaq_I_G = buildUpdateQuat(deltatheta_I_G);
    ekfState_up.q_I_G = quatLeftComp(deltaq_I_G) * ekfState.q_I_G;
    ekfState_up.q_I_G = ekfState_up.q_I_G/norm(ekfState_up.q_I_G);
    
    ekfState_up.bg = ekfState.bg + deltabg;
    ekfState_up.v_I_G = ekfState.v_I_G + deltav_I_G;
    ekfState_up.ba = ekfState.ba + deltaba;
    ekfState_up.p_I_G = ekfState.p_I_G + deltap_I_G;
    
    deltaq_I_C = buildUpdateQuat(deltatheta_I_C);
    ekfState_up.q_I_C = quatLeftComp(deltaq_I_C) * ekfState.q_I_C;
    ekfState_up.q_I_C = ekfState_up.q_I_C/norm(ekfState_up.q_I_C);
    
    ekfState_up.p_C_I= ekfState.p_C_I + deltap_C_I;
    
   
    
    
    
    
end