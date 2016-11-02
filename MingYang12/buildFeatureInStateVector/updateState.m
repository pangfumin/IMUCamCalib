function ekfState_up = updateState(ekfState, deltaX)
    % Initialize updated state with current state
    ekfState_up = ekfState;

    % Update IMU State
    deltatheta_I_G = deltaX(1:3);
    deltap_I_G = deltaX(4:6);
    deltav_I_G = deltaX(7:9);
    deltabg = deltaX(10:12);
    deltaba = deltaX(13:15);
  
    % q
    deltaq_I_G = buildUpdateQuat(deltatheta_I_G);
    ekfState_up.q_I_G = quatLeftComp(deltaq_I_G) * ekfState.q_I_G;
    ekfState_up.q_I_G = ekfState_up.q_I_G/norm(ekfState_up.q_I_G);
    
    % p
    ekfState_up.p_I_G = ekfState.p_I_G + deltap_I_G;
    
    % v
    ekfState_up.v_I_G = ekfState.v_I_G + deltav_I_G;
    
    % bias
    ekfState_up.bg = ekfState.bg + deltabg;
    ekfState_up.ba = ekfState.ba + deltaba;
    
    for i = 1:length(ekfState.featureState)/3
        pStart = 3*(i-1)+1;
        pEnd = pStart + 2;
        deltapf = deltaX(15+pStart:15+pEnd);
        ekfState_up.featureState(pStart:pEnd) = ekfState.featureState(pStart:pEnd) + deltapf;
        
    end
    
    
end