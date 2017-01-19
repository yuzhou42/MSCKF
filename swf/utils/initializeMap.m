function rho_i_pj_i_est = initializeMap(imuStates, y_k_j, y_var, ...
    calibParams, vehicleCamTransform, k1, k2)

    % Initialize the map
    rho_i_pj_i_est = nan(3,size(y_k_j,3));

    % Repackage stuff for calcGNPosEst
    noiseParams.u_var_prime = y_var(1)/calibParams.f_u^2;
    noiseParams.v_var_prime = y_var(2)/calibParams.f_v^2;
    
    % Get cam states from IMU state
    for i = 1:size(imuStates, 2)
        camStates{i}.p_C_G = imuStates{i}.C_vi' * vehicleCamTransform.rho_cv_v + imuStates{i}.r_vi_i;
        camStates{i}.q_CG  = rotMatToQuat(vehicleCamTransform.C_cv * imuStates{i}.C_vi);
    end
    
    % Make the map
    for j = 1:size(y_k_j, 3)
        observations = squeeze(y_k_j(1:2,k1:k2,j));
        
        % Pick out camera states with valid observations of the current
        % feature
        validCamStates = {};
        for i = 1:size(observations,2)
            if observations(1,i) ~= -1
                validCamStates{end+1} = camStates{i};
            end
        end
        
        % Estimate the current feature position
        if size(validCamStates, 2) >= 2
            [rho_i_pj_i_est(:,j), Jnew] = calcGNPosEst(camStates, observations, noiseParams);
        end
    end
    
end