function [updatedStates, rho_i_pj_i_est] = updateStateStruct( currentStates, lmIds, rho_i_pj_i_est, dx)
%UPDATESTATESTRUCT Updates states by applying a step dx

updatedStates = currentStates;
numStates = length(currentStates);
for stIdx = 1:numStates-1
    dr = dx(1+(stIdx-1)*6:3+(stIdx-1)*6);
    phi = dx(4+(stIdx-1)*6:6+(stIdx-1)*6);
    
    updatedStates{stIdx+1}.r_vi_i = currentStates{stIdx+1}.r_vi_i + dr;
    updatedStates{stIdx+1}.C_vi = Cfrompsi(phi)*currentStates{stIdx+1}.C_vi;
end

initialIdx = (numStates-1)*6 + 1;
for lm_i = 1:length(lmIds)
    idx = initialIdx + (lm_i-1)*3;
    dp = dx(idx:idx+2);
    rho_i_pj_i_est(:, lmIds(lm_i)) = rho_i_pj_i_est(:, lmIds(lm_i)) + dp;
end


end

