function avgNEES = ANEES(trans_err, rot_err, err_sigma)

    stateErr = [rot_err;trans_err];
    stateVar = err_sigma.^2;
    avgNEES = 0;
    stepNum = size(stateErr, 2);
    for i = 1:stepNum
        avgNEES = avgNEES + (1/stepNum)*stateErr(:,i)'*inv(diag(stateVar(:,i)))*stateErr(:,i);
    end
end