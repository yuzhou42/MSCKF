kNum = length(prunedStates);

if kNum > 0
    p_I_G_est = zeros(3,kNum);
    p_I_G_imu = zeros(3, kNum);
    kPlot = zeros(1,kNum);

    for k=1:kNum
        state_k = prunedStates{k}.state_k;

        C_CG = quatToRotMat(prunedStates{k}.q_CG);
        C_CI = quatToRotMat(camera.q_CI);
        p_I_G_est(:,k) = prunedStates{k}.p_C_G - C_CG' * C_CI * camera.p_C_I;
        p_I_G_imu(:,k) = msckfState_imuOnly{state_k}.imuState.p_I_G;

        kPlot(k) = state_k;
    end

    figure(1); clf; hold on;
    plot3(p_I_G_est(1,:),p_I_G_est(2,:),p_I_G_est(3,:),'-b');
    plot3(p_I_G_imu(1,:),p_I_G_imu(2,:),p_I_G_imu(3,:),'-r');
    plot3(r_i_vk_i(1,kPlot),r_i_vk_i(2,kPlot),r_i_vk_i(3,kPlot),'-g');
%     if ~isempty(map)
%         scatter3(map(1,:),map(2,:),map(3,:),'or');
%     end
    xlabel('x');ylabel('y');zlabel('z');
    legend('MSCKF','IMU integration','Ground Truth');
%     legend('MSCKF','IMU integration');
    grid on;
    
    drawnow;
end