p_I_G = zeros(3,kEnd-kStart+1);
q_IG = zeros(4,kEnd-kStart+1);

for plotting_k = kStart:kEnd
    p_I_G(:,plotting_k - kStart + 1) = imuStates{plotting_k}.p_I_G;
    q_IG(:,plotting_k - kStart + 1) = imuStates{plotting_k}.q_IG;
end

close all;
figure(1); clf;
plot3(p_I_G(1,:), p_I_G(2,:), p_I_G(3,:));
