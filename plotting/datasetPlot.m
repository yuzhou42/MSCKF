% load('../datasets/dataset3.mat');
load('../datasets/dataset3_fresh2_500noisy.mat');

fontSize = 14;
markerSize = 50;
lineWidth = 1.2;

figure(1); clf; hold on;

% scatter3(rho_i_pj_i(1,1:40), rho_i_pj_i(2,1:40), rho_i_pj_i(3,1:40), markerSize, 'om', 'fill');
% scatter3(rho_i_pj_i(1,41:60), rho_i_pj_i(2,41:60), rho_i_pj_i(3,41:60), markerSize, 'og', 'fill');
% scatter3(rho_i_pj_i(1,61:end), rho_i_pj_i(2,61:end), rho_i_pj_i(3,61:end), markerSize, 'or', 'fill');
scatter3(rho_i_pj_i(1,:), rho_i_pj_i(2,:), rho_i_pj_i(3,:), markerSize, 'or', 'fill');
plot3(r_i_vk_i(1,:), r_i_vk_i(2,:), r_i_vk_i(3,:), '-b', 'LineWidth', lineWidth);
legend('Features (100)', 'Features (60)', 'Features (40)', 'Sensor trajectory');

set(gca, 'FontSize', fontSize);
campos([-18.7401, -5.8668, 10.6855]);
camtarget([1.5, 1.5, 1]);
grid on; grid minor;
axis equal;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');