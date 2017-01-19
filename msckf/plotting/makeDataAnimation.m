addpath('../utils');
load('../dataset3');

fontSize = 16;
markerSize = 100;
lineWidth = 3;

writerObj = VideoWriter('dataset3movie.mp4', 'MPEG-4');
writerObj.FrameRate=60;

open(writerObj);

h = figure(1); clf; hold on;
set(h, 'Position', [0 0 640 480], 'Color', 'w');

k1 = 15;

hplot = plot3(r_i_vk_i(1,k1), r_i_vk_i(2,k1), r_i_vk_i(3,k1), '-b', 'LineWidth', 3);
C = axisAngleToRotMat(theta_vk_i(:,k1));
hx = quiver3(r_i_vk_i(1,k1), r_i_vk_i(2,k1), r_i_vk_i(3,k1), C(1,1), C(2,1), C(3,1), 0.5, '-r', 'LineWidth', lineWidth);
hy = quiver3(r_i_vk_i(1,k1), r_i_vk_i(2,k1), r_i_vk_i(3,k1), C(1,2), C(2,2), C(3,2), 0.5, '-g', 'LineWidth', lineWidth);
hz = quiver3(r_i_vk_i(1,k1), r_i_vk_i(2,k1), r_i_vk_i(3,k1), C(1,3), C(2,3), C(3,3), 0.5, '-b', 'LineWidth', lineWidth);

visible = y_k_j(1,k1,:) ~= -1;
hvisible = scatter3(rho_i_pj_i(1,visible), rho_i_pj_i(2,visible), rho_i_pj_i(3,visible), markerSize, 'og', 'fill');
hinvisible = scatter3(rho_i_pj_i(1,~visible), rho_i_pj_i(2,~visible), rho_i_pj_i(3,~visible), markerSize, 'or', 'fill');

xlim([0,3]); ylim([0,3]); zlim([0,2]);
set(gca, 'FontSize', fontSize);
campos([-18.7401, -5.8668, 10.6855]);
camtarget([1.5, 1.5, 1]);
grid on;

writeVideo(writerObj, getframe(h));

for i = k1+1:numel(t)
    delete(hvisible); delete(hinvisible); delete(hx); delete(hy); delete(hz)
    
    visible = y_k_j(1,i,:) ~= -1;
    hvisible = scatter3(rho_i_pj_i(1,visible), rho_i_pj_i(2,visible), rho_i_pj_i(3,visible), markerSize, 'og', 'fill');
    hinvisible = scatter3(rho_i_pj_i(1,~visible), rho_i_pj_i(2,~visible), rho_i_pj_i(3,~visible), markerSize, 'or', 'fill');
    set(hplot, 'XData', r_i_vk_i(1,1:i), 'YData', r_i_vk_i(2,1:i), 'ZData', r_i_vk_i(3,1:i));
    
    C = axisAngleToRotMat(theta_vk_i(:,i));
    hx = quiver3(r_i_vk_i(1,i), r_i_vk_i(2,i), r_i_vk_i(3,i), C(1,1), C(2,1), C(3,1), 0.5, '-r', 'LineWidth', lineWidth);
    hy = quiver3(r_i_vk_i(1,i), r_i_vk_i(2,i), r_i_vk_i(3,i), C(1,2), C(2,2), C(3,2), 0.5, '-g', 'LineWidth', lineWidth);
    hz = quiver3(r_i_vk_i(1,i), r_i_vk_i(2,i), r_i_vk_i(3,i), C(1,3), C(2,3), C(3,3), 0.5, '-b', 'LineWidth', lineWidth);
    
    drawnow;
    writeVideo(writerObj, getframe(h));
end

close(writerObj);