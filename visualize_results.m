function visualize_results(t, y0, y, u0, u, e1, Kp, Ki, Kd, kp, ki, kd)
close all
subplot(131)
plot(t, y, 'LineWidth',1, 'Color', [0.6350 0.0780 0.1840])
hold on
plot(t, y0, 'LineWidth',1, 'Color',[0 0.4470 0.7410])
title('y signals')
legend('SI','Tuner')
axis([0 4 0.8 1.2])
kp_txt = ['SI kp         ' num2str(kp)];
text(0.1,0.900,kp_txt)
kp_txt2 = ['Tuner kp    ' num2str(Kp)];
text(0.1,0.885,kp_txt2)
ki_txt = ['SI ki         ' num2str(ki)];
text(0.1,0.865,ki_txt)
ki_txt2 = ['Tuner ki    ' num2str(Ki)];
text(0.1,0.850,ki_txt2)
kd_txt = ['SI kd         ' num2str(kd)];
text(0.1,0.830,kd_txt)
kd_txt2 = ['Tuner kd    ' num2str(Kd)];
text(0.1,0.815,kd_txt2)

subplot(132)
plot(t, u, 'LineWidth', 1, 'Color',[0.6350 0.0780 0.1840])
hold on
plot(t, u0, 'LineWidth', 1, 'Color',[0 0.4470 0.7410])
title('u signal')
legend('SI','Tuner')
axis([0 4 -2 2])

subplot(133)
plot(t, e1, 'LineWidth',1, 'Color', [0.6350 0.0780 0.1840])
title('e1=(unitstep(j) - y(j))^2')
axis([0 4 -0.1 1.5])
end