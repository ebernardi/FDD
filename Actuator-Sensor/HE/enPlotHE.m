clc; clear; close all;

load runHE

% When generates flat figures
set(0, 'DefaultFigureRenderer', 'painters');

%% Set Plots
% Colors
vecrojo = [0.7; 0; 0]; vecverde = [0; 0.8; 0]; vecazul = [0; 0; 0.6]; negro = [.1; .1; .1]; gris = [.5; .7; .5];
azul = [0 0.4470 0.7410]; naranja = [0.8500 0.3250 0.0980]; amarillo = [0.9290 0.6940 0.1250];
violeta = [0.4940 0.1840 0.5560]; verde = [0.4660 0.6740 0.1880]; celeste = [0.3010 0.7450 0.9330];
bordo = [0.6350 0.0780 0.1840]; 
orange_red = [255 69 0]/255; forest_green = [34 139 34]/255; royal_blue = [65 105 225]/255; 
dark_blue = [0 0 139]/255; gold = [255 215 0]/255; chocolate = [210 105 30]/255;

%% States
fig = figure('Name', 'States');
subplot(311)
plot(t, Xsp(1, :), 'r-.', 'LineWidth', 1.5);
hold on
plot(t, Y(1, :), 'b:', 'LineWidth', 1.5);
plot(t, Yfail(1, :), 'g--', 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\theta_{1_s} [K]'); grid on
axis([0 inf 494 499])
leg = legend('Setpoint', 'System', 'Measured', 'Location', 'SouthEast');
set(leg, 'Position', [0.748 0.764 0.148 0.109], 'FontSize', 8);
leg.ItemTokenSize = [20, 15];
subplot(312)
plot(t, Xsp(2, :), 'r-.', 'LineWidth', 1.5);
hold on
plot(t, Y(2, :), 'b:', 'LineWidth', 1.5);
plot(t, Yfail(2, :), 'g--', 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\theta_{2_s} [K]'); grid on
axis([0 inf 675 705])
subplot(313)
plot(t, Y(3, :), 'b:', 'LineWidth', 1.5);
hold on
plot(t, Yfail(3, :), 'g--', 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\theta_p [K]'); grid on
axis([0 inf 554 562])

% Create textarrow
annotation(fig, 'textarrow',[0.325 0.355], [0.538 0.563], ...
    'String', {'Sensor fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.529 0.563], [0.799 0.822], ...
    'String', {'Difference due to', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_HE_state.svg

%% Manipulated variables
fig = figure('Name', 'Manipulated variables');
subplot(211)
stairs(t, U(1, 1:end-1), 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(1, :), 'g--', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
axis([0 inf 77 123])
subplot(212)
stairs(t, U(2, 1:end-1), 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(2, :), 'g--', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf 7.2 9.2])
leg = legend('Faulty', 'Compensated', 'Location', 'SouthWest');
leg.ItemTokenSize = [20, 18];

% Create textarrow
annotation(fig, 'textarrow',[0.201 0.166], [0.799 0.880], ...
    'String', {'Actuator fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.799 0.8], [0.302 0.235], ...
    'String', {'Difference due to', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_HE_input.svg

%% RUIO error
figure('Name', 'RUIO error')
subplot(211)
stairs(t, Error_1, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(1, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_1}'); grid on
axis([0 inf 0 2.5])
subplot(212)
stairs(t, Error_2, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(2, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_2}'); grid on
axis([0 inf 0 1.3])

%% UIOO error
figure('Name', 'UIOO error')
subplot(211)
stairs(t, Error1, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(3, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{\theta_1}'); grid on
% axis([0 inf 0 6e-3])
axis([0 inf 0 1.5])
subplot(212)
stairs(t, Error2, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(4, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{\theta_2}'); grid on
axis([0 inf 0 2.2])

%% Actuator fault estimation
fig = figure('Name', 'Actuator fault estimation');
subplot(211)
stairs(t, Fact1, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
axis([0 inf -0.5 5.5])
leg = legend('Estimation', 'Fault', 'Location', 'NorthEast');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, Fact2, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(2, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf -0.5 0.03])

% Create axes
ax = axes('Parent', fig, 'Position', [0.319 0.716 0.2 0.167], 'FontSize', 8);
hold(ax, 'on');
plot(t, Fact1, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [28 60]); ylim(ax, [0 5]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.327 0.211 0.255 0.165], 'FontSize', 8);
hold(ax, 'on');
plot(t, Fact2, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(2, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [620 680]); ylim(ax, [-0.432 -0.426]);
box(ax, 'on'); grid(ax, 'on');

% Create textarrow
annotation(fig, 'textarrow', [0.597 0.573], [0.705 0.733], ...
    'String', {'Detection', 'error'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.400 0.366], [0.754 0.739], ...
    'String', {'Threshold', 'effect'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);

print -dsvg figs/FDD_HE_RUIOestimation.svg

%% Sensor fault estimation
fig = figure('Name', 'Sensor fault estimation');
subplot(211)
stairs(t, Fsen1, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\Theta_1 [K]'); grid on
axis([0 inf -0.25 3])
leg = legend('Estimation', 'Fault', 'Location', 'NorthEast');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, Fsen2, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(2, :) - Y(2, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\Theta_2 [K]'); grid on
axis([0 inf -4 0.25])

% Create axes
ax = axes('Parent', fig, 'Position', [0.226 0.684 0.229 0.2], 'FontSize', 8);
hold(ax, 'on');
plot(t, Fsen1, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [400 500]); ylim(ax, [2 3]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.572 0.192 0.243 0.193], 'FontSize', 8);
hold(ax, 'on');
plot(t, Fsen2, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Yfail(2, :) - Y(2, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [240 310]); ylim(ax, [-3.65 -3.35]);
box(ax, 'on'); grid(ax, 'on');

print -dsvg figs/FDD_HE_UIOOestimation.svg

%% Membership
fig = figure('DefaultAxesFontSize', 9, 'Color', [1 1 1], 'Name', 'Membership');
plot(t, mu_out(1, :), 'Color', naranja, 'linewidth', 1.5); hold on; grid on;
plot(t, mu_out(2, :), 'Color', azul, 'linewidth', 1.5);
plot(t, mu_out(3, :), 'k', 'linewidth', 1.5);
plot(t, mu_out(4, :), 'r', 'linewidth', 1.5);
plot(t, mu_out(5, :), 'Color', forest_green, 'linewidth', 1.5);
plot(t, mu_out(6, :), 'Color', bordo, 'linewidth', 1.5);
plot(t, mu_out(7, :), 'm', 'linewidth', 1.5);
plot(t, mu_out(8, :), 'Color', violeta, 'linewidth', 1.5);
plot(t, mu_out(9, :), 'Color', amarillo, 'linewidth', 1.5); hold off;
axis([0 inf 0 1]);
xlabel('Tiempo [min]'); ylabel('\mu_i');% yticks([0 0.2 0.4 0.6 0.8 1]); yticklabels({'0', '0,2', '0,4', '0,6', '0,8', '1'});
pbaspect([2 1 1]);
leg = legend('\mu_1', '\mu_2', '\mu_3', '\mu_4', '\mu_5', '\mu_6', '\mu_7', '\mu_8', '\mu_9', 'Location', 'East');
set(leg, 'Position', [0.159 0.307 0.077 0.418], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];
print -dsvg figs/FDD_HE_member.svg