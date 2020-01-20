clc; clear; close all;

load runCSTR

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
plot(t, Xsp(1, :), 'r:', 'LineWidth', 1.5);
hold on
plot(t, Y(1, :), 'b', 'LineWidth', 1.5);
plot(t, Yfail(1, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('V [l]'); grid on; hold off
axis([0 inf 90 112])
leg = legend('Setpoint', 'System', 'Measured', 'Location', 'SouthEast');
leg.ItemTokenSize = [20, 15];
subplot(312)
plot(t, Y(2, :), 'b', 'LineWidth', 1.5);
hold on
plot(t, Yfail(2, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('C_A [mol/l]'); grid on; hold off
axis([0 inf 0.04 0.12])
subplot(313)
plot(t, Xsp(3, :), 'r:', 'LineWidth', 1.5);   
hold on
plot(t, Y(3, :), 'b', 'LineWidth', 1.5);
plot(t, Yfail(3, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('T [K]'); grid on; hold off
axis([0 inf 430 460])

% Create textarrow
annotation(fig, 'textarrow',[0.301 0.357], [0.855 0.813], ...
    'String', {'Sensor fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.746 0.793], [0.198 0.231], ...
    'String', {'Difference due to', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_CSTR_state.svg

%% Manipulated variables
fig = figure('Name', 'Manipulated variables');
subplot(211)
stairs(t, U(1, 1:end-1), 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(1, :), 'g--', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('Q_s [l/min]'); grid on
axis([0 inf 94 102])
leg = legend('Faulty', 'Compensated', 'Location', 'SouthEast');
leg.ItemTokenSize = [20, 18];
subplot(212)
stairs(t, U(2, 1:end-1), 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(2, :), 'g--', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('Q_c [l/min]'); grid on
axis([0 inf 80 110])

% Create textarrow
annotation(fig, 'textarrow',[0.531 0.551], [0.234 0.285], ...
    'String', {'Actuator fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.329 0.287], [0.722 0.703], ...
    'String', {'Difference due to', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_CSTR_input.svg

%% RUIO error
figure('Name', 'RUIO error')
subplot(211)
stairs(t, RUIO(1).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(1, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_s}'); grid on
axis([0 inf 0 2])
subplot(212)
stairs(t, RUIO(2).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(2, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_c}'); grid on
axis([0 inf 0 0.3])

%% UIOO error
figure('Name', 'UIOO error')
subplot(211)
plot(t, UIOO(1).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(3, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_V'); grid on
axis([0 inf 0 3.2e-4])
subplot(212)
plot(t, UIOO(2).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(4, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_T'); grid on
axis([0 inf 0 2e-4])

%% Actuator fault estimation
fig = figure('Name', 'Actuator fault estimation');
subplot(211)
stairs(t, RUIO(1).Fact, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_s [l/min]'); grid on
axis([0 inf -0.5 5.5])
leg = legend('Estimation', 'Fault');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, RUIO(2).Fact, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(2, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_c [l/min]'); grid on
axis([0 inf -5.5 0.25])

% Create axes
ax = axes('Parent', fig, 'Position', [0.4 0.69 0.226 0.2], 'FontSize', 8);
hold(ax, 'on');
plot(t, RUIO(1).Fact, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [29.5 35]); ylim(ax, [0 1.8]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.259 0.202 0.203 0.165], 'FontSize', 8);
hold(ax, 'on');
plot(t, RUIO(2).Fact, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(2, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [440 500]); ylim(ax, [-5.1 -4.85]);
box(ax, 'on'); grid(ax, 'on');

% Create textarrow
annotation(fig, 'textarrow', [0.518 0.485], [0.7313 0.731], ...
    'String', {'Threshold', 'effect'}, 'LineWidth', 1, ...
    'HorizontalAlignment', 'center', 'HeadWidth', 6, ...
    'HeadLength', 6, 'FontSize', 8);

print -dsvg figs/FDD_CSTR_RUIOestimation.svg

%% Sensor fault estimation
fig = figure('Name', 'Sensor fault estimation');
subplot(211)
stairs(t, UIOO(1).Fsen, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('V [l]'); grid on
axis([0 inf -2 0.25])
leg = legend('Estimation', 'Fault', 'Location', 'SouthWest');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, UIOO(2).Fsen, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(3, :) - Y(3, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('T [K]'); grid on
axis([0 inf -0.5 5])

% Create axes
ax = axes('Parent', fig, 'Position', [0.585 0.657 0.229 0.2], 'FontSize', 8);
hold(ax, 'on');
plot(t, UIOO(1).Fsen, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [250 310]); ylim(ax, [-1.6 -1.2]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.319 0.229 0.266 0.165], 'FontSize', 8);
hold(ax, 'on');
plot(t, UIOO(2).Fsen, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Yfail(3, :) - Y(3, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [610 660]); ylim(ax, [4.1 4.6]);
box(ax, 'on'); grid(ax, 'on');

print -dsvg figs/FDD_CSTR_UIOOestimation.svg

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
set(leg, 'Position', [0.748 0.307 0.075 0.418], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];
print -dsvg figs/FDD_CSTR_member.svg