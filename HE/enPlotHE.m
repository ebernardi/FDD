clc; clear; close all;

% Load data
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
plot(t, Yfail(1, :)-UIOO(1).Fsen, 'b:', 'LineWidth', 1.5);
plot(t, Yfail(1, :), 'g--', 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\theta_{1_s} [K]'); grid on
axis([0 inf 494 499])
leg = legend('Setpoint', 'Estimated', 'Measured', 'Location', 'SouthEast');
set(leg, 'Position', [0.748 0.764 0.148 0.109], 'FontSize', 8);
leg.ItemTokenSize = [20, 15];
subplot(312)
plot(t, Xsp(2, :), 'r-.', 'LineWidth', 1.5);
hold on
plot(t, Yfail(2, :)-UIOO(2).Fsen, 'b:', 'LineWidth', 1.5);
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
annotation(fig, 'textarrow', [0.325 0.355], [0.538 0.563], ...
    'String', {'Sensor fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.245 0.259], [0.844 0.809], ...
    'String', {'Actuator fault', 'effect'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'arrow', [0.217 0.191], [0.84 0.807], 'LineWidth', 1, ...
    'HeadWidth', 6, 'HeadLength', 6);
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
axis([0 inf 85 125])
subplot(212)
stairs(t, U(2, 1:end-1), 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(2, :), 'g--', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf 7.2 9.2])
leg = legend('Faulty', 'Compensated', 'Location', 'SouthWest');
leg.ItemTokenSize = [20, 18];

% Create textarrow
annotation(fig, 'textarrow', [0.202 0.175], [0.743 0.824], ...
    'String', {'Actuator', 'abrupt fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.799 0.8], [0.302 0.235], ...
    'String', {'Difference due to', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_HE_input.svg

%% RUIO error
fig = figure('Name', 'RUIO error');
subplot(211)
stairs(t, RUIO(1).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(1, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_1}'); grid on
axis([0 inf 0 2.8])
subplot(212)
stairs(t, RUIO(2).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(2, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{Q_2}'); grid on
axis([0 inf 0 1.4])
leg = legend('Residue', 'Threshold');
leg.ItemTokenSize = [20, 18];

% Create textarrow
annotation(fig, 'textarrow', [0.335 0.357], [0.654 0.626], ...
    'String', {'Fault', 'detection'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.523 0.55], [0.259 0.233], ...
    'String', {'Exponential', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'arrow', [0.435 0.426], [0.247 0.197], 'LineWidth', 1, ...
    'HeadWidth', 6, 'HeadLength', 6);
annotation(fig, 'textarrow', [0.221 0.221], [0.262 0.209], ...
    'String', {'Abrupt', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.5 0.484], [0.714 0.685], ...
    'String', {'Exponential', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'arrow', [0.607 0.616], [0.714 0.664], 'LineWidth', 1, ...
    'HeadWidth', 6, 'HeadLength', 6);
annotation(fig, 'textarrow', [0.741 0.757], [0.842 0.819], ...
    'String', {'Exponential', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.789 0.789], [0.171 0.133], ...
    'String', {'Modelling', 'errors'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);

print -dsvg figs/FDD_HE_RUIOerror.svg

%% UIOO error
fig = figure('Name', 'UIOO error');
subplot(211)
stairs(t, UIOO(1).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(3, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{\theta_1}'); grid on
% axis([0 inf 0 6e-3])
axis([0 inf 0 1.8])
leg = legend('Residue', 'Threshold');
leg.ItemTokenSize = [20, 18];
subplot(212)
stairs(t, UIOO(2).error, 'b', 'LineWidth', 1.5)
hold on
plot(t, threshold(4, :), '-.r', 'linewidth', 1.5); hold off;
xlabel('Time [min]'); ylabel('|e|_{\theta_2}'); grid on
axis([0 inf 0 3.5])

% Create textarrow
annotation(fig, 'textarrow', [0.526 0.551], [0.659 0.623], ...
    'String', {'Fault', 'detection'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.532 0.562], [0.864 0.845], ...
    'String', {'Exponential', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.21 0.21], [0.728 0.671], ...
    'String', {'Abrupt', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.169 0.142], [0.185 0.157], ...
    'String', {'Exponential', 'threshold', 'convergence'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.335 0.369], [0.371 0.347], ...
    'String', {'Exponential', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.73 0.759], [0.359 0.333], ...
    'String', {'Exponential', 'actuator fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.659 0.687], [0.164 0.138], ...
    'String', {'Modelling', 'errors'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);

print -dsvg figs/FDD_HE_UIOOerror.svg

%% Actuator fault estimation
fig = figure('Name', 'Actuator fault estimation');
subplot(211)
stairs(t, RUIO(1).Fact, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
axis([0 inf -0.5 5.5])
leg = legend('Estimation', 'Fault', 'Location', 'NorthEast');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, RUIO(2).Fact, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Ufails(2, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf -0.5 0.03])

% Create axes
ax = axes('Parent', fig, 'Position', [0.319 0.716 0.2 0.167], 'FontSize', 8);
hold(ax, 'on');
plot(t, RUIO(1).Fact, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [28 68]); ylim(ax, [4.7 5.3]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.327 0.175 0.255 0.200], 'FontSize', 8);
hold(ax, 'on');
plot(t, RUIO(2).Fact, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Ufails(2, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [579 586]); ylim(ax, [-0.15 0]);
box(ax, 'on'); grid(ax, 'on');

% Create textarrow
annotation(fig, 'textarrow', [0.597 0.573], [0.705 0.733], ...
    'String', {'Detection', 'error'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.457 0.401], [0.308 0.334], ...
    'String', {'Threshold', 'effect'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);

print -dsvg figs/FDD_HE_RUIOestimation.svg

%% Sensor fault estimation
fig = figure('Name', 'Sensor fault estimation');
subplot(211)
stairs(t, UIOO(1).Fsen, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\Theta_1 [K]'); grid on
axis([0 inf -0.25 3])
leg = legend('Estimation', 'Fault', 'Location', 'NorthEast');
leg.ItemTokenSize = [20, 15];
subplot(212)
stairs(t, UIOO(2).Fsen, 'Color', bordo, 'LineWidth', 1.5)
hold on
stairs(t, Yfail(2, :) - Y(2, :), '-.', 'Color', azul, 'LineWidth', 1.5); hold off
xlabel('Time [min]'); ylabel('\Theta_2 [K]'); grid on
axis([0 inf -4 0.25])

% Create axes
ax = axes('Parent', fig, 'Position', [0.226 0.684 0.229 0.2], 'FontSize', 8);
hold(ax, 'on');
plot(t, UIOO(1).Fsen, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
plot(t, Yfail(1, :) - Y(1, :), '-.', 'Color', azul, 'linewidth', 1.5); hold off;
xlim(ax, [400 500]); ylim(ax, [2 3]);
box(ax, 'on'); grid(ax, 'on');

% Create axes
ax = axes('Parent', fig, 'Position', [0.572 0.192 0.243 0.193], 'FontSize', 8);
hold(ax, 'on');
plot(t, UIOO(2).Fsen, 'Color', bordo, 'linewidth', 1.5); hold on; grid on;
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
xlabel('Time [min]'); ylabel('\mu_i');% yticks([0 0.2 0.4 0.6 0.8 1]); yticklabels({'0', '0,2', '0,4', '0,6', '0,8', '1'});
pbaspect([2 1 1]);
leg = legend('\mu_1', '\mu_2', '\mu_3', '\mu_4', '\mu_5', '\mu_6', '\mu_7', '\mu_8', '\mu_9', 'Location', 'East');
set(leg, 'Position', [0.159 0.307 0.077 0.418], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];
print -dsvg figs/FDD_HE_member.svg
