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
axis([0 inf 90 110])
leg = legend('Setpoint', 'System', 'Measured');
set(leg, 'Position', [0.139 0.795 0.16 0.119], 'FontSize', 8);
leg.ItemTokenSize = [20, 15];
subplot(312)
plot(t, Y(2, :), 'b', 'LineWidth', 1.5);
hold on
plot(t, Yfail(2, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('C_A [mol/l]'); grid on; hold off
axis([0 inf 0.05 0.15])
subplot(313)
plot(t, Xsp(3, :), 'r:', 'LineWidth', 1.5);   
hold on
plot(t, Y(3, :), 'b', 'LineWidth', 1.5);
plot(t, Yfail(3, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('T [K]'); grid on; hold off
axis([0 inf 430 460])

% Create textarrow
annotation(fig, 'textarrow',[0.712 0.749], [0.8 0.826], ...
    'String', {'Sensor fault', 'income'}, 'LineWidth', 1, 'HorizontalAlignment', 'center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
annotation(fig, 'textarrow', [0.494 0.561], [0.504 0.518], ...
    'String', {'Difference due to', 'sensor fault'}, 'LineWidth', 1, 'HorizontalAlignment','center', ...
    'HeadWidth', 6, 'HeadLength', 6, 'FontSize', 8);
print -dsvg figs/FDD_CSTR_state.svg

%% RUIO error
figure('Name', 'RUIO error')
subplot(211)
stairs(t, Error_1, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|'); grid on
axis([0 inf 0 1.5])
subplot(212)
stairs(t, Error_2, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|'); grid on
axis([0 inf 0 0.3])

%% UIOO error
figure('Name', 'UIOO error')
subplot(211)
stairs(t, Error1, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|_1'); grid on
axis([0 inf 0 4e-4])
subplot(212)
stairs(t, Error2, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|_2'); grid on
axis([0 inf 0 2e-2])

%% Actuator fault estimation
figure('Name', 'Actuator fault estimation')
subplot(211)
stairs(t, Fact1, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufails(1, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
axis([0 inf -0.5 5.5])
subplot(212)
stairs(t, Fact2, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufails(2, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf -5.5 0.1])

%% Sensor fault estimation
figure('Name', 'Sensor fault estimation');
subplot(211)
stairs(t, Fsen1, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Yfail(1, :) - Y(1, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\Theta_1 [K]'); grid on
axis([0 inf -1.25 0.25])
subplot(212)
stairs(t, Fsen2, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Yfail(3, :) - Y(3, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\Theta_2 [K]'); grid on
axis([0 inf -4.5 0.5])

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
xlabel('Tiempo [min]'); ylabel('\mu_i'); yticks([0 0.2 0.4 0.6 0.8 1]); yticklabels({'0', '0,2', '0,4', '0,6', '0,8', '1'});
pbaspect([2 1 1]);
leg = legend('\mu_1', '\mu_2', '\mu_3', '\mu_4', '\mu_5', '\mu_6', '\mu_7', '\mu_8', '\mu_9', 'Location', 'East');
set(leg, 'Position', [0.697 0.325 0.077 0.383], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];