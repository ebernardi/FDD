%% CSTR
clc; clear all; close all;
yalmip('clear');

%% Simulation parameters
Time = 400;                              % Simulation end time
Ts = 0.05;                               % Sample time [min]
Nsim = Time/Ts;                     % Simulation steps
t = 0:Ts:Time-Ts;                    % Simulation time
Fact_1 = [0; -5]; Fact_2 = [5; 0];              % Actuator fault magnitude
Fsen_1 = [-5; 0; 0]; Fsen_2 = [0; 0; -3];	% Sensor fault magnitude

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
% Kr = [0.5; 0.5]; 
% Ki = [1.25; 1.25];
Kr = [-2; -2]; 
Ki = [-1.5; -1.5];

% Ingreso las opciones de la ODE 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-12, 'AbsTol', 1e-12, ...
	'NormControl', 'on', 'InitialStep', 1.0e-4, 'MaxStep', 1.0);

%% Polytope model
V_min = 90;		% Volumen mínimo (m^3)
V_mid = 100;	% Volumen medio (m^3)
V_max = 110;	% Volumen máximo (m^3)
% CA_min = 0.06;	% Concentración mínima (mol/l)
% CA_mid = 0.087;	% Concentración media (mol/l)
% CA_max = 0.12;	% Concentración máxima (mol/l)
Tr_min = 434;	% Temperatura mínima (°K)
Tr_mid = 441.5;	% Temperatura media (°K)
Tr_max = 450;	% Temperatura máxima (°K)

run CSTR_polytope;

% Start point
Vr = V_mid;				% [l] Volumen del reactor
Tr = Tr_min;             % [K] Temperatura de salida
% Ca = CA_min;            % [mol/l] Concentración de salida
run CSTR_linear;
x0 = [Vr; Ca; Tr];

%% Unknown input output observer
run CSTR_DLPV_UIOO;
M = 9;

%% Simulation Setup
U = zeros(nu, Nsim);                    % Control Input
Ufail = zeros(nu, Nsim);               % Fault control Input
X = zeros(nx, Nsim+1);                % States
Y = zeros(ny, Nsim);                     % Measure outputs
Yfail = zeros(ny, Nsim);                % Fault measure outputs
mu = zeros(M, Nsim);                  % Membership
Xsp = zeros(nx, Nsim);                 % Set-point

% Observer 1
Z1 = zeros(nx, Nsim+1);              % Observer states
J1 = zeros(rank(L1_1), Nsim);                      % Monitorated outputs
X_UIOO1 = zeros(nx, Nsim);         % Estimated states
res1 = zeros(nx, Nsim);                % Residue
Error1 = zeros(1, Nsim);               % Error
Fsen1 = zeros(1, Nsim);               % Estimated sensor fault
FO1 = zeros(1, Nsim);                  % Sensor fault detection

% Observer 2
Z2 = zeros(nx, Nsim+1);              % Observer states
J2 = zeros(rank(L2_1), Nsim);       % Monitorated outputs
X_UIOO2 = zeros(nx, Nsim);         % Estimated states
res2 = zeros(nx, Nsim);                % Residue
Error2 = zeros(1, Nsim);               % Error
Fsen2 = zeros(1, Nsim);               % Estimated sensor fault
FO2 = zeros(1, Nsim);                  % Sensor fault detection

% Initial states and inputs
X(:, 1) = x0;
Xsp(:, 1) = x0;
U(:, 1) = U_lin;

%% Simulation
for FTC = 0 % 0 - FTC is off; 1 - FTC is on

	for k = 1:Nsim
        tk = k*Ts; % Simulation time

        %% Actuator fault income
        Ufail(:, k) = U(:, k);      

        if tk > 70 && tk < 120
            Ufail(:, k) = U(:, k) + Fact_2;
        end
        
        if tk > 170 && tk < 200
            Ufail(:, k) = U(:, k) + Fact_1;
        end
        
        %% Process simulation with ODE
        [tsim, x] = ode45(@(x, u) CSTR(X(:, k), Ufail(:, k)) , [0 Ts], X(:, k), options);
        X(:, k+1) = x(end, :)';
        Y(:, k) = C*X(:, k);
        
        %% Sensor fault income
        Yfail(:, k) = Y(:, k);      

        if tk >280 && tk < 330
            Yfail(:, k) = Y(:, k) + Fsen_2;
        end

        if tk > 340 && tk < 370
            Yfail(:, k) = Y(:, k) + Fsen_1;
        end
        
        %% Output membership
        mu(:, k) = membership(Y(:, k), V_min, V_mid, V_max, Tr_min, Tr_mid, Tr_max);

        %% DLPV-UIOO 1
        J1(:, k) = Yfail([1 2], k);
        Z1(:, k+1) = mu(1, k)*(N1_1*Z1(:, k) + L1_1*J1(:, k) + G1_1*U(:, k) + Tg1_1) ...
                         + mu(2, k)*(N1_2*Z1(:, k) + L1_2*J1(:, k) + G1_2*U(:, k) + Tg1_2) ...
                         + mu(3, k)*(N1_3*Z1(:, k) + L1_3*J1(:, k) + G1_3*U(:, k) + Tg1_3) ...
                         + mu(4, k)*(N1_4*Z1(:, k) + L1_4*J1(:, k) + G1_4*U(:, k) + Tg1_4) ...
                         + mu(5, k)*(N1_5*Z1(:, k) + L1_5*J1(:, k) + G1_5*U(:, k) + Tg1_5) ...
                         + mu(6, k)*(N1_6*Z1(:, k) + L1_6*J1(:, k) + G1_6*U(:, k) + Tg1_6) ...
                         + mu(7, k)*(N1_7*Z1(:, k) + L1_7*J1(:, k) + G1_7*U(:, k) + Tg1_7) ...
                         + mu(8, k)*(N1_8*Z1(:, k) + L1_8*J1(:, k) + G1_8*U(:, k) + Tg1_8) ...
                         + mu(9, k)*(N1_9*Z1(:, k) + L1_9*J1(:, k) + G1_9*U(:, k) + Tg1_9);

        X_UIOO1(:, k) = Z1(:, k) - E1*J1(:, k);

        % Residue 1
        res1(:, k) = Yfail(:, k) - X_UIOO1(:, k);
        
        % Error norm 1
        Error1(k) = sqrt(res1(2, k)^2);
        
        if Error1(k) > 3e-5
            FO1(k) = true;
        else
            FO1(k) = false;
        end
            
        %% DLPV-UIOO 2
        J2(:, k) = Yfail([2 3], k);
        Z2(:, k+1) = mu(1, k)*(N2_1*Z2(:, k) + L2_1*J2(:, k) + G2_1*U(:, k) + Tg2_1) ...
                         + mu(2, k)*(N2_2*Z2(:, k) + L2_2*J2(:, k) + G2_2*U(:, k) + Tg2_2) ...
                         + mu(3, k)*(N2_3*Z2(:, k) + L2_3*J2(:, k) + G2_3*U(:, k) + Tg2_3) ...
                         + mu(4, k)*(N2_4*Z2(:, k) + L2_4*J2(:, k) + G2_4*U(:, k) + Tg2_4) ...
                         + mu(5, k)*(N2_5*Z2(:, k) + L2_5*J2(:, k) + G2_5*U(:, k) + Tg2_5) ...
                         + mu(6, k)*(N2_6*Z2(:, k) + L2_6*J2(:, k) + G2_6*U(:, k) + Tg2_6) ...
                         + mu(7, k)*(N2_7*Z2(:, k) + L2_7*J2(:, k) + G2_7*U(:, k) + Tg2_7) ...
                         + mu(8, k)*(N2_8*Z2(:, k) + L2_8*J2(:, k) + G2_8*U(:, k) + Tg2_8) ...
                         + mu(9, k)*(N2_9*Z2(:, k) + L2_9*J2(:, k) + G2_9*U(:, k) + Tg2_9);

        X_UIOO2(:, k) = Z2(:, k) - E2*J2(:, k);
        
        % Residue 2
        res2(:, k) = Yfail(:, k) - X_UIOO2(:, k);

        % Error norm 2
        Error2(k) = sqrt(res2(3, k)^2);
        
        if Error2(k) > 2e-2
            FO2(k) = true;
        else
            FO2(k) = false;
        end
        
        %% Sensor fault estimation
        % Sensor fault 1
        if FO1(k) && ~FO2(k)
            Fsen1(k) = res2(1, k);
        end
        
        % Sensor fault 2
         if ~FO1(k) && FO2(k)
            Fsen2(k) = res1(3, k);
         end
        
        %% Setpoint
        Xsp(1, k) = V_mid;
        Xsp(3, k) = Tr_mid;
    
        if tk <= 10
            Xsp(3, k) = Tr_min;
        elseif tk <= 25
            Xsp(3, k) = Tr_min+((Tr_mid-Tr_min)*(tk-10)/15);
        elseif tk <= 65
           Xsp(3, k) = Tr_mid;   
        elseif tk <= 80
            Xsp(3, k) = Tr_mid+((Tr_max-Tr_mid)*(tk-65)/15);
        elseif tk <= 95
           Xsp(1, k) = V_min;%+((V_mid-V_min)*(tk-80)/15);
        elseif tk <= 135
           Xsp(1, k) = V_mid;   
        elseif tk <= 150
            Xsp(1, k) = V_mid+((V_max-V_mid)*(tk-135)/15);
        elseif tk <= 200
            Xsp(1, k) = V_max;
        end
        
        %% PID
        u0(1) = U(1, k);
        u0(2) = U(2, k);
        ek_1 = ek;
        ek(1) = (Xsp(1, k) - Yfail(1, k));
        ek(2) = (Xsp(3, k) - Yfail(3, k));
        % Only calefactor fluid
%         u0(1) = 100;
%         u0(2) = u0(2) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        % Mixed control
%         u0(1) = u0(1) + Kr(2)*(ek(2) - ek_1(2)) + Ts*Ki(2)*ek(2);
%         u0(2) = u0(2) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        % Direct control
        u0(1) = u0(1) + Kr(1)*(ek(1) - ek_1(1)) + Ts*Ki(1)*ek(1);
        u0(2) = u0(2) + Kr(2)*(ek(2) - ek_1(2)) + Ts*Ki(2)*ek(2);
        U(:, k+1) = [u0(1); u0(2)];
        
	end
end

%% Set Plots
vecrojo = [0.7; 0; 0]; vecverde = [0; 0.8; 0]; vecazul = [0; 0; 0.6]; negro = [.1; .1; .1]; gris = [.5; .7; .5];

%% Colors
azul = [0 0.4470 0.7410];
naranja = [0.8500 0.3250 0.0980];
amarillo = [0.9290 0.6940 0.1250];
violeta = [0.4940 0.1840 0.5560];
verde = [0.4660 0.6740 0.1880];
celeste = [0.3010 0.7450 0.9330];
bordo = [0.6350 0.0780 0.1840];

orange_red = [255 69 0]/255;
forest_green = [34 139 34]/255;
royal_blue = [65 105 225]/255;
dark_blue = [0 0 139]/255;
gold = [255 215 0]/255;
chocolate = [210 105 30]/255;

%% States
figure
subplot(311)
plot(t, Xsp(1, :), 'r:', 'LineWidth', 1.5);
hold on
plot(t, Y(1, :), 'b', 'LineWidth', 1.5);
plot(t, Yfail(1, :), 'g--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('V [l]'); grid on; hold off
axis([0 inf 90 110])
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

%% Error
figure
subplot(211)
stairs(t, Error1, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|_1'); grid on
axis([0 inf 0 1.5e-4])
subplot(212)
stairs(t, Error2, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|_2'); grid on
axis([0 inf 0 6e-1])

%% Fault estimation
figure
subplot(211)
stairs(t, Fsen1, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Yfail(1, :) - Y(1, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\Theta_1 [K]'); grid on
axis([0 inf -5.5 0.5])
subplot(212)
stairs(t, Fsen2, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Yfail(3, :) - Y(3, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\Theta_2 [K]'); grid on
axis([0 inf -3.5 0.5])

%% Membership
fig = figure('DefaultAxesFontSize', 9, 'Color', [1 1 1]);
plot(t, mu(1, :), 'Color', naranja, 'linewidth', 1.5); hold on; grid on;
plot(t, mu(2, :), 'Color', azul, 'linewidth', 1.5);
plot(t, mu(3, :), 'k', 'linewidth', 1.5);
plot(t, mu(4, :), 'r', 'linewidth', 1.5);
plot(t, mu(5, :), 'Color', forest_green, 'linewidth', 1.5);
plot(t, mu(6, :), 'Color', bordo, 'linewidth', 1.5);
plot(t, mu(7, :), 'm', 'linewidth', 1.5);
plot(t, mu(8, :), 'Color', violeta, 'linewidth', 1.5);
plot(t, mu(9, :), 'Color', amarillo, 'linewidth', 1.5); hold off;
axis([0 inf 0 1]);
xlabel('Tiempo [min]'); ylabel('\mu_i'); yticks([0 0.2 0.4 0.6 0.8 1]); yticklabels({'0', '0,2', '0,4', '0,6', '0,8', '1'});
pbaspect([2 1 1]);
leg = legend('\mu_1', '\mu_2', '\mu_3', '\mu_4', '\mu_5', '\mu_6', '\mu_7', '\mu_8', '\mu_9', 'Location', 'East');
set(leg, 'Position', [0.697 0.325 0.077 0.383], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];
