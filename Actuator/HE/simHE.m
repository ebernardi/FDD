%% HE
clc; clear; yalmip('clear');
close all;

%% Simulation parameters
Time = 50;                              % Simulation end time 
Ts = 0.05;                               % Sample time [min]
Nsim = Time/Ts;                     % Simulation steps
t = 0:Ts:Time-Ts;                    % Simulation time
Fault_1 = [0; 1]; Fault_2 = [-8; 0];          % Fault magnitude

%% Parámetros del PID
ek = [0; 0]; ek_1 = [0; 0];
Kr = [0.5 0.5]; 
Ki = [1.25; 1.25];

% Ingreso las opciones de la ODE 'RelTol', 1e-6, 'AbsTol', 1e-6
options = odeset ('RelTol', 1e-6, 'AbsTol', 1e-6, ...
	'NormControl', 'on', 'InitialStep', 1.0e-2, 'MaxStep', 1.0);

%% Polytope model
Theta_1s_min = 495;     % Temperatura mínima de salida de fluido 1 (K)
Theta_1s_mid = 497.32;     % Temperatura media de salida de fluido 1 (K)
Theta_1s_max = 500;     % Temperatura máxima de salida de fluido 1 (K)

Theta_2s_min = 662;     % Temperatura mínima de salida de fluido 2 (K)
Theta_2s_mid = 695.915;     % Temperatura media de salida de fluido 2 (K)
Theta_2s_max = 725;     % Temperatura máxima de salida de fluido 2 (K)

run HE_polytope;

% Start point
Theta_1s = Theta_1s_min;     % Temperatura de salida de fluido 1 (K)
Theta_2s = Theta_2s_min;     % Temperatura de salida de fluido 2 (K)
run HE_linear;
x0 = [Theta_1s; Theta_2s; Theta_p];

%% MPC controller
% Parameters
N = 5;                                     % Prediction horizon

% Model for set-point
Theta_1s = Theta_1s_mid;     % Temperatura de salida de fluido 1 (K) (set-point) (460)
Theta_2s = Theta_2s_mid;     % Temperatura de salida de fluido 2 (K)
run HE_linear;
xsp = [Theta_1s; Theta_2s; Theta_p];     % Set-point

% Constraints
xmin = [495; 662; 530];
xmax = [500; 725; 590];
umin = [90; 6];
umax = [110; 10];

% Wheight matrix
Qx = eye(nx);
% Qx = diag([1 1 1]);
%Rx = eye(nu);
Rx = diag([1 0.1]);
gamma = 1e2*diag([1e6 1 1]);

run MPC;

%% Reduced-order unknown input observer
run RUIO;
nobs = 2;
M = 9;

%% Simulation Setup
U = zeros(nu, Nsim);                   % Control Input
Ufail = zeros(nu, Nsim);               % Fault control Input
X = zeros(nx, Nsim+1);               % States
Y = zeros(ny, Nsim);                    % Measure outputs
mu = zeros(M, Nsim);                  % Membership

% Observer 1
Phi_1 = zeros(nobs, Nsim+1);     % Observer states
X_UIO1 = zeros(nx, Nsim);           % Estimated states
Error_1 = zeros(1, Nsim);             % Error
F1est = zeros(1, Nsim);                % Estimated control Input

% Observer 2
Phi_2 = zeros(nobs, Nsim+1);     % Observer states
X_UIO2 = zeros(nx, Nsim);           % Estimated states
Error_2 = zeros(1, Nsim);             % Error
F2est = zeros(1, Nsim);                % Estimated control Input

% Initial States
X(:, 1) = xsp;
Phi_1(:, 1) = xsp(1:nobs);
Phi_2(:, 1) = xsp(1:nobs);

Uff = zeros(nu, 1);
%% Simulation
for FTC = 0 % 0 - FTC is off; 1 - FTC is on

    for k = 1:Nsim

        % setpoint
        if k*Ts == 35
            xsp = x0;
        end
        
        %% Calculo el PID
        ek_1 = ek;
        ek(1) = (xsp(1) - Y(1, end));
        ek(2) = (xsp(2) - Y(2, end));
        u0(1) = 100.2766;
        u0(2) = 11.4602;
%         u0(2) = -Kr(2)*Ts*Ki(2)*ek_1(1) + Kr(2)*ek(1) + Y(1, end-1);        
        U(:, k) = [u0(1); u0(2)];

%         %% Run MPC controller       
%         [sol, diag] = mpc{X(:, k), xsp, Uff};
%         if diag
%             msg = ['Infeasible problem at t = ', num2str(k*Ts)];
%             disp(msg)
%             return;
%         end
%         U(:, k) = sol{1}; Obj = sol{2};

%         if U(1, k) < umin(1)
%             U(1, k) = umin(1);
%         elseif U(1, k) > umax(1)
%             U(1, k) = umax(1);
%         end
%         
%         if U(2, k) < umin(2)
%             U(2, k) = umin(2);
%         elseif U(2, k) > umax(2)
%             U(2, k) = umax(2);
%         end
        
        %% Fault income
        Ufail(:, k) = U(:, k);      
        Uff = [0; 0];

        if (k*Ts > 10) && (k*Ts < 20)
            Ufail(:, k) = U(:, k) + Fault_1;
            Uff = Fault_1;
        end
        
        if k*Ts > 25
            Ufail(:, k) = U(:, k) + Fault_2;
            Uff = Fault_2;
        end
        
        %% Process simulation with ODE
        [tsim, x] = ode45(@HE, [0 Ts], X(:, k), options, Ufail(:, k));
        X(:, k+1) = x(end, :)';
        Y(:, k) = C*X(:, k) + D*Ufail(:, k);
        
        %% Membership
        mu(:, k) = membership(Y(:, k), Theta_1s_min, Theta_1s_mid, Theta_1s_max, Theta_2s_min, Theta_2s_mid, Theta_2s_max);
      
        %% Observer 1
        Phi_1(:, k+1) = mu(1, k)*(H1_K1*Phi_1(:, k) + H1_L1_ast*Y(:, k) + H1_B1_bar_1*U(:, k) + H1_delta1_bar_1) ...
                              + mu(2, k)*(H1_K2*Phi_1(:, k) + H1_L2_ast*Y(:, k) + H1_B2_bar_1*U(:, k) + H1_delta2_bar_1) ...
                              + mu(3, k)*(H1_K3*Phi_1(:, k) + H1_L3_ast*Y(:, k) + H1_B3_bar_1*U(:, k) + H1_delta3_bar_1) ...
                              + mu(4, k)*(H1_K4*Phi_1(:, k) + H1_L4_ast*Y(:, k) + H1_B4_bar_1*U(:, k) + H1_delta4_bar_1) ...
                              + mu(5, k)*(H1_K5*Phi_1(:, k) + H1_L5_ast*Y(:, k) + H1_B5_bar_1*U(:, k) + H1_delta5_bar_1) ...
                              + mu(6, k)*(H1_K6*Phi_1(:, k) + H1_L6_ast*Y(:, k) + H1_B6_bar_1*U(:, k) + H1_delta6_bar_1) ...
                              + mu(7, k)*(H1_K7*Phi_1(:, k) + H1_L7_ast*Y(:, k) + H1_B7_bar_1*U(:, k) + H1_delta7_bar_1) ...
                              + mu(8, k)*(H1_K8*Phi_1(:, k) + H1_L8_ast*Y(:, k) + H1_B8_bar_1*U(:, k) + H1_delta8_bar_1) ...
                              + mu(9, k)*(H1_K9*Phi_1(:, k) + H1_L9_ast*Y(:, k) + H1_B9_bar_1*U(:, k) + H1_delta9_bar_1);

        X_UIO1(:, k) = mu(1, k)*(H1_T1*[Phi_1(:, k); H1_U1_1*Y(:, k)-H1_U1_1*H1_C1_tilde_1*Phi_1(:, k)]) ...
                            + mu(2, k)*(H1_T2*[Phi_1(:, k); H1_U2_1*Y(:, k)-H1_U2_1*H1_C2_tilde_1*Phi_1(:, k)]) ...
                            + mu(3, k)*(H1_T3*[Phi_1(:, k); H1_U3_1*Y(:, k)-H1_U3_1*H1_C3_tilde_1*Phi_1(:, k)]) ...
                            + mu(4, k)*(H1_T4*[Phi_1(:, k); H1_U4_1*Y(:, k)-H1_U4_1*H1_C4_tilde_1*Phi_1(:, k)]) ...
                            + mu(5, k)*(H1_T5*[Phi_1(:, k); H1_U5_1*Y(:, k)-H1_U5_1*H1_C5_tilde_1*Phi_1(:, k)]) ...
                            + mu(6, k)*(H1_T6*[Phi_1(:, k); H1_U6_1*Y(:, k)-H1_U6_1*H1_C6_tilde_1*Phi_1(:, k)]) ...
                            + mu(7, k)*(H1_T7*[Phi_1(:, k); H1_U7_1*Y(:, k)-H1_U7_1*H1_C7_tilde_1*Phi_1(:, k)]) ...
                            + mu(8, k)*(H1_T8*[Phi_1(:, k); H1_U8_1*Y(:, k)-H1_U8_1*H1_C8_tilde_1*Phi_1(:, k)]) ...
                            + mu(9, k)*(H1_T9*[Phi_1(:, k); H1_U9_1*Y(:, k)-H1_U9_1*H1_C9_tilde_1*Phi_1(:, k)]);

        % Error norm 1
        Error_1(k) = sqrt((X_UIO1(1, k)-Y(1, k))^2 + (X_UIO1(2, k)-Y(2, k))^2 + (X_UIO1(3, k)-Y(3, k))^2);
        
        % Fault estimation 1
        F1est(k) = mu(1, k)*(H1_U1_1*(X(:, k+1) - H1_C1_tilde_1*Phi_1(:, k+1)) + H1_A1_bar_22*H1_U1_1*(H1_C1_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A1_bar_21*Phi_1(:, k) - H1_B1_bar_2*U(:, k) -  H1_delta1_bar_2) ...
                      + mu(2, k)*(H1_U2_1*(X(:, k+1) - H1_C2_tilde_1*Phi_1(:, k+1)) + H1_A2_bar_22*H1_U2_1*(H1_C2_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A2_bar_21*Phi_1(:, k) - H1_B2_bar_2*U(:, k) -  H1_delta2_bar_2) ...
                      + mu(3, k)*(H1_U3_1*(X(:, k+1) - H1_C3_tilde_1*Phi_1(:, k+1)) + H1_A3_bar_22*H1_U3_1*(H1_C3_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A3_bar_21*Phi_1(:, k) - H1_B3_bar_2*U(:, k) -  H1_delta3_bar_2) ...
                      + mu(4, k)*(H1_U4_1*(X(:, k+1) - H1_C4_tilde_1*Phi_1(:, k+1)) + H1_A4_bar_22*H1_U4_1*(H1_C4_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A4_bar_21*Phi_1(:, k) - H1_B4_bar_2*U(:, k) -  H1_delta4_bar_2) ...
                      + mu(5, k)*(H1_U5_1*(X(:, k+1) - H1_C5_tilde_1*Phi_1(:, k+1)) + H1_A5_bar_22*H1_U5_1*(H1_C5_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A5_bar_21*Phi_1(:, k) - H1_B5_bar_2*U(:, k) -  H1_delta5_bar_2) ...
                      + mu(6, k)*(H1_U6_1*(X(:, k+1) - H1_C6_tilde_1*Phi_1(:, k+1)) + H1_A6_bar_22*H1_U6_1*(H1_C6_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A6_bar_21*Phi_1(:, k) - H1_B6_bar_2*U(:, k) -  H1_delta6_bar_2) ...                  
                      + mu(7, k)*(H1_U7_1*(X(:, k+1) - H1_C7_tilde_1*Phi_1(:, k+1)) + H1_A7_bar_22*H1_U7_1*(H1_C7_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A7_bar_21*Phi_1(:, k) - H1_B7_bar_2*U(:, k) -  H1_delta7_bar_2) ...
                      + mu(8, k)*(H1_U8_1*(X(:, k+1) - H1_C8_tilde_1*Phi_1(:, k+1)) + H1_A8_bar_22*H1_U8_1*(H1_C8_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A8_bar_21*Phi_1(:, k) - H1_B8_bar_2*U(:, k) -  H1_delta8_bar_2) ...
                      + mu(9, k)*(H1_U9_1*(X(:, k+1) - H1_C9_tilde_1*Phi_1(:, k+1)) + H1_A9_bar_22*H1_U9_1*(H1_C9_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A9_bar_21*Phi_1(:, k) - H1_B9_bar_2*U(:, k) -  H1_delta9_bar_2);

         %% Observer 2
        Phi_2(:, k+1) = mu(1, k)*(H2_K1*Phi_2(:, k) + H2_L1_ast*Y(:, k) + H2_B1_bar_1*U(:, k) + H2_delta1_bar_1) ...
                              + mu(2, k)*(H2_K2*Phi_2(:, k) + H2_L2_ast*Y(:, k) + H2_B2_bar_1*U(:, k) + H2_delta2_bar_1) ...
                              + mu(3, k)*(H2_K3*Phi_2(:, k) + H2_L3_ast*Y(:, k) + H2_B3_bar_1*U(:, k) + H2_delta3_bar_1) ...
                              + mu(4, k)*(H2_K4*Phi_2(:, k) + H2_L4_ast*Y(:, k) + H2_B4_bar_1*U(:, k) + H2_delta4_bar_1) ...
                              + mu(5, k)*(H2_K5*Phi_2(:, k) + H2_L5_ast*Y(:, k) + H2_B5_bar_1*U(:, k) + H2_delta5_bar_1) ...
                              + mu(6, k)*(H2_K6*Phi_2(:, k) + H2_L6_ast*Y(:, k) + H2_B6_bar_1*U(:, k) + H2_delta6_bar_1) ...
                              + mu(7, k)*(H2_K7*Phi_2(:, k) + H2_L7_ast*Y(:, k) + H2_B7_bar_1*U(:, k) + H2_delta7_bar_1) ...
                              + mu(8, k)*(H2_K8*Phi_2(:, k) + H2_L8_ast*Y(:, k) + H2_B8_bar_1*U(:, k) + H2_delta8_bar_1) ...
                              + mu(9, k)*(H2_K9*Phi_2(:, k) + H2_L9_ast*Y(:, k) + H2_B9_bar_1*U(:, k) + H2_delta9_bar_1);

        X_UIO2(:, k) = mu(1, k)*(H2_T1*[Phi_2(:, k); H2_U1_1*Y(:, k)-H2_U1_1*H2_C1_tilde_1*Phi_2(:, k)]) ...
                            + mu(2, k)*(H2_T2*[Phi_2(:, k); H2_U2_1*Y(:, k)-H2_U2_1*H2_C2_tilde_1*Phi_2(:, k)]) ...
                            + mu(3, k)*(H2_T3*[Phi_2(:, k); H2_U3_1*Y(:, k)-H2_U3_1*H2_C3_tilde_1*Phi_2(:, k)]) ...
                            + mu(4, k)*(H2_T4*[Phi_2(:, k); H2_U4_1*Y(:, k)-H2_U4_1*H2_C4_tilde_1*Phi_2(:, k)]) ...
                            + mu(5, k)*(H2_T5*[Phi_2(:, k); H2_U5_1*Y(:, k)-H2_U5_1*H2_C5_tilde_1*Phi_2(:, k)]) ...
                            + mu(6, k)*(H2_T6*[Phi_2(:, k); H2_U6_1*Y(:, k)-H2_U6_1*H2_C6_tilde_1*Phi_2(:, k)]) ...
                            + mu(7, k)*(H2_T7*[Phi_2(:, k); H2_U7_1*Y(:, k)-H2_U7_1*H2_C7_tilde_1*Phi_2(:, k)]) ...
                            + mu(8, k)*(H2_T8*[Phi_2(:, k); H2_U8_1*Y(:, k)-H2_U8_1*H2_C8_tilde_1*Phi_2(:, k)]) ...
                            + mu(9, k)*(H2_T9*[Phi_2(:, k); H2_U9_1*Y(:, k)-H2_U9_1*H2_C9_tilde_1*Phi_2(:, k)]);
                         
        % Error norm 2
        Error_2(k) = sqrt((X_UIO2(1, k)-Y(1, k))^2 + (X_UIO2(2, k)-Y(2, k))^2 + (X_UIO2(3, k)-Y(3, k))^2);
        
        % Fault estimation 2
        F2est(k) = mu(1, k)*(H2_U1_1*(X(:, k+1) - H2_C1_tilde_1*Phi_2(:, k+1)) + H2_A1_bar_22*H2_U1_1*(H2_C1_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A1_bar_21*Phi_2(:, k) - H2_B1_bar_2*U(:, k) -  H2_delta1_bar_2) ...
                      + mu(2, k)*(H2_U2_1*(X(:, k+1) - H2_C2_tilde_1*Phi_2(:, k+1)) + H2_A2_bar_22*H2_U2_1*(H2_C2_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A2_bar_21*Phi_2(:, k) - H2_B2_bar_2*U(:, k) -  H2_delta2_bar_2) ...
                      + mu(3, k)*(H2_U3_1*(X(:, k+1) - H2_C3_tilde_1*Phi_2(:, k+1)) + H2_A3_bar_22*H2_U3_1*(H2_C3_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A3_bar_21*Phi_2(:, k) - H2_B3_bar_2*U(:, k) -  H2_delta3_bar_2) ...
                      + mu(4, k)*(H2_U4_1*(X(:, k+1) - H2_C4_tilde_1*Phi_2(:, k+1)) + H2_A4_bar_22*H2_U4_1*(H2_C4_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A4_bar_21*Phi_2(:, k) - H2_B4_bar_2*U(:, k) -  H2_delta4_bar_2) ...
                      + mu(5, k)*(H2_U5_1*(X(:, k+1) - H2_C5_tilde_1*Phi_2(:, k+1)) + H2_A5_bar_22*H2_U5_1*(H2_C5_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A5_bar_21*Phi_2(:, k) - H2_B5_bar_2*U(:, k) -  H2_delta5_bar_2) ...
                      + mu(6, k)*(H2_U6_1*(X(:, k+1) - H2_C6_tilde_1*Phi_2(:, k+1)) + H2_A6_bar_22*H2_U6_1*(H2_C6_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A6_bar_21*Phi_2(:, k) - H2_B6_bar_2*U(:, k) -  H2_delta6_bar_2) ...                  
                      + mu(7, k)*(H2_U7_1*(X(:, k+1) - H2_C7_tilde_1*Phi_2(:, k+1)) + H2_A7_bar_22*H2_U7_1*(H2_C7_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A7_bar_21*Phi_2(:, k) - H2_B7_bar_2*U(:, k) -  H2_delta7_bar_2) ...
                      + mu(8, k)*(H2_U8_1*(X(:, k+1) - H2_C8_tilde_1*Phi_2(:, k+1)) + H2_A8_bar_22*H2_U8_1*(H2_C8_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A8_bar_21*Phi_2(:, k) - H2_B8_bar_2*U(:, k) -  H2_delta8_bar_2) ...
                      + mu(9, k)*(H2_U9_1*(X(:, k+1) - H2_C9_tilde_1*Phi_2(:, k+1)) + H2_A9_bar_22*H2_U9_1*(H2_C9_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A9_bar_21*Phi_2(:, k) - H2_B9_bar_2*U(:, k) -  H2_delta9_bar_2);
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
subplot(311)
%         plot(t, state(1, :), 'b.', tc, y(1, :), 'y-.', t, xsp(1)*ones(length(t)), 'r--', 'LineWidth', 1.5);
plot(t, xsp(1)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
hold on
plot(t, Y(1, :), 'b', 'LineWidth', 1.5);
% plot(t, xmin(1)*ones(length(t)), 'y--')
% plot(t, xmax(1)*ones(length(t)), 'y--')
% plot(t, state(1, :), 'b-.', t, x1_hat(1, :), 'g:', t, x2_hat(1, :), 'y:', t, xsp(1)*ones(length(t)), 'r--', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('\theta_1 [K]'); grid on
subplot(312)
%         plot(t, state(2, :), 'b.', tc, y(2, :), 'y-.', t, xsp(2)*ones(length(t)), 'r--', 'LineWidth', 1.5)
plot(t, xsp(2)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
hold on
h(2) = plot(t, Y(2, :), 'b', 'LineWidth', 1.5);
% plot(t, xmin(2)*ones(length(t)), 'y--')
% plot(t, xmax(2)*ones(length(t)), 'y--')
% plot(t, state(2, :), 'b-.', t, x1_hat(2, :), 'g:', t, x2_hat(2, :), 'y:', t, xsp(2)*ones(length(t)), 'r--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\theta_2 [K]'); grid on
subplot(313)
%         plot(t, state(3, :), 'b.', tc, y(3, :), 'y-.', t, xsp(3)*ones(length(t)), 'r--', 'LineWidth', 1.5)
plot(t, xsp(3)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
hold on
plot(t, Y(3, :), 'b', 'LineWidth', 1.5);   
% plot(t, xmin(3)*ones(length(t)), 'y--')
% plot(t, xmax(3)*ones(length(t)), 'y--')
% plot(t, state(3, :), 'b-.', t, x1_hat(3, :), 'g:', t, x2_hat(3, :), 'y:', t, xsp(3)*ones(length(t)), 'r--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('\theta_p [K]'); grid on

%% Error
figure
subplot(211)
stairs(t, Error_1, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|'); grid on
axis([0 inf 0 6])
subplot(212)
stairs(t, Error_2, 'b', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('|e_x|'); grid on
axis([0 inf 0 0.6])

%% Fault estimation
figure
subplot(211)
stairs(t, F1est, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(1, :) - U(1, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
axis([0 inf -10 1])
subplot(212)
stairs(t, F2est, 'b', 'LineWidth', 1.5)
hold on
stairs(t, Ufail(2, :) - U(2, :), 'm--', 'LineWidth', 1.5)
xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
axis([0 inf -0.5 1.5])

%% Membership
m = reshape(mu, 9, length(t));
fig = figure('DefaultAxesFontSize', 9, 'Color', [1 1 1]);
plot(t, m(1, :), 'Color', naranja, 'linewidth', 1.5); hold on; grid on;
plot(t, m(2, :), 'Color', azul, 'linewidth', 1.5);
plot(t, m(3, :), 'k', 'linewidth', 1.5);
plot(t, m(4, :), 'r', 'linewidth', 1.5);
plot(t, m(5, :), 'Color', forest_green, 'linewidth', 1.5);
plot(t, m(6, :), 'Color', bordo, 'linewidth', 1.5);
plot(t, m(7, :), 'm', 'linewidth', 1.5);
plot(t, m(8, :), 'Color', violeta, 'linewidth', 1.5);
plot(t, m(9, :), 'Color', amarillo, 'linewidth', 1.5); hold off;
axis([0 inf 0 1]);
xlabel('Tiempo [min]'); ylabel('\mu_i'); yticks([0 0.2 0.4 0.6 0.8 1]); yticklabels({'0', '0,2', '0,4', '0,6', '0,8', '1'});
pbaspect([2 1 1]);
leg = legend('\mu_1', '\mu_2', '\mu_3', '\mu_4', '\mu_5', '\mu_6', '\mu_7', '\mu_8', '\mu_9', 'Location', 'East');
set(leg, 'Position', [0.697 0.325 0.077 0.383], 'FontSize', 8);
leg.ItemTokenSize = [20, 18];
