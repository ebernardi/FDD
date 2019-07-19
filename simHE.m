%% HE
clc; clear; yalmip('clear');
close all;

%% Simulation parameters
Time = 50;                              % Simulation end time 
Ts = 0.05;                               % Sample time [min]
Nsim = Time/Ts;                     % Simulation steps
t = 0:Ts:Time-Ts;                    % Simulation time
Fault_1 = [0; 1]; Fault_2 = [-8; 0];          % Fault magnitude

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
Thi_1(:, 1) = xsp(1:nobs);
Phi_2(:, 1) = xsp(1:nobs);

Uff = zeros(nu, 1);
%% Simulation
for FTC = 0 % 0 - FTC is off; 1 - FTC is on

    for k = 1:Nsim
        
        if k*Ts == 35
            xsp = xsp + [1; 0; 0];
        end
        
        % Run MPC controller       
        [sol, diag] = mpc{X(:, k), xsp, Uff};
        if diag
            msg = ['Infeasible problem at t = ', num2str(k*Ts)];
            disp(msg)
            return;
        end
        
        U(:, k) = sol{1}; Obj = sol{2};
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
        Phi_1(:, k+1) = mu(1)*(H1_K1*Phi_1(:, k) + H1_L1_ast*Y(:, k) + H1_B1_bar_1*U(:, k) + H1_delta1_bar_1) ...
                              + mu(2)*(H1_K2*Phi_1(:, k) + H1_L2_ast*Y(:, k) + H1_B2_bar_1*U(:, k) + H1_delta2_bar_1) ...
                              + mu(3)*(H1_K3*Phi_1(:, k) + H1_L3_ast*Y(:, k) + H1_B3_bar_1*U(:, k) + H1_delta3_bar_1) ...
                              + mu(4)*(H1_K4*Phi_1(:, k) + H1_L4_ast*Y(:, k) + H1_B4_bar_1*U(:, k) + H1_delta4_bar_1) ...
                              + mu(5)*(H1_K5*Phi_1(:, k) + H1_L5_ast*Y(:, k) + H1_B5_bar_1*U(:, k) + H1_delta5_bar_1) ...
                              + mu(6)*(H1_K6*Phi_1(:, k) + H1_L6_ast*Y(:, k) + H1_B6_bar_1*U(:, k) + H1_delta6_bar_1) ...
                              + mu(7)*(H1_K7*Phi_1(:, k) + H1_L7_ast*Y(:, k) + H1_B7_bar_1*U(:, k) + H1_delta7_bar_1) ...
                              + mu(8)*(H1_K8*Phi_1(:, k) + H1_L8_ast*Y(:, k) + H1_B8_bar_1*U(:, k) + H1_delta8_bar_1) ...
                              + mu(9)*(H1_K9*Phi_1(:, k) + H1_L9_ast*Y(:, k) + H1_B9_bar_1*U(:, k) + H1_delta9_bar_1);

        X_UIO1(:, k) = mu(1)*(H1_T1*[Phi_1(:, k); H1_U1_1*Y(:, k)-H1_U1_1*H1_C1_tilde_1*Phi_1(:, k)]) ...
                            + mu(2)*(H1_T2*[Phi_1(:, k); H1_U2_1*Y(:, k)-H1_U2_1*H1_C2_tilde_1*Phi_1(:, k)]) ...
                            + mu(3)*(H1_T3*[Phi_1(:, k); H1_U3_1*Y(:, k)-H1_U3_1*H1_C3_tilde_1*Phi_1(:, k)]) ...
                            + mu(4)*(H1_T4*[Phi_1(:, k); H1_U4_1*Y(:, k)-H1_U4_1*H1_C4_tilde_1*Phi_1(:, k)]) ...
                            + mu(5)*(H1_T5*[Phi_1(:, k); H1_U5_1*Y(:, k)-H1_U5_1*H1_C5_tilde_1*Phi_1(:, k)]) ...
                            + mu(6)*(H1_T6*[Phi_1(:, k); H1_U6_1*Y(:, k)-H1_U6_1*H1_C6_tilde_1*Phi_1(:, k)]) ...
                            + mu(7)*(H1_T7*[Phi_1(:, k); H1_U7_1*Y(:, k)-H1_U7_1*H1_C7_tilde_1*Phi_1(:, k)]) ...
                            + mu(8)*(H1_T8*[Phi_1(:, k); H1_U8_1*Y(:, k)-H1_U8_1*H1_C8_tilde_1*Phi_1(:, k)]) ...
                            + mu(9)*(H1_T9*[Phi_1(:, k); H1_U9_1*Y(:, k)-H1_U9_1*H1_C9_tilde_1*Phi_1(:, k)]);

        % Error norm 1
        Error_1(k) = sqrt((X_UIO1(1, k)-Y(1, k))^2 + (X_UIO1(2, k)-Y(2, k))^2 + (X_UIO1(3, k)-Y(3, k))^2);
        
        % Fault estimation 1
        F1est(k) = mu(1)*(H1_U1_1*(X(:, k+1) - H1_C1_tilde_1*Phi_1(:, k+1)) + H1_A1_bar_22*H1_U1_1*(H1_C1_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A1_bar_21*Phi_1(:, k) - H1_B1_bar_2*U(:, k) -  H1_delta1_bar_2) ...
                      + mu(2)*(H1_U2_1*(X(:, k+1) - H1_C2_tilde_1*Phi_1(:, k+1)) + H1_A2_bar_22*H1_U2_1*(H1_C2_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A2_bar_21*Phi_1(:, k) - H1_B2_bar_2*U(:, k) -  H1_delta2_bar_2) ...
                      + mu(3)*(H1_U3_1*(X(:, k+1) - H1_C3_tilde_1*Phi_1(:, k+1)) + H1_A3_bar_22*H1_U3_1*(H1_C3_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A3_bar_21*Phi_1(:, k) - H1_B3_bar_2*U(:, k) -  H1_delta3_bar_2) ...
                      + mu(4)*(H1_U4_1*(X(:, k+1) - H1_C4_tilde_1*Phi_1(:, k+1)) + H1_A4_bar_22*H1_U4_1*(H1_C4_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A4_bar_21*Phi_1(:, k) - H1_B4_bar_2*U(:, k) -  H1_delta4_bar_2) ...
                      + mu(5)*(H1_U5_1*(X(:, k+1) - H1_C5_tilde_1*Phi_1(:, k+1)) + H1_A5_bar_22*H1_U5_1*(H1_C5_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A5_bar_21*Phi_1(:, k) - H1_B5_bar_2*U(:, k) -  H1_delta5_bar_2) ...
                      + mu(6)*(H1_U6_1*(X(:, k+1) - H1_C6_tilde_1*Phi_1(:, k+1)) + H1_A6_bar_22*H1_U6_1*(H1_C6_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A6_bar_21*Phi_1(:, k) - H1_B6_bar_2*U(:, k) -  H1_delta6_bar_2) ...                  
                      + mu(7)*(H1_U7_1*(X(:, k+1) - H1_C7_tilde_1*Phi_1(:, k+1)) + H1_A7_bar_22*H1_U7_1*(H1_C7_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A7_bar_21*Phi_1(:, k) - H1_B7_bar_2*U(:, k) -  H1_delta7_bar_2) ...
                      + mu(8)*(H1_U8_1*(X(:, k+1) - H1_C8_tilde_1*Phi_1(:, k+1)) + H1_A8_bar_22*H1_U8_1*(H1_C8_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A8_bar_21*Phi_1(:, k) - H1_B8_bar_2*U(:, k) -  H1_delta8_bar_2) ...
                      + mu(9)*(H1_U9_1*(X(:, k+1) - H1_C9_tilde_1*Phi_1(:, k+1)) + H1_A9_bar_22*H1_U9_1*(H1_C9_tilde_1*Phi_1(:, k) - Y(:, k)) - H1_A9_bar_21*Phi_1(:, k) - H1_B9_bar_2*U(:, k) -  H1_delta9_bar_2);

         %% Observer 2
        Phi_2(:, k+1) = mu(1)*(H2_K1*Phi_2(:, k) + H2_L1_ast*Y(:, k) + H2_B1_bar_1*U(:, k) + H2_delta1_bar_1) ...
                              + mu(2)*(H2_K2*Phi_2(:, k) + H2_L2_ast*Y(:, k) + H2_B2_bar_1*U(:, k) + H2_delta2_bar_1) ...
                              + mu(3)*(H2_K3*Phi_2(:, k) + H2_L3_ast*Y(:, k) + H2_B3_bar_1*U(:, k) + H2_delta3_bar_1) ...
                              + mu(4)*(H2_K4*Phi_2(:, k) + H2_L4_ast*Y(:, k) + H2_B4_bar_1*U(:, k) + H2_delta4_bar_1) ...
                              + mu(5)*(H2_K5*Phi_2(:, k) + H2_L5_ast*Y(:, k) + H2_B5_bar_1*U(:, k) + H2_delta5_bar_1) ...
                              + mu(6)*(H2_K6*Phi_2(:, k) + H2_L6_ast*Y(:, k) + H2_B6_bar_1*U(:, k) + H2_delta6_bar_1) ...
                              + mu(7)*(H2_K7*Phi_2(:, k) + H2_L7_ast*Y(:, k) + H2_B7_bar_1*U(:, k) + H2_delta7_bar_1) ...
                              + mu(8)*(H2_K8*Phi_2(:, k) + H2_L8_ast*Y(:, k) + H2_B8_bar_1*U(:, k) + H2_delta8_bar_1) ...
                              + mu(9)*(H2_K9*Phi_2(:, k) + H2_L9_ast*Y(:, k) + H2_B9_bar_1*U(:, k) + H2_delta9_bar_1);

        X_UIO2(:, k) = mu(1)*(H2_T1*[Phi_2(:, k); H2_U1_1*Y(:, k)-H2_U1_1*H2_C1_tilde_1*Phi_2(:, k)]) ...
                            + mu(2)*(H2_T2*[Phi_2(:, k); H2_U2_1*Y(:, k)-H2_U2_1*H2_C2_tilde_1*Phi_2(:, k)]) ...
                            + mu(3)*(H2_T3*[Phi_2(:, k); H2_U3_1*Y(:, k)-H2_U3_1*H2_C3_tilde_1*Phi_2(:, k)]) ...
                            + mu(4)*(H2_T4*[Phi_2(:, k); H2_U4_1*Y(:, k)-H2_U4_1*H2_C4_tilde_1*Phi_2(:, k)]) ...
                            + mu(5)*(H2_T5*[Phi_2(:, k); H2_U5_1*Y(:, k)-H2_U5_1*H2_C5_tilde_1*Phi_2(:, k)]) ...
                            + mu(6)*(H2_T6*[Phi_2(:, k); H2_U6_1*Y(:, k)-H2_U6_1*H2_C6_tilde_1*Phi_2(:, k)]) ...
                            + mu(7)*(H2_T7*[Phi_2(:, k); H2_U7_1*Y(:, k)-H2_U7_1*H2_C7_tilde_1*Phi_2(:, k)]) ...
                            + mu(8)*(H2_T8*[Phi_2(:, k); H2_U8_1*Y(:, k)-H2_U8_1*H2_C8_tilde_1*Phi_2(:, k)]) ...
                            + mu(9)*(H2_T9*[Phi_2(:, k); H2_U9_1*Y(:, k)-H2_U9_1*H2_C9_tilde_1*Phi_2(:, k)]);
                         
        % Error norm 2
        Error_2(k) = sqrt((X_UIO2(1, k)-Y(1, k))^2 + (X_UIO2(2, k)-Y(2, k))^2 + (X_UIO2(3, k)-Y(3, k))^2);
        
        % Fault estimation 2
        F2est(k) = mu(1)*(H2_U1_1*(X(:, k+1) - H2_C1_tilde_1*Phi_2(:, k+1)) + H2_A1_bar_22*H2_U1_1*(H2_C1_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A1_bar_21*Phi_2(:, k) - H2_B1_bar_2*U(:, k) -  H2_delta1_bar_2) ...
                      + mu(2)*(H2_U2_1*(X(:, k+1) - H2_C2_tilde_1*Phi_2(:, k+1)) + H2_A2_bar_22*H2_U2_1*(H2_C2_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A2_bar_21*Phi_2(:, k) - H2_B2_bar_2*U(:, k) -  H2_delta2_bar_2) ...
                      + mu(3)*(H2_U3_1*(X(:, k+1) - H2_C3_tilde_1*Phi_2(:, k+1)) + H2_A3_bar_22*H2_U3_1*(H2_C3_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A3_bar_21*Phi_2(:, k) - H2_B3_bar_2*U(:, k) -  H2_delta3_bar_2) ...
                      + mu(4)*(H2_U4_1*(X(:, k+1) - H2_C4_tilde_1*Phi_2(:, k+1)) + H2_A4_bar_22*H2_U4_1*(H2_C4_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A4_bar_21*Phi_2(:, k) - H2_B4_bar_2*U(:, k) -  H2_delta4_bar_2) ...
                      + mu(5)*(H2_U5_1*(X(:, k+1) - H2_C5_tilde_1*Phi_2(:, k+1)) + H2_A5_bar_22*H2_U5_1*(H2_C5_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A5_bar_21*Phi_2(:, k) - H2_B5_bar_2*U(:, k) -  H2_delta5_bar_2) ...
                      + mu(6)*(H2_U6_1*(X(:, k+1) - H2_C6_tilde_1*Phi_2(:, k+1)) + H2_A6_bar_22*H2_U6_1*(H2_C6_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A6_bar_21*Phi_2(:, k) - H2_B6_bar_2*U(:, k) -  H2_delta6_bar_2) ...                  
                      + mu(7)*(H2_U7_1*(X(:, k+1) - H2_C7_tilde_1*Phi_2(:, k+1)) + H2_A7_bar_22*H2_U7_1*(H2_C7_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A7_bar_21*Phi_2(:, k) - H2_B7_bar_2*U(:, k) -  H2_delta7_bar_2) ...
                      + mu(8)*(H2_U8_1*(X(:, k+1) - H2_C8_tilde_1*Phi_2(:, k+1)) + H2_A8_bar_22*H2_U8_1*(H2_C8_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A8_bar_21*Phi_2(:, k) - H2_B8_bar_2*U(:, k) -  H2_delta8_bar_2) ...
                      + mu(9)*(H2_U9_1*(X(:, k+1) - H2_C9_tilde_1*Phi_2(:, k+1)) + H2_A9_bar_22*H2_U9_1*(H2_C9_tilde_1*Phi_2(:, k) - Y(:, k)) - H2_A9_bar_21*Phi_2(:, k) - H2_B9_bar_2*U(:, k) -  H2_delta9_bar_2);
    end
end

%% Set Plots
vecrojo = [0.7; 0; 0]; vecverde = [0; 0.8; 0]; vecazul = [0; 0; 0.6]; negro = [.1; .1; .1]; gris = [.5; .7; .5];

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

% figure
% subplot(3, 1, 1)
% plot(t, Y(1, :));
% hold on
% plot(t, X_UIO1(1, :), 'o');
% plot(t, X_UIO2(1, :), '.');
% hold off; grid on;
% subplot(3, 1, 2)
% plot(t, Y(2, :));
% hold on
% plot(t, X_UIO1(2, :), 'o');
% plot(t, X_UIO2(2, :), '.');
% hold off; grid on;
% subplot(3, 1, 3)
% plot(t, Y(3, :));
% hold on
% plot(t, X_UIO1(3, :), 'o');
% plot(t, X_UIO2(3, :), '.');
% hold off; grid on;
% legend('X', 'X_{obs 1}', 'X_{obs 2}');
%     tc = 0:Ts/10:Nsim/20;
%     
%     % Outputs
%     figure(1)
%     if FTC == 0
%         subplot(311)
% %         plot(t, state(1, :), 'b.', tc, y(1, :), 'y-.', t, xsp(1)*ones(length(t)), 'r--', 'LineWidth', 1.5);
%         plot(t, xsp(1)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
%         hold on
%         plot(tc, y(1, :), 'b', 'LineWidth', 1.5);
%         % plot(t, xmin(1)*ones(length(t)), 'y--')
%         % plot(t, xmax(1)*ones(length(t)), 'y--')
%         % plot(t, state(1, :), 'b-.', t, x1_hat(1, :), 'g:', t, x2_hat(1, :), 'y:', t, xsp(1)*ones(length(t)), 'r--', 'LineWidth', 1.5);
%         xlabel('Time [min]'); ylabel('\theta_1 [K]'); grid on
%         subplot(312)
% %         plot(t, state(2, :), 'b.', tc, y(2, :), 'y-.', t, xsp(2)*ones(length(t)), 'r--', 'LineWidth', 1.5)
%         plot(t, xsp(2)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
%         hold on
%         h(2) = plot(tc, y(2, :), 'b', 'LineWidth', 1.5);
%         % plot(t, xmin(2)*ones(length(t)), 'y--')
%         % plot(t, xmax(2)*ones(length(t)), 'y--')
%         % plot(t, state(2, :), 'b-.', t, x1_hat(2, :), 'g:', t, x2_hat(2, :), 'y:', t, xsp(2)*ones(length(t)), 'r--', 'LineWidth', 1.5)
%         xlabel('Time [min]'); ylabel('\theta_2 [K]'); grid on
%         subplot(313)
% %         plot(t, state(3, :), 'b.', tc, y(3, :), 'y-.', t, xsp(3)*ones(length(t)), 'r--', 'LineWidth', 1.5)
%         plot(t, xsp(3)*ones(1, length(t)), 'r:', 'LineWidth', 1.5);
%         hold on
%         plot(tc, y(3, :), 'b', 'LineWidth', 1.5);   
%         % plot(t, xmin(3)*ones(length(t)), 'y--')
%         % plot(t, xmax(3)*ones(length(t)), 'y--')
%         % plot(t, state(3, :), 'b-.', t, x1_hat(3, :), 'g:', t, x2_hat(3, :), 'y:', t, xsp(3)*ones(length(t)), 'r--', 'LineWidth', 1.5)
%         xlabel('Time [min]'); ylabel('\theta_p [K]'); grid on
%     else
%         subplot(311)
% %         plot(t, state(1, :), 'k.', tc, y(1, :), 'g-.', 'LineWidth', 1.5);
%         plot(tc, y(1, :), 'k-.', 'LineWidth', 1.5);
%         hold off
%         legend('x_s', 'Location', 'SouthEast');
%         legend boxoff
%         subplot(312)
% %         plot(t, state(2, :), 'k.', tc, y(2, :), 'g-.', 'LineWidth', 1.5);
%         plot(tc, y(2, :), 'k-.', 'LineWidth', 1.5);        
%         hold off
%         legend(h(2), 'MPC', 'Location', 'SouthEast');
%         legend boxoff
%         subplot(313)
% %         plot(t, state(3, :), 'k.', tc, y(3, :), 'g-.', 'LineWidth', 1.5);
%         h(3) = plot(tc, y(3, :), 'k-.', 'LineWidth', 1.5);
%         hold off
%         legend(h(3), 'FTMPC', 'Location', 'NorthEast');
%         legend boxoff
%         pause(2)
%         print -dsvg outputHE.svg
%     end
%     % print -dsvg outputCSTR.svg
% 
% %     % Inputs
% %     figure(2)
% %     if FTC == 0
% %         subplot(211)
% %         stairs(t, input(1, :), 'b', 'LineWidth', 1.5)
% %         hold on
% %         xlabel('Time [min]'); ylabel('q_1 [l/min]'); grid on
% %         subplot(212)
% %         stairs(t, input(2, :), 'b', 'LineWidth', 1.5)
% %         hold on
% %         xlabel('Time [min]'); ylabel('q_2 [l/min]'); grid on
% % 	else
% %         subplot(211)
% %         stairs(t, input(1, :), 'k-.', 'LineWidth', 1.5)
% %         plot(t, u_min(1, :), 'r--')
% %         plot(t, u_max(1, :), 'r--')
% %         hold off
% %         legend('MPC', 'FTMPC', 'Location', 'NorthWest');
% %         legend boxoff
% %         subplot(212)
% %         stairs(t, input(2, :), 'k-.', 'LineWidth', 1.5)
% %         plot(t, u_min(2, :), 'r--')
% %         plot(t, u_max(2, :), 'r--')
% %         hold off
% %         pause(2)
% %         print -dsvg inputHE.svg
% %     end
%     % print -dsvg inputCSTR.svg
% 
% %     % Failure inputs
% %     figure(3)
% %     if FTC == 0
% %         subplot(211)
% %         plot(t, ufail_min(1, :), 'r--')
% %         hold on
% %         plot(t, ufail_max(1, :), 'r--')
% %         stairs(t, ufail(1, :), 'b', 'LineWidth', 1.5)
% %         xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
% %         subplot(212)
% %         plot(t, ufail_min(2, :), 'r--')
% %         hold on
% %         plot(t, ufail_max(2, :), 'r--')
% %         stairs(t, ufail(2, :), 'b', 'LineWidth', 1.5)
% %         xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
% % 	else
% %         subplot(211)
% %         stairs(t, ufail(1, :), 'k-.', 'LineWidth', 1.5)
% %         hold off
% %         subplot(212)
% %         stairs(t, ufail(2, :), 'k-.', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg inputfailCSTR.svg    
% % 
% %     % Estimation error
% %     figure(4)
% %     if FTC == 0
% %         subplot(211)
% %         plot(t, error1, 'b', 'LineWidth', 1.5)
% %         hold on; grid on
% %         xlabel('Time [min]'); ylabel('|e_x|');
% %         subplot(212)
% %         plot(t, error2, 'b', 'LineWidth', 1.5)
% %         hold on; grid on        
% %         xlabel('Time [min]'); ylabel('|e_x|');
% % 	else
% %         subplot(211)
% %         plot(t, error1, 'k-.', 'LineWidth', 1.5)
% %         plot(t, 0.1*ones(length(t)),  'r--', 'LineWidth', 1.5)
% %         hold off
% %         legend('MPC', 'FTMPC', 'Threshold', 'Location', 'NorthEast');
% %         legend boxoff
% %         subplot(212)
% %         plot(t, error2, 'k-.', 'LineWidth', 1.5)
% %         plot(t, 0.1*ones(length(t)),  'r--', 'LineWidth', 1.5)
% %         hold off
% %         pause(2)
% %         print -dsvg errorHE.svg
% %     end
%     % print -dsvg errorCSTR.svg
% 
% %     % Fault estimation
% %     figure(5)
% %     if FTC == 0
% %         subplot(211)
% %         stairs(t, f1est, 'b', 'LineWidth', 1.5)
% %         hold on
% %         stairs(t, ufail(1, :) - input(1, :), 'm--', 'LineWidth', 1.5)
% %         xlabel('Time [min]'); ylabel('Q_1 [l/min]'); grid on
% %         subplot(212)
% %         stairs(t, f2est, 'b', 'LineWidth', 1.5)
% %         hold on
% %         stairs(t, ufail(2, :) - input(2, :), 'm--', 'LineWidth', 1.5)
% %         xlabel('Time [min]'); ylabel('Q_2 [l/min]'); grid on
% %     else
% %         subplot(211)
% %         stairs(t, f1est, 'k-.', 'LineWidth', 1.5)
% %         hold off
% %         subplot(212)
% %         stairs(t, f2est, 'k-.', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg estimationCSTR.svg
% 
% 
% %     % Objective
% %     figure(6)
% %     if FTC == 0
% %         plot(t, obj, 'b', 'LineWidth', 1.5)
% %         hold on
% %         xlabel('Time [min]'); ylabel('Objective'); grid on
% %     else
% %         plot(t, obj, 'k-.', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg objectiveCSTR.svg
%     
% %     % State evolution
% %     figure(7)
% %     if FTC == 0
% %         plot3(x0(1), x0(2), x0(3), 'g*', 'LineWidth', 1.5);
% %         hold on
% %         plot3(y(1, :), y(2, :), y(3, :), 'y', 'LineWidth', 1.5)
% %         plot3(state(1, :), state(2, :), state(3, :), 'b', 'LineWidth', 1.5)
% %         plot3(state(1, end), state(2, end), state(3, end), 'mo', 'LineWidth', 1.5)
% %         plot3(xsp(1), xsp(2), xsp(3), 'rp', 'LineWidth', 1.5)
% %         plot(X+X_lin, 'Color', vecrojo, 'Alpha', 0.05, 'edgecolor', vecrojo, 'linestyle', '--', 'LineWidth', 1.5)
% %         plot(Xs+X_lin, 'Color', gris, 'Alpha', 0.2, 'edgecolor', gris, 'linestyle', '--', 'LineWidth', 1.5)
% %         xlabel('\theta_1 [K]'); ylabel('\theta_2 [K]'); zlabel('\theta_3 [K]'); grid on;
% %     else
% %         plot3(y(1, :), y(2, :), y(3, :), 'g', 'LineWidth', 1.5)        
% %         plot3(state(1, :), state(2, :), state(3, :), 'k-.', 'LineWidth', 1.5)
% %         plot3(state(1, end), state(2, end), state(3, end), 'ro', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg stateCSTR.svg
%     
% %     % State evolution
% %     Xx = X.projection(1:2).minHRep();
% %     Xxs = Xs.projection(1:2).minHRep();
% % 
% %     figure(8)
% %     if FTC == 0
% %         plot(x0(1), x0(2), 'g*', 'LineWidth', 1.5);
% %         hold on
% %         plot(y(1, :), y(2, :), 'y', 'LineWidth', 1.5)        
% %         plot(state(1, :), state(2, :), 'b.', 'LineWidth', 1.5)
% %         plot(state(1, end), state(2, end), 'mo', 'LineWidth', 1.5)
% %         plot(xsp(1), xsp(2), 'rp', 'LineWidth', 1.5)
% %         plot(Xxs+X_lin(1:2), 'Color', gris, 'Alpha', 0.2, 'edgecolor', gris, 'linestyle', '--', 'LineWidth', 1.5)
% %         plot(Xx+X_lin(1:2), 'Color', vecrojo, 'Alpha', 0.05, 'edgecolor', vecrojo, 'linestyle', '--', 'LineWidth', 1.5)
% %         xlabel('\theta_1 [K]'); ylabel('\theta_2 [K]'); grid on;
% %     else
% %         plot(y(1, :), y(2, :), 'g', 'LineWidth', 1.5)        
% %         plot(state(1, :), state(2, :), 'k.', 'LineWidth', 1.5)
% %         plot(state(1, end), state(2, end), 'ro', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg stateCSTR.svg
% %     
% %     % State evolution
% %     Xx = X.projection(1:2:3).minHRep();
% %     Xxs = Xs.projection(1:2:3).minHRep();
% % 
% %     figure(9)
% %     if FTC == 0
% %         plot(x0(1), x0(3), 'gd', 'LineWidth', 1.5);
% %         hold on
% %         plot(y(1, :), y(3, :), 'b', 'LineWidth', 1.5)
% % %         plot(state(1, :), state(3, :), 'b.', 'LineWidth', 1.5)
% %         plot(state(1, end), state(3, end), 'mo', 'LineWidth', 1.5)
% %         xlabel('\theta_1 [K]'); ylabel('\theta_3 [K]'); grid on;
% %     else
% %         plot(y(1, :), y(3, :), 'k--', 'LineWidth', 1.5)
% % %         plot(state(1, :), state(3, :), 'k.', 'LineWidth', 1.5)
% %         plot(state(1, end), state(3, end), 'y*', 'LineWidth', 1.5)
% %         plot(xsp(1), xsp(3), 'ro', 'LineWidth', 1.5)
% %         plot(Xxs+X_lin(1:2:3), 'Color', gris, 'Alpha', 0.2, 'edgecolor', gris, 'LineWidth', 1.5)
% %         plot(Xx+X_lin(1:2:3), 'Color', vecrojo, 'Alpha', 0.05, 'edgecolor', vecrojo, 'LineWidth', 1.5)
% %         hold off
% %         leg = legend('$x(0)$', '$x_{MPC}$', '$x_{MPC}(end)$', '$x_{FTMPC}$', '$x_{FTMPC}(end)$', '$x_s$', '$\bf{X}_s$', '$\bf{X}$', 'Location', 'SouthEast');
% %         set(leg, 'Interpreter', 'latex');
% %         pause(2)
% %         print -dsvg stateHE.svg
% %     end
% %     % print -dsvg stateCSTR.svg
% %     
% %     % State evolution
% %     Xx = X.projection(2:3).minHRep();
% %     Xxs = Xs.projection(2:3).minHRep();
% % 
% %     figure(10)
% %     if FTC == 0
% %         plot(x0(2), x0(3), 'g*', 'LineWidth', 1.5);
% %         hold on
% %         plot(y(2, :), y(3, :), 'y', 'LineWidth', 1.5)
% %         plot(state(2, :), state(3, :), 'b.', 'LineWidth', 1.5)
% %         plot(state(2, end), state(3, end), 'mo', 'LineWidth', 1.5)
% %         plot(xsp(2), xsp(3), 'rp', 'LineWidth', 1.5)
% %         plot(Xxs+X_lin(2:3), 'Color', gris, 'Alpha', 0.2, 'edgecolor', gris, 'linestyle', '--', 'LineWidth', 1.5)
% %         plot(Xx+X_lin(2:3), 'Color', vecrojo, 'Alpha', 0.05, 'edgecolor', vecrojo, 'linestyle', '--', 'LineWidth', 1.5)
% %         xlabel('\theta_2 [K]'); ylabel('\theta_3 [K]'); grid on;
% %     else
% %         plot(y(2, :), y(3, :), 'g', 'LineWidth', 1.5)
% %         plot(state(2, :), state(3, :), 'k.', 'LineWidth', 1.5)
% %         plot(state(2, end), state(3, end), 'ro', 'LineWidth', 1.5)
% %         hold off
% %     end
% %     % print -dsvg stateCSTR.svg    
% end